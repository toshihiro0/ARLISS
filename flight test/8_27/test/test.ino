#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

#define outpin 3 //PPM
#define deploy_judge_pin_INPUT1  10 //一段階目溶断の抜けピン
#define deploy_judge_pin_INPUT2  9 //二段階目溶断の抜けピン
#define LoRa_sw 6 //MOSFETのスイッチピン
#define LoRa_rst 18 //LoRaのリセット
#define LoRa_RX 17
#define LoRa_TX 19

#define SLEEP 0
#define TRAINING 1
#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define AUTO 4
#define DEEPSTALL 5

#define goal_latitude 35.7954471
#define goal_longtitude 139.8910135
#define goal_altitude 5.0
#define difference_lat 111316.2056

static const float difference_lon = cos(goal_latitude/180*M_PI)*M_PI*6378.137/180*1000;

SoftwareSerial LoRa(LoRa_RX,LoRa_TX); //LoRaと接続、PixhawkはSerialでつなぐ。

int EEPROM_Address = 1;
unsigned long time_auto_zero = 0;//オートが始まった最初の時刻を格納
unsigned long time_auto = 0;//オートが始まってからの経過時間を格納
int LoRa_send_Mode = 0; //LoRaでどれを送るか決める。

unsigned long time_deploy2_start;
unsigned long time_deploy2_end;

int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要
int PPMMODE_MANUAL[8] = {500,500,0,500,100,500,500,0}; //ケースに入ってる間
int PPMMODE_TRAINING[8] = {500,500,0,500,300,500,500,0}; //ピン抜け1回後に入る。
int PPMMODE_STABILIZE_NOSEUP[8] = {500,900,0,500,425,500,500,0}; //900側が機首上げ
int PPMMODE_STABILIZE[8] = {500,500,300,500,425,500,500,0}; //300から徐々に上げる
int PPMMODE_AUTO[8] = {500,500,0,500,815,500,500,0};
int PPMMODE_DEEPSTALL[8] = {500,900,0,500,425,500,500,0}; //900側がエレベーター上げ

void setup()
{
    pinMode(outpin,OUTPUT);
    pinMode(deploy_judge_pin_INPUT1,INPUT_PULLUP);
    pinMode(deploy_judge_pin_INPUT2,INPUT_PULLUP);

    pinMode(LoRa_sw,OUTPUT);  //LoRaの通信on
    digitalWrite(LoRa_sw,HIGH);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    LoRa.begin(19200); //LoRaとの通信開始
    Serial.begin(57600); //Pixhawkとの通信

    request_datastream(); //データ吸出し

    EEPROM.write(0,0);
    int i,j;

    while(digitalRead(deploy_judge_pin_INPUT2) == HIGH){}

    for(i = 0;i < 300;++i){ //アーム
        PPM_Transmit(PPMMODE_Arm);
    }
}

void loop()
{
    int i,j;
    int plane_condition = EEPROM.read(0); //再起動用に読み出し
    switch (plane_condition){
        case SLEEP: //溶断開始判定を受け取るまで
            EEPROM.write(EEPROM_Address,SLEEP); //ログ残し用
            ++EEPROM_Address;

            for(i = 0;i < 10;++i){ //モード確定
                PPM_Transmit(PPMMODE_TRAINING);
            }

            while(true){
                if(digitalRead(deploy_judge_pin_INPUT2) == HIGH){
                    delay(3000); //抜いていきなり回るのは怖いので、delay入れる
                    EEPROM.write(0,STABILIZE); //再起動しても大丈夫なように、先に書き込んでおきたい
                    plane_condition = STABILIZE;
                    break;
                }else{
                    PPM_Transmit(PPMMODE_TRAINING);//ここで一応、TRAININGのPPMを送っておく
                    continue; //いちいち宣言したくなかったので、whileに突っ込んだ
                }
            }
        break;

        case TRAINING:
            EEPROM.write(EEPROM_Address,TRAINING); //ログ残し用
            ++EEPROM_Address;
            
            for(i = 0;i < 100;++i){ //2*1000/20 = 100 加速2秒間
                PPM_Transmit(PPMMODE_TRAINING);
            }

            EEPROM.write(0,STABILIZE_NOSEUP);
            plane_condition = STABILIZE_NOSEUP;
        break;

        case STABILIZE_NOSEUP:
            EEPROM.write(EEPROM_Address,2); //ログ残し用
            ++EEPROM_Address;

            for(i = 0;i <= 100;++i){ //2*1000/20 = 100、強制機首上げ2秒間
                PPM_Transmit(PPMMODE_STABILIZE_NOSEUP);
            }

            EEPROM.write(0,STABILIZE); //次に遷移
            plane_condition = STABILIZE;
        break;

        case STABILIZE:
            EEPROM.write(EEPROM_Address,3); //ログ残し用
            ++EEPROM_Address;

            for(i = 3;i <= 9;++i){
                PPMMODE_STABILIZE[2] = i*100;
                for(j = 0;j < 14;++j){
                    PPM_Transmit(PPMMODE_STABILIZE); //7*14*20 = 1960で2秒間かけてプロペラ回転
                }
            }

            for(i = 0;i < 250;++i){ //5*1000/20 = 250より、5秒間Stablizeで加速する。
                PPM_Transmit(PPMMODE_STABILIZE);
            }

            EEPROM.write(0,AUTO); //次に遷移
            plane_condition = AUTO;
        break;

        case AUTO://離陸判定後、仕様変更あり
            EEPROM.write(EEPROM_Address,4); //ログ残し用
            ++EEPROM_Address;

            for(i = 0;i < 10;++i){//AUTO確定
                PPM_Transmit(PPMMODE_AUTO);
            }
            
            time_auto_zero = millis();
            MavLink_receive_GPS_and_send_with_LoRa_and_detect_waypoint(); //AUTOの間はここにいる。

            EEPROM.write(0,DEEPSTALL); //次に遷移
            plane_condition = DEEPSTALL;
        break;

        case DEEPSTALL:
            EEPROM.write(EEPROM_Address,5); //ログ残し用
            ++EEPROM_Address;

            while(true){ //ずっと
                MavLink_receive_GPS_and_send_with_LoRa_Deep_Stall();
                PPM_Transmit(PPMMODE_DEEPSTALL); //DEEPSTALL確定
            }
            
        break;

        default:
        break;
    }
}

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
/*float MavLink_receive_attitude() //使わないけど...
{
    mavlink_message_t msg;
    mavlink_status_t status;
    while(Serial.available()){ //取れなかったら多分while素通り(試したい)
        uint8_t c = Serial.read();
        //Get new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
            //Handle new message from autopilot
            switch(msg.msgid){
                case MAVLINK_MSG_ID_ATTITUDE:{
                    mavlink_attitude_t packet;
                    mavlink_msg_attitude_decode(&msg, &packet);
                    return(packet.pitch/M_PI*180.0);
                }break;
            }
        }
    }
    return 90.0; //whileが取れなかった時に応じて、Stabilizeを続ける返り値を返してあげる。
}*/

void request_datastream()
{
//Request Data from Pixhawk
    uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
    uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
    uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
    uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
    uint8_t _req_stream_id = MAV_DATA_STREAM_ALL; //変えても早くはならない
    uint16_t _req_message_rate = 0x03; //number of times per second to request the data in hex //体感3が一番早い
    uint8_t _start_stop = 1; //1 = start, 0 = stop

// STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
   *
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   *
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

    Serial.write(buf, len); //Write data to serial port
}

void MavLink_receive_GPS_and_send_with_LoRa_and_detect_waypoint() //使わないけど...
{
    int i;
    mavlink_message_t msg;
    mavlink_status_t status;
    float latitude,longtitude,altitude,distance;
    boolean waypoint_near_flag;
    while(true){
        while(Serial.available()){ //通信出来てなかったら...悲しいね...
            uint8_t c= Serial.read();
            //Get new message
            if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
                //Handle new message from autopilot
                switch(msg.msgid){
                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                    {
                        mavlink_gps_raw_int_t packet;
                        mavlink_msg_gps_raw_int_decode(&msg, &packet);
                        latitude = packet.lat/1e7;
                        longtitude = packet.lon/1e7;
                        altitude = packet.alt/1e3;
                        distance = calculate_distance(latitude,longtitude);
                        if(distance < 10.0){
                            EEPROM.write(EEPROM_Address,6);
                            ++EEPROM_Address;

                            record_deep_stall_point(latitude,longtitude,altitude);
                            
                            return;
                        }
                        if(LoRa_send_Mode == 0){
                            LoRa.print("Lat:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_AUTO);}
                            LoRa.println(latitude*1e7);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_AUTO);}
                        }else if(LoRa_send_Mode == 1){
                            LoRa.print("Long:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_AUTO);}
                            LoRa.println(longtitude*1e7);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_AUTO);}
                        }else if(LoRa_send_Mode == 2){
                            LoRa.print("Alt:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_AUTO);}
                            LoRa.println(altitude*1e3);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_AUTO);}
                        }else if(LoRa_send_Mode == 3){
                            LoRa.print("Distance:");for(i = 0;i < 5;++i){PPM_Transmit(PPMMODE_AUTO);}
                            LoRa.println(distance);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_AUTO);}
                        }
                        ++LoRa_send_Mode;
                        if(LoRa_send_Mode == 4){
                            LoRa_send_Mode = 0;
                        }
                    }//ここまで540ms
                    break;
                }
            }
        }
        time_auto = millis(); //通信出来てなかったらずっとここにいる。
        if((time_auto - time_auto_zero) > 180000){
            return;
        }
        PPM_Transmit(PPMMODE_AUTO);
    }
}

void OnePulth(int PPMtime)
{
    digitalWrite(outpin,HIGH);
    delayMicroseconds(250);
    digitalWrite(outpin,LOW);
    delayMicroseconds(750+PPMtime);
}

void PPM_Transmit(int ch[8])
{
    int ppmWaitTimeSum=0;

    for(int i=0;i<8;i++){
        ppmWaitTimeSum += ch[i]+1000;
    }

    for(int i=0;i<8;i++){
        OnePulth(ch[i]);
    }

    OnePulth(19000-ppmWaitTimeSum);
}

/*void stabilize_func(int ch[8]) //使わないけど...
{
    float pitch_angle;
    long time_temp_1 = millis();
    long time_temp_2;
    while(true){
        pitch_angle = MavLink_receive_attitude();
        if(-45<pitch_angle && pitch_angle<45){
            return;
        }else{
            time_temp_2 = millis();
            if(time_temp_2 - time_temp_1 > 5000){ //MavLinkが取れなくて、永遠にstabilizeにいるのに留まるのを防ぐ 5秒間
                return; //MavLinkの問題では無く、そもそもPixhawk本体が死んでたらそれはもうどうしようもない...
            }else{
                PPM_Transmit(ch); //stabilizeを続けるためにPPMを送る。
                continue;
            }
        }
    } //breakは無いが、returnで戻るようになっている。
}*/

void record_deep_stall_point(float latitude,float longtitude,float altitude)
{
    EEPROM.put(EEPROM_Address,latitude);
    EEPROM_Address += 4;
    EEPROM.put(EEPROM_Address,longtitude);
    EEPROM_Address += 4;
    EEPROM.put(EEPROM_Address,altitude);
    EEPROM_Address += 4;
    return;
}

void MavLink_receive_GPS_and_send_with_LoRa_Deep_Stall() //使わないけど...
{
    LoRa_send_Mode = 0;
    int i;
    mavlink_message_t msg;
    mavlink_status_t status;
    float latitude,longtitude,altitude,velocity,distance;
    while(true){
        while(Serial.available()){ //通信出来てなかったら...悲しいね...
            uint8_t c= Serial.read();
            //Get new message
            if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
                //Handle new message from autopilot
                switch(msg.msgid){
                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                    {
                        mavlink_gps_raw_int_t packet;
                        mavlink_msg_gps_raw_int_decode(&msg, &packet);
                        latitude = packet.lat/1e7;
                        longtitude = packet.lon/1e7;
                        altitude = packet.alt/1e3;
                        distance = calculate_distance(latitude,longtitude);
                        if(LoRa_send_Mode == 0){
                            LoRa.print("Lat:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                            LoRa.println(latitude*1e7);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                        }else if(LoRa_send_Mode == 1){
                            LoRa.print("Long:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                            LoRa.println(longtitude*1e7);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                        }else if(LoRa_send_Mode == 2){
                            LoRa.print("Alt:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                            LoRa.println(altitude*1e3);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                        }else if(LoRa_send_Mode == 3){
                            LoRa.print("Distance:");for(i = 0;i < 5;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                            LoRa.println(distance);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_DEEPSTALL);}
                        }
                        ++LoRa_send_Mode;
                        if(LoRa_send_Mode == 4){
                            LoRa_send_Mode = 0;
                        }
                        return;
                    }//ここまで640ms
                    break;
                }
            }
        }
        PPM_Transmit(PPMMODE_DEEPSTALL);
    }
}

float calculate_distance(float latitude,float longtitude)
{
    float distance = sqrt((latitude-goal_latitude)*(latitude-goal_latitude)*difference_lat*difference_lat+(longtitude-goal_longtitude)*(longtitude-goal_longtitude)*difference_lon*difference_lon);
    return distance;
}
