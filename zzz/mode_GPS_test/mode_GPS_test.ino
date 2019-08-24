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

#define goal_latitude 35.7133315
#define goal_longtitude 139.7618383
#define goal_altitude 5.0
#define difference_lat 111316.2056

static const float difference_lon = cos(goal_latitude/180*M_PI)*M_PI*6378.137/180*1000;

SoftwareSerial LoRa(LoRa_RX,LoRa_TX); //LoRaと接続、PixhawkはSerialでつなぐ。

int EEPROM_Address = 1;
unsigned long int time_auto_zero = 0;//オートが始まった最初の時刻を格納
unsigned long int time_auto = 0;//オートが始まってからの経過時間を格納
int LoRa_send_Mode = 0; //LoRaでどれを送るか決める。

int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要
int PPMMODE_MANUAL[8] = {500,500,0,500,100,500,500,0}; //ケースに入ってる間
int PPMMODE_TRAINING[8] = {500,500,0,500,300,500,500,0}; //ピン抜け1回後に入る。
int PPMMODE_STABILIZE_NOSEUP[8] = {500,900,0,500,425,500,500,0}; //900側が機首上げ
int PPMMODE_STABILIZE[8] = {500,500,0,500,425,500,500,0}; //300から徐々に上げる
int PPMMODE_AUTO[8] = {500,500,0,500,815,500,500,0};
int PPMMODE_DEEPSTALL[8] = {500,900,0,500,425,500,500,0}; //900側がエレベーター上げ

long time1,time2;

void setup()
{
    pinMode(outpin,OUTPUT);
    pinMode(LoRa_sw,OUTPUT);  //LoRaの通信on
    digitalWrite(LoRa_sw,HIGH);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    LoRa.begin(19200); //LoRaとの通信開始
    delay(2000);

    Serial.begin(57600); //Pixhawkとの通信
    request_datastream(); //データ吸出し
    time1 = millis();
}

void loop()
{
    int i;
    for(i = 0;i < 10;++i){ //AUTO0.2秒間
        PPM_Transmit(PPMMODE_STABILIZE);
    }
    MavLink_receive_GPS_and_send_with_LoRa_and_detect_waypoint();
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
                        if(LoRa_send_Mode == 0){
                            LoRa.print("Lat:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                            LoRa.println(latitude);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                        }else if(LoRa_send_Mode == 1){
                            LoRa.print("Long:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                            LoRa.println(longtitude);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                        }else if(LoRa_send_Mode == 2){
                            LoRa.print("Alt:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                            LoRa.println(altitude);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                        }else if(LoRa_send_Mode == 3){
                            distance = calculate_distance(latitude,longtitude);
                            LoRa.print("Distance:");for(i = 0;i < 2;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                            LoRa.println(distance);for(i = 0;i < 30;++i){PPM_Transmit(PPMMODE_STABILIZE);}
                        }
                        ++LoRa_send_Mode;
                        if(LoRa_send_Mode == 4){
                            LoRa_send_Mode = 0;
                        }
                        return;
                    }//ここまで540ms
                }
            }
        }
        time2 = millis();
        if((time2-time1) > 30000){
            while(true){
                PPM_Transmit(PPMMODE_AUTO);
            }
        }
        PPM_Transmit(PPMMODE_STABILIZE);
    }
}

float calculate_distance(float latitude,float longtitude)
{
    float distance = sqrt((latitude-goal_latitude)*(latitude-goal_latitude)*difference_lat*difference_lat+(longtitude-goal_longtitude)*(longtitude-goal_longtitude)*difference_lon*difference_lon);
    return distance;
}
