#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

#define inpin 7  //D7、PPMエンコーダの信号線に繋ぐ
#define deploy_judge_pin_INPUT  12 //D12、抜けピン(初期状態ではGNDに挿さっていて、抜けたら制御開始)
#define LoRa_rst 14 //A0、LoRaのリセット
#define CH1_PIN 15  //A1、RCレシーバのCh6から直接配線する
#define outpin 18  //A4、PixhawkのRCの信号線に直接配線する
#define LoRa_sw 19 //A5、LoRaに電源供給する

#define SLEEP 0
#define MANUAL 1
#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define AUTO 4
#define DEEPSTALL 5

SoftwareSerial LoRa(17,16); //LoRaと接続、PixhawkはSerialでつなぐ。

int CH1_value = 0;//プロポのスイッチA入力のPWM値を格納
int rc_ppm;//PPM信号の入力を格納
int EEPROM_Address = 1;
unsigned long int time_auto_zero = 0;//オートが始まった最初の時刻を格納
unsigned long int time_auto = 0;//オートが始まってからの経過時間を格納

int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要
int PPMMODE_MANUAL[8] = {500,500,0,500,165,500,500,0};
int PPMMODE_STABILIZE_NOSEUP[8] = {500,900,0,500,425,500,500,0}; //900側が機首上げ
int PPMMODE_STABILIZE[8] = {500,500,300,500,425,500,500,0}; //300から徐々に上げる
int PPMMODE_AUTO[8] = {500,500,0,500,815,500,500,0};
int PPMMODE_DEEPSTALL[8] = {500,900,0,500,425,500,500,0}; //900側がエレベーター上げ

float pointlat = 0;
float pointlon = 0;

void setup()
{
    pinMode(CH1_PIN, INPUT);
    pinMode(outpin,OUTPUT);
    pinMode(inpin,INPUT);
    pinMode(deploy_judge_pin_INPUT,INPUT_PULLUP);
    while(digitalRead(deploy_judge_pin_INPUT) == HIGH){}
    delay(3000);
    Serial.begin(57600);
    request_datastream();
    EEPROM.write(0,0);
    for(int i = 0;i <= 300;++i){ //アーム
        PPM_Transmit(PPMMODE_Arm);
    }
}

void loop() 
{
    CH1_value = pulseIn(CH1_PIN,HIGH);
    if(CH1_value<1500){//スイッチAが上に上がっている状態だとArduino制御する、オート時のみプロポ制御への遷移を許可
      Serial.println(CH1_value);
      int plane_condition = EEPROM.read(0); //再起動用に読み出し

      switch (plane_condition) {
        case SLEEP: //溶断開始判定を受け取るまで
          EEPROM.write(EEPROM_Address,0); //ログ残し用
          ++EEPROM_Address;
          for(int i = 0;i < 10;++i){
            PPM_Transmit(PPMMODE_MANUAL);
          }
          while(true){
            if(digitalRead(deploy_judge_pin_INPUT) == HIGH){
              EEPROM.write(0,STABILIZE); //再起動しても大丈夫なように、先に書き込んでおきたい
              plane_condition = STABILIZE;
              break;
            }else{
              PPM_Transmit(PPMMODE_MANUAL);//ここで一応、マニュアルのPPMを送っておく
              continue; //いちいち宣言したくなかったので、whileに突っ込んだ
            }
          }

       case MANUAL:
          EEPROM.write(EEPROM_Address,1); //ログ残し用
          ++EEPROM_Address;
          for(int i = 0;i < 150;++i){ //加速3秒間
            PPM_Transmit(PPMMODE_MANUAL);
          }
          EEPROM.write(0,STABILIZE_NOSEUP);
          plane_condition = STABILIZE_NOSEUP;
          break;

        case STABILIZE_NOSEUP:
          EEPROM.write(EEPROM_Address,2); //ログ残し用
          ++EEPROM_Address;

          for(int i = 0;i <= 100;++i){ //2*1000/20 = 100、強制機首上げ2秒間
            PPM_Transmit(PPMMODE_STABILIZE_NOSEUP);
          }

          EEPROM.write(0,STABILIZE); //次に遷移
          plane_condition = STABILIZE;
	        break;

        case STABILIZE://カットオフ後
          EEPROM.write(EEPROM_Address,3); //ログ残し用
          ++EEPROM_Address;
          for(int i = 3;i <= 9;++i){
            PPMMODE_STABILIZE[2] = i*100;
            for(int j = 0;j < 14;++j){
                PPM_Transmit(PPMMODE_STABILIZE); //7*14*20 = 1960で2秒間かけてプロペラ回転
            }
          }
          for(int i = 0;i < 250;++i){ //5*1000/20 = 250より、5秒間Stablizeで加速する。
            PPM_Transmit(PPMMODE_STABILIZE);
          }
          EEPROM.write(0,AUTO); //次に遷移
        	plane_condition = AUTO;
          break;

        case AUTO://離陸判定後、仕様変更あり
          EEPROM.write(EEPROM_Address,4); //ログ残し用
          ++EEPROM_Address;
          for(int i = 0;i < 10;++i){//AUTO確定
            PPM_Transmit(PPMMODE_AUTO);
          }
          if(time_auto_zero == 0){//初めてオートに入った時刻を記録
            time_auto_zero = millis();
            break;
          }else{//2回目以降のループでは、他の変数に時刻を記録
            time_auto = millis();
            if(when_to_deepstall()){
                EEPROM.write(0,DEEPSTALL); //次に遷移
                plane_condition = DEEPSTALL;
                break;
            }
            if(time_auto - time_auto_zero > 120000){//2分間
              EEPROM.write(0,DEEPSTALL); //次に遷移
              plane_condition = DEEPSTALL;
              break;
            }else{
              PPM_Transmit(PPMMODE_AUTO);
              break;
            }
          }

        case DEEPSTALL:
          EEPROM.write(EEPROM_Address,5); //次に遷移
          ++EEPROM_Address;
          while(true){ //ずっと
            PPM_Transmit(PPMMODE_DEEPSTALL); //AUTO確定
          }

        default:
          break;
      }

    }else{//スイッチAを下に倒すとプロポ操作に移行
      while(true){//一度プロポ操作モードに入ったらArduino操作には戻れない仕様になっているので、書き方は変えた方がいいかもしれない(プロポ信号のパススルーと他の処理を並列させると、動作しなくなる)
        rc_ppm = digitalRead(inpin);
        digitalWrite(outpin,rc_ppm);
      }
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

/*void MavLink_receive_GPS_and_send_with_LoRa() //使わないけど...
{
    int i;
    mavlink_message_t msg;
    mavlink_status_t status;

    while(Serial.available()){
        uint8_t c= Serial.read();
        //Get new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
            //Handle new message from autopilot
            switch(msg.msgid){
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    mavlink_gps_raw_int_t packet;
                    mavlink_msg_gps_raw_int_decode(&msg, &packet);

                    LoRa.print("Lat:");
                    for(i = 0;i < 5;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };
                    LoRa.println(packet.lat);
                    for(i = 0;i < 25;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };

                    LoRa.print("Long:");
                    for(i = 0;i < 5;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };
                    LoRa.println(packet.lon);
                    for(i = 0;i < 25;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };

                    LoRa.print("Alt:");
                    for(i = 0;i < 5;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };
                    LoRa.println(packet.alt);
                    for(i = 0;i < 25;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };

                    LoRa.print("Speed:");
                    for(i = 0;i < 5;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    };
                    LoRa.println(packet.vel);
                    for(i = 0;i < 25;++i){
                        PPM_Transmit(PPMMODE_AUTO);
                    }; //ここまで、600ms*3 = 1.8s
                }
                break;
            }
            return;
        }
        return; //GPSが取れても取れなくても、returnを返す。(Pixhawkと繋がないこともある。)
    }
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

/*void mission_count()
{
  //Step #1 of uploading a new waypoint
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)
 
  uint16_t count = 2; // How many items to upload (HOME coordinates are always the first way-point)
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_mission_count_pack(_system_id, _component_id, &msg, _target_system, _target_component, count);
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t count
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

void MavLink_receive()
{ 
  	mavlink_message_t msg;
  	mavlink_status_t status;
 
  	while(Serial.available()){
    	uint8_t c= Serial.read();
 
    	//Get new message
    	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
 		//Handle new message from autopilot
      	switch(msg.msgid){
      
      // Step 2 uploading a new waypoint - Check for mission replies
      case MAVLINK_MSG_ID_MISSION_REQUEST:
      {
        mavlink_mission_request_t missionreq;
        mavlink_msg_mission_request_decode(&msg, &missionreq);
 
        if (missionreq.seq == 0) {
        create_home();
        }
 
        if (missionreq.seq == 1) {
        create_waypoint1();
        }
      }
      break;
 
      case MAVLINK_MSG_ID_MISSION_ACK:
      // Step 4 uploading a new waypoint - Receive Mission Ack Message
      {
       mavlink_mission_ack_t missionack;
       mavlink_msg_mission_ack_decode(&msg, &missionack);  
      }
      break;
      
      }
    }
  }
}

void create_home() {
  //Step 3 of uploading a new waypoint (send HOME coordinates)
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)
 
  uint16_t seq = 0; // Sequence number
  uint8_t frame = 0; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4
  uint8_t current = 0; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle
  float x = homex; // Latitude - degrees
  float y = homey;// Longitude - degrees
  float z = homez; // Altitude - meters
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
  
}
 
void create_waypoint1() {
  //Step 3 continuation of uploading a new waypoint (send 1st coordinates)
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)
 
  uint16_t seq = 1; // Sequence number
  uint8_t frame = 0; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4
  uint8_t current = 1; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 5; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle
  float x = pointx; // Latitude - degrees
  float y = pointy; // Longitude - degrees
  float z = pointz; // Altitude - meters
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}*/

boolean when_to_deepstall() //使わないけど...
{
    int i;
    mavlink_message_t msg;
    mavlink_status_t status;

    while(Serial.available()){
        uint8_t c= Serial.read();
        //Get new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
            //Handle new message from autopilot
            switch(msg.msgid){
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    mavlink_gps_raw_int_t packet;
                    mavlink_msg_gps_raw_int_decode(&msg, &packet);
                    float distance;
                    distance = pow((packet.lat-pointlat)*(packet.lat-pointlat)*111000.0*111000.0+(packet.lon-pointlon)*(packet.lon-pointlon)*91000.0*91000.0,0.5);
                    if(distance <= 5){
                        return true;
                    }else{
                        return false;
                    }
                }
                break;
            }
            return false;
        }
        PPM_Transmit(PPMMODE_AUTO);
        return false; //GPSが取れても取れなくても、returnを返す。
    }
}
