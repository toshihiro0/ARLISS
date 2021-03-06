#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

#define outpin 18 //PPM

#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define AUTO 4
#define DEEP_STALL 5

SoftwareSerial SerialMavlink(17,16); //Pixhawkと接続

int EEPROM_Address = 1;

//loopで何回も宣言するのが嫌だからグローバル宣言
int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要
int PPMMODE_STABILIZENOSEUP[8] = {500,900,0,500,425,500,500,0}; //E900側が機首上げ
int PPMMODE_STABILIZE[8] = {500,500,300,500,425,500,500,0}; //徐々に加速
int PPMMODE_AUTO[8] = {500,500,0,500,815,500,500,0}; //Auto飛行
int PPMMODE_DEEPSTALL[8] = {500,900,0,500,425,500,500,0}; //900側が機首上げ

void setup()
{
    pinMode(14,INPUT_PULLUP);
    while(digitalRead(14) == HIGH){}
    SerialMavlink.begin(57600); //RXTX from Pixhawk
    
  	pinMode(outpin,OUTPUT);

  	request_datastream();
    EEPROM.write(0,2);
    for(int i = 0;i <= 300;++i){
        PPM_Transmit(PPMMODE_Arm);
    }
}

void loop()
{
  	int ch[8]; //これ要らない説ある

	int i,j; //for文のループ数
    int plane_condition = EEPROM.read(0);

  	switch (plane_condition){
        
		case STABILIZE_NOSEUP:

            EEPROM.write(EEPROM_Address,2); //ログ残し用
            ++EEPROM_Address;

			for(i = 0;i < 8;++i){
				ch[i] = PPMMODE_STABILIZENOSEUP[i];
			}
            for(i = 0;i < 10;++i){
                PPM_Transmit(ch);
            }
            stabilize_func(ch);
            EEPROM.write(0,STABILIZE);
            plane_condition = STABILIZE;
		break;

        case STABILIZE:

            EEPROM.write(EEPROM_Address,3); //ログ残し用
            ++EEPROM_Address;

            for(i = 3;i <= 9;++i){
                PPMMODE_STABILIZE[2] = i*100; 
                for(j = 0;j < 14;++j){
                    PPM_Transmit(PPMMODE_STABILIZE);
                }
            }

            for(i=0;i<8;i++){
        		ch[i] = PPMMODE_STABILIZE[i];
      		}
            for(i = 0;i < 500;++i){ //10*1000/20 = 500より、10秒間Stablizeで加速する。
                PPM_Transmit(ch);
            }

            EEPROM.write(0,AUTO); //次に遷移
            plane_condition = AUTO;

        break;

    	case AUTO://離陸判定後
            EEPROM.write(EEPROM_Address,4); //ログ残し用
            ++EEPROM_Address;

      		for(i=0;i<8;i++){ //モード確定
        		ch[i] = PPMMODE_AUTO[i];
      		}
            for(i= 0;i < 6000;++i){ //2分間、だから2*60*1000/20 = 6000
                PPM_Transmit(ch); //AUTO確定
            }

            EEPROM.write(0,DEEP_STALL); //次に遷移
            plane_condition = DEEP_STALL;

      	break;
        
        case DEEP_STALL:
            EEPROM.write(EEPROM_Address,5); //ログ残し用
            ++EEPROM_Address;

            for(i = 0;i < 9;++i){ //次に遷移
                ch[i] = PPMMODE_DEEPSTALL[i];
            }
            while(true){
                PPM_Transmit(ch);
            }

        break;

    	default:
      	break;
  	}
}

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
float MavLink_receive_attitude()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    while(SerialMavlink.available()){ //取れなかったら多分while素通り(試したい)
        uint8_t c = SerialMavlink.read();
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
}

/*void MavLink_receive_GPS_and_send_with_LoRa()
{
    mavlink_message_t msg;
    mavlink_status_t status;
 
    while(SerialMavlink.available()){
        uint8_t c= SerialMavlink.read();
        //Get new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
            //Handle new message from autopilot
            switch(msg.msgid){
                case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    mavlink_gps_raw_int_t packet;
                    mavlink_msg_gps_raw_int_decode(&msg, &packet);
                    Serial.print("Lat:");delay(100);Serial.println(packet.lat);delay(100);
                    Serial.print("Long:");delay(100);Serial.println(packet.lon);delay(100);
                    Serial.print("Alt:");delay(100);Serial.println(packet.alt);delay(100);
                    Serial.print("Speed:");delay(100);Serial.println(packet.vel);delay(100);
                }
                break;
            }
            return;
        }
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

    SerialMavlink.write(buf, len); //Write data to serial port
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

    OnePulth(20000-ppmWaitTimeSum);
}

void stabilize_func(int ch[8])
{
    float pitch_angle;
    int i;
    while(true){
        pitch_angle = MavLink_receive_attitude();
        if(-45<pitch_angle && pitch_angle<45){
            for(i = EEPROM_Address;i < EEPROM_Address+4;++i){
                EEPROM.write(i,0);
            }
            EEPROM.put(EEPROM_Address,pitch_angle);
            EEPROM_Address += 4;
            return;
        }else{
            PPM_Transmit(ch); //stabilizeを続けるためにPPMを送る。
            continue;
        }
    } //breakは無いが、returnで戻るようになっている。
}
