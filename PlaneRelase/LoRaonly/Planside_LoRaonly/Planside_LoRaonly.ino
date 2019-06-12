#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <string.h>
#include <math.h>

#define outpin 13 //PPM
#define deployment_judge_pin 

#define SLEEP 0
#define MANUAL 1
#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define GUIDED 4

SoftwareSerial SerialMavlink(10, 11); //Pixhawkと接続
SoftwareSerial LoRa(2,3); //LoRaのSS

//loopで何回も宣言するのが嫌だからグローバル宣言
//Cdsセルでの機体開放は未実装(冗長系に使いたい)、231行
//LoRaはプロセッサーモードを使う

int PPMMODE_MANUAL[8] = {0,0,0,0,165,0,0,0};
int PPMMODE_STABILIZENOSEUP[8] = {0,400,0,0,425,0,0,0}; //傾きがひどいと流れが剥離して全然効かなくなるかも、と思ったので400に抑えている。
int PPMMODE_STABILIZE[8] = {0,0,0,0,425,0,0,0};
int PPMMODE_GUIDED[8] = {0,0,0,0,815,0,0,0};

void setup()
{
  	SerialMavlink.begin(57600); //RXTX from Pixhawk
  	Serial.begin(57600); //Main serial port for console output
  	LoRa.begin(19200);
  	pinMode(outpin,OUTPUT);
  	request_datastream();
    EEPROM.write(0,0);
}

void loop()
{
  	int ch[8];
	int i; //for文のループ数
    int plane_condition = EEPROM.read(0);

  	switch (plane_condition) {
    	case SLEEP: //溶断開始判定を受け取るまで
      		for(i = 0;i < 8;++i){
        		ch[i]=PPMMODE_MANUAL[i];
      		}
      		for(int i = 0;i < 10;++i){
        	    PPM_Transmit(ch);
      		}
            LoRa_recv(ch); //中でPPMも送ってあげる
            EEPROM.write(0,STABILIZE_NOSEUP);
            plane_condition = STABILIZE_NOSEUP;
      	break;

		case STABILIZE_NOSEUP:
			for(i = 0;i < 8;++i){
				ch[i] = PPMMODE_GUIDED[i];
			}
            PPM_Transmit(ch);
            LoRa.print("cutoff\r");
            for(i = 0;i < 15;++i){ //200ms*15より、 3秒間はPPMを送る
                PPM_Transmit(ch);
            }
            //ここまでに2回目溶断は終わっているはず
            EEPROM.write(0,STABILIZE);
            plane_condition = STABILIZE;
		break;

    	case STABILIZE://カットオフ後
            for(i = 0;i < 8;++i){
        		ch[i]=PPMMODE_STABILIZE[i];
      		}
            PPM_Transmit(ch);
            stabilize_func(ch); //時間による冗長系が欲しかったので、stabilize_func()を作った、これが終わったらstabilize終了
            EEPROM.write(0,GUIDED);
        	plane_condition = GUIDED;
      	break;

    	case GUIDED://離陸判定後
      		for(i=0;i<8;i++){
        		ch[i] = PPMMODE_GUIDED[i];
      		}
            for(i= 0;i < 10;++i){
                PPM_Transmit(ch); //Guided確定
            }
            MavLink_receive_GPS_and_send_with_LoRa();
            delay(1000); //あまり高頻度のGPS送るにしてもなぁ...(多分この後に一番最後の機構が入る。)
      	break;

    	default:
      	break;
  	}
}

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
float MavLink_receive()
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

void MavLink_receive_GPS_and_send_with_LoRa()
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
                    LoRa.print("Lat: ");LoRa.println(packet.lat);
                    LoRa.print("Long: ");LoRa.println(packet.lon);
                    LoRa.print("Alt; ");LoRa.println(packet.alt);
                    LoRa.print("Speed: ");LoRa.println(packet.vel);
                }
                break;
            }
            return;
        }
    }
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
    int time_temp_1 = millis();
    int time_temp_2;
    while(true){
        pitch_angle = MavLink_receive();
        if(-45<pitch_angle&&pitch_angle<45){
            return;
        }else{
            time_temp_2 = millis();
            if(time_temp_2 - time_temp_1 > 10000){ //MavLinkが取れなくて、永遠にstabilizeにいるのに留まるのを防ぐ
                return; //MavLinkの問題では無く、そもそもPixhawk本体が死んでたらそれはもうどうしようもない...
            }else{
                PPM_Transmit(ch); //stabilizeを続けるためにPPMを送る。
                continue;
            }
        }
    } //breakは無いが、returnで戻るようになっている。
}

void LoRa_recv(int ch[8]){
    char buf[128];
    char* temp; //開始を記憶
    while(true){ //ここの無限ループどうにかしたい
        temp = buf;
        while (LoRa.available() > 0){
            *temp++ = LoRa.read();
            if (*(temp-1) == '\r') {
                *temp = '\0';
                break;
            }
        }
        if(strstr(buf, "cutoff") != NULL){
            return; //cutoffが入っていたので帰れる
        }else{
            PPM_Transmit(ch); //MANUAL続行
            continue; //まだなのでcutoffが来るまで待つ。
        }
    }
}
