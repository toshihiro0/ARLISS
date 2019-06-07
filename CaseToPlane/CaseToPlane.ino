/******************************************************************
    MAVLinkAndPPM.ino

    MAVLinkでピッチ角を取得して、絶対値が45度以内ならフライトモードを変更する.
    関数名などの変更をしているので注意.

    The circuit:
    *MAVLink:D10,D11
    *PPM:D13

    Created 2019/6/3
    By Toshihiro Suzuki

    https://github.com/toshihiro0/ARLISS

******************************************************************/

#include <mavlink.h>
#include <SoftwareSerial.h>
#include <string.h>

#define outpin 13

#define M_PI 3.14159

int plane_condition;
#define SLEEP 0
#define MANUAL 1
#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define GUIDED 4

SoftwareSerial SerialMavlink(10, 11);
SoftwareSerial LoRa_ss(2,3);

void setup() {
  SerialMavlink.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port for console output
  LoRa_ss.begin(19200);
  pinMode(outpin,OUTPUT);

  request_datastream();

}

void loop() {
  int ch[8];
  int PPMMODE_MANUAL[8]={0,0,0,0,165,0,0,0};
  int PPMMODE_STABILIZE[8]={0,0,0,0,425,0,0,0};
  int PPMMODE_GUIDED[8]={0,0,0,0,815,0,0,0};
  char buf[128];


  float pitch_angle;

  switch (plane_condition) {
    case SLEEP: //溶断開始判定を受け取るまで
      for(int i=0;i<8;i++){
        ch[i]=PPMMODE_MANUAL[i];
      }
      for(int i=0;i<10;i++){
        PPM_Transmit(ch);
      }
      LoRa_recv(buf);
      if(strstr(buf,"cut off")!=NULL){
        plane_condition=STABILIZE;
      }
      break;

    case STABILIZE://カットオフ後
      pitch_angle=MavLink_receive();
      for(int i=0;i<8;i++){
        ch[i]=PPMMODE_STABILIZE[i];
      }
      if(-45<pitch_angle&&pitch_angle<45){
        plane_condition=GUIDED;
      }
      break;

    case GUIDED://離陸判定後
      for(int i=0;i<8;i++){
        ch[i]=PPMMODE_GUIDED[i];
      }
      break;

    default:
      break;

  }

  PPM_Transmit(ch);

}

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
float MavLink_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  while(SerialMavlink.available())
  {
    uint8_t c= SerialMavlink.read();

    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

    //Handle new message from autopilot
      switch(msg.msgid)
      {

        case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(&msg, &packet);
        return(packet.pitch/M_PI*180.0);
      }break;
      }
    }
  }
}

void request_datastream() {
//Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x03; //number of times per second to request the data in hex
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

void OnePulth(int PPMtime){
  digitalWrite(outpin,HIGH);
  delayMicroseconds(250);
  digitalWrite(outpin,LOW);
  delayMicroseconds(750+PPMtime);
}

void PPM_Transmit(int ch[8]){
  int ppmWaitTimeSum=0;

  for(int i=0;i<8;i++){
    ppmWaitTimeSum += ch[i]+1000;
  }

  for(int i=0;i<8;i++){
     OnePulth(ch[i]);
  }

  OnePulth(20000-ppmWaitTimeSum);
}

int LoRa_recv(char *buf) {
    char *start = buf;

    while (true) {
        delay(0);
        while (LoRa_ss.available() > 0) {
            *buf++ = LoRa_ss.read();
            if (*(buf-2) == '\r' && *(buf-1) == '\n') {
                *buf = '\0';
                return (buf - start);
            }
        }
      
    }
}
