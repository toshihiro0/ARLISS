/*************************************************
 * rc_Arduino_rc.ino
 * 2019/7/29 作成者鈴木
 * 2019/7/31 更新者鈴木 ボタンがなくなったので、ボタンでアームのシーケンスを時間制御に変更
 * 2019/8/14 更新者鈴木 RCで制御→Arduinoで制御→RCで制御というシーケンスに変更。松戸での飛行試験を想定
 * 操作方法が難しくなってしまったのでここにメモしておく
 * そもそも抜けピンは必要ないので挿さない
 * プログラム開始時にはスイッチAをARD側に倒しておき、スイッチCはスタビライズにしておくこと。
 * 投げた後は、できるだけ高度を稼ぐように飛行させる。プログラム開始から45秒経過で、自動的にArduino制御に移行
 * オート飛行中は、スイッチAを下に倒すとプロポ操作に移行することが可能。
 *
 * test3を改変、抜けピン後オート飛行中にスイッチA押下げでプロポ飛行に遷移
 *************************************************/

#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

#define inpin 7  //D7、PPMエンコーダの信号線に繋ぐ。PD7としてrc制御しているので書き換えるならそこも書き換える
#define deploy_judge_pin_INPUT  12 //D12、抜けピン(初期状態ではGNDに挿さっていて、抜けたら制御開始)
#define LoRa_rst 14 //A0、LoRaのリセット
#define CH1_PIN 15  //A1、RCレシーバのCh6から直接配線する
#define outpin 18  //A4、PixhawkのRCの信号線に直接配線する。PC4としてrc制御しているので書き換えるならそこも書き換える
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

void setup() {
  pinMode(CH1_PIN, INPUT);
  pinMode(outpin,OUTPUT);
  pinMode(inpin,INPUT);
  pinMode(deploy_judge_pin_INPUT,INPUT_PULLUP);
  delay(15000);//15秒待てば操作可能状態にはなりそう
  Serial.begin(19200);

  pinMode(LoRa_sw,OUTPUT);
  digitalWrite(LoRa_sw,HIGH);
  pinMode(LoRa_rst,OUTPUT);
  digitalWrite(LoRa_rst,HIGH);

   request_datastream();
   EEPROM.write(0,4);//最初からオートに移行するようにしておく
   for(int i = 0;i <= 300;++i){ //アーム
     PPM_Transmit(PPMMODE_Arm);
   }

   while(millis()<45000){//プログラム開始から45秒間はrc入力で飛行する(アームまでに最低15秒間使っているから、そんなに長くはない)。極限まで処理のサイクル数を減らさないとパススルー出来なかったので、このような書き方になっている。
     rc_ppm = PIND & _BV(7);
     if(rc_ppm == 0){
       PORTC &= ~_BV(PC4);
     }else{
       PORTC |= _BV(PC4);
     }
   }

}

void loop() {
    CH1_value = pulseIn(CH1_PIN,HIGH);
    if(CH1_value<1500){//スイッチAが上に上がっている状態だとArduino制御する、オート時のみプロポ制御への遷移を許可
      Serial.println(CH1_value);
      int plane_condition = EEPROM.read(0); //再起動用に読み出し

      switch (plane_condition) {

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
            if(time_auto - time_auto_zero > 30000){//30秒間
              EEPROM.write(0,DEEPSTALL); //次に遷移
              plane_condition = DEEPSTALL;
              break;
            }else{
              MavLink_receive_GPS_and_send_with_LoRa();
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
float MavLink_receive_attitude() //使わないけど...
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
}

void MavLink_receive_GPS_and_send_with_LoRa() //使わないけど...
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
}

void request_datastream() //使わないけど...
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
