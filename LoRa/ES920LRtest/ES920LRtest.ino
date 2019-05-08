/*
 *スイッチに関する記述とかあったっけ、、、
 */
#include <SoftwareSerial.h>
#include <string.h>

static const int LED = 5;//LEDのピン、どこでもいい。今回はD2(5番ピン)

// static const int sw = 16;
// static const unsigned long PUSH_SHORT = 1000;
//上の二行いるかわからんのでコメントアウト

static const int LoRa_RX = 13, LoRa_TX = 12;//RX:D10 TX:D9
static const int LoRa_Rst = 14;//Loraのリセットピンに接続:D11
SoftwareSerial LoRa_ss(LoRa_RX, LoRa_TX);

void LoRa_reset(void) {//Loraのリセット
    digitalWrite(LoRa_Rst, LOW);
    delay(100);
    digitalWrite(LoRa_Rst, HIGH);
}

int LoRa_recv(char *buf) {//LoraからArduinoの読み出し
    char *start = buf;

    while (true) {//無限ループ
        delay(0);
        while (LoRa_ss.available() > 0) {//Loraからの信号が届いている限りループ
            *buf++ = LoRa_ss.read();
            if (*(buf-2) == '\r' && *(buf-1) == '\n') {
                *buf = '\0';
                return (buf - start);
            }
        }
    }
}

int LoRa_send(char * msg) {//ArduinoからLoRaへの送信
    char *start = msg;//startはmsgの先頭アドレス
    while (*msg != '\0') {
        LoRa_ss.write(*msg++);//1バイト書き込み、ポインタをインクリメントする
    }
    return (msg - start);//最終的なmsgのアドレスとスタートアドレスの差を取るから、送信メッセージのバイト数を返している？
}

// #define NMODE 24
//
// struct mode {
//     int bw;//帯域幅
//     int sf;//拡散率
//     int timeout;
// };
// struct mode Mode[NMODE] = {
//     {3, 12, 5}, {3, 11, 5}, {3, 10, 4}, {3, 9, 3}, {3, 8, 2}, {3, 7, 2},
//     {4, 12, 5}, {4, 11, 4}, {4, 10, 3}, {4, 9, 3}, {4, 8, 2}, {4, 7, 2},
//     {5, 12, 4}, {5, 11, 3}, {5, 10, 2}, {5, 9, 2}, {5, 8, 2}, {5, 7, 2},
//     {6, 12, 3}, {6, 11, 3}, {6, 10, 2}, {6, 9, 2}, {6, 8, 2}, {6, 7, 2},
// };

int sendcmd(char *cmd) {//Loraにコマンドを送信する関数
    unsigned long t;
    char buf[64];

//    Serial.print(cmd);
    LoRa_send(cmd);

    while (true) {
        LoRa_recv(buf);
        if (strstr(buf, "OK")) {
//            Serial.print(buf);
            return true;
        } else if (strstr(buf, "NG")) {
//            Serial.print(buf);
            return false;
        }
    }
}

void setMode(void) {//プロセッサモードかつオペレーションモードにする
    char buf[64];

    LoRa_send("config\r\n");
    delay(200);
    LoRa_reset();
    delay(1500);

    while (true) {
        LoRa_recv(buf);
        if (strstr(buf, "Mode")) {
            Serial.print(buf);
            break;
        }
    }

    sendcmd("2\r\n");//プロセッサーモードにする
    // sprintf(buf, "bw %d\r\n", bw);//帯域幅の設定
    // sendcmd(buf);
    // sprintf(buf, "sf %d\r\n", sf);//拡散率の設定
    // sendcmd(buf);
    sendcmd("q 2\r\n");//モード設定、ここだけ必要かも
    sendcmd("w\r\n");

    LoRa_reset();
    Serial.println("LoRa module set to new mode");
    delay(1000);
}

void send2LoRa() {
    char obuf;
    char * buf;

    obuf = "Hello";
    buf = &obuf;

    LoRa_send(buf);
    delay(500);
    // int n;
    // unsigned long t;

    // strcpy(obuf, "loc=(");//obufに"loc="をコピー
    // dtostrf(gps.location.lat(), 12, 8, &obuf[strlen(obuf)]);//緯度情報を文字列に変換
    // strcat(obuf, ",");//文字列を連結
    // dtostrf(gps.location.lng(), 12, 8, &obuf[strlen(obuf)]);//経度情報を文字列に変換
    // strcat(obuf, ")\r\n");//文字列を連結
    //
    // strcpy(shortbuf, "loc=(");
    // dtostrf(gps.location.lat(), 8, 4, &shortbuf[strlen(shortbuf)]);
    // strcat(shortbuf, ",");
    // dtostrf(gps.location.lng(), 8, 4, &shortbuf[strlen(shortbuf)]);
    // strcat(shortbuf, ")\r\n");

    // digitalWrite(LED, HIGH);//LED点灯

    // for (int i = 0; i < NMODE; i++) {//帯域幅と拡散率を切り替える
    //     // Serial.print("setMode(bw: ");
    //     // Serial.print(Mode[i].bw);
    //     // Serial.print(", sf: ");
    //     // Serial.print(Mode[i].sf);
    //     // Serial.println(")");
    //
    //     setMode(Mode[i].bw, Mode[i].sf);
    //
    //     t = millis();//プログラムが実行されてからの時間
    //     delay(500);
    //
    //     Serial.print("send to LoRa: ");
    //     buf = (i == 0) ? shortbuf : obuf;//i=0ならbuf=shotbuf、それ以降ならbuf=obuf
    //     Serial.print(buf);
    //     LoRa_send(buf);
    //
    //     while (true) {
    //         LoRa_recv(ibuf);
    //         if (strstr(ibuf, "OK")) {
    //             Serial.print(ibuf);
    //             break;
    //         } else if (strstr(ibuf, "NG")) {
    //             Serial.print(ibuf);
    //             break;
    //         }
    //     }
    //
    //     if (i != 0) {
    //         long s;
    //
    //         s = Mode[i].timeout * 1000 - (millis() - t);
    //         if (s > 0) {
    //             Serial.print("delay: ");
    //             Serial.println(s);
    //             delay(s);
    //         }
    //     }
    // }

    // digitalWrite(LED, LOW);
}

void setup()
{
    // WiFi.mode(WIFI_OFF);

    Serial.begin(115200);
    delay(20);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // gps_ss.begin(9600);

    // pinMode(sw, INPUT);

    LoRa_ss.begin(115200);
    pinMode(LoRa_Rst, OUTPUT);
    digitalWrite(LoRa_Rst, HIGH);

    LoRa_reset();

    delay(1000); // LoRaモジュールをリセットしてからCPUと通信できるまでに1秒程度かかるようだ

    while (LoRa_ss.available() > 0) {//何やってるかわからんけどとりあえず残しときたい
        char c = LoRa_ss.read();
            if (c < 0x80) {
                Serial.print(c);
            }
    }
    Serial.println(F("\r\nStart"));
    setMode();//setupの中でsetModeしないといけなさそうなので。
}

void loop()
{
    // unsigned long gauge = 0;
    char buf[128];
    digitalWrite(LED, HIGH);//LED点灯


    // while (digitalRead(sw) == 0) {
    //     gauge++;
    //     delay(0);
    // }
    // if (gauge > PUSH_SHORT) {
    //     if (gps.location.isValid()) {
    //         gps_ss.enableRx(false);
    //         send2LoRa();
    //         gps_ss.enableRx(true);
    //     }
    // }
    // while (gps_ss.available() > 0) {
    //     if (gps.encode(gps_ss.read())) {
    //         break;
    //     }
    // }
}
