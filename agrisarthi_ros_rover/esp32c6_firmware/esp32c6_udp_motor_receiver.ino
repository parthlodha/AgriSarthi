#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
const char* WIFI_SSID = "REPLACE_WITH_WIFI_SSID";
const char* WIFI_PASSWORD = "REPLACE_WITH_WIFI_PASSWORD";
const uint16_t UDP_PORT = 5005;
const uint32_t COMMAND_TIMEOUT_MS = 700;
const uint32_t PWM_FREQUENCY = 15000;
const uint8_t PWM_BITS = 8;
const int PWM_MAX = (1 << PWM_BITS) - 1;
// EXAMPLE PINS ONLY: verify these against your exact ESP32-C6 board and H-bridge.
const int LEFT_PWM_PIN=4, LEFT_DIR_PIN=5, RIGHT_PWM_PIN=6, RIGHT_DIR_PIN=7;
WiFiUDP udp; uint32_t lastCommandMs=0, lastSeq=0; bool haveSeq=false;
void stopMotors(){ ledcWrite(LEFT_PWM_PIN,0); ledcWrite(RIGHT_PWM_PIN,0); }
void setMotor(int pwmPin,int dirPin,float value){ value=constrain(value,-1.0f,1.0f); digitalWrite(dirPin,value>=0?HIGH:LOW); ledcWrite(pwmPin,(int)(fabsf(value)*PWM_MAX)); }
void applyCommand(float left,float right,bool enable){ if(!enable){stopMotors();return;} setMotor(LEFT_PWM_PIN,LEFT_DIR_PIN,left); setMotor(RIGHT_PWM_PIN,RIGHT_DIR_PIN,right); }
void setup(){
  Serial.begin(115200); pinMode(LEFT_DIR_PIN,OUTPUT); pinMode(RIGHT_DIR_PIN,OUTPUT); pinMode(LEFT_PWM_PIN,OUTPUT); pinMode(RIGHT_PWM_PIN,OUTPUT);
  if(!ledcAttach(LEFT_PWM_PIN,PWM_FREQUENCY,PWM_BITS)||!ledcAttach(RIGHT_PWM_PIN,PWM_FREQUENCY,PWM_BITS)){Serial.println("LEDC setup failed; stopped");while(true){stopMotors();delay(1000);}}
  stopMotors(); WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASSWORD); Serial.print("Connecting Wi-Fi");
  while(WiFi.status()!=WL_CONNECTED){delay(300);Serial.print('.');} Serial.println(); Serial.print("ESP32 IP: ");Serial.println(WiFi.localIP()); udp.begin(UDP_PORT);lastCommandMs=millis();
}
void loop(){
  int n=udp.parsePacket(); if(n>0){char buf[768];int len=udp.read(buf,sizeof(buf)-1);if(len>0){buf[len]='\0';JsonDocument doc;auto err=deserializeJson(doc,buf);
    if(!err && String((const char*)(doc["type"]|""))=="motor_cmd"){
      uint32_t seq=doc["seq"]|0;bool enable=doc["enable"]|false;float left=doc["left"]|0.0f,right=doc["right"]|0.0f;
      if(!haveSeq||(int32_t)(seq-lastSeq)>0){haveSeq=true;lastSeq=seq;lastCommandMs=millis();applyCommand(left,right,enable);Serial.printf("seq=%lu enable=%d L=%.2f R=%.2f\n",(unsigned long)seq,enable,left,right);}
    }else Serial.println("Ignored malformed packet");
  }}
  if(millis()-lastCommandMs>COMMAND_TIMEOUT_MS)stopMotors(); delay(2);
}
