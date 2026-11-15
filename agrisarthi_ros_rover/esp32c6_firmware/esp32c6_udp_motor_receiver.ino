#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <math.h>
#include "esp32_protocol.h"
#include "probe_fsm.h"
const char* WIFI_SSID = "REPLACE_WITH_WIFI_SSID";
const char* WIFI_PASSWORD = "REPLACE_WITH_WIFI_PASSWORD";
const uint16_t UDP_PORT = 5005;
const uint32_t COMMAND_TIMEOUT_MS = 700;
const uint32_t WIFI_RETRY_MS = 3000;
const uint32_t PWM_FREQUENCY = 15000;
const uint8_t PWM_BITS = 8;
const int PWM_MAX = (1 << PWM_BITS) - 1;
// EXAMPLE PINS ONLY: verify these against your exact ESP32-C6 board and H-bridge.
const int LEFT_PWM_PIN=4, LEFT_DIR_PIN=5, RIGHT_PWM_PIN=6, RIGHT_DIR_PIN=7;
// Example angles only: mechanically calibrate the rack-and-pinion before use.
const int PROBE_SERVO_PIN=3, PROBE_INSERT_ANGLE=120, PROBE_RETRACT_ANGLE=30;
const int PROBE_MIN_ANGLE=10, PROBE_MAX_ANGLE=170;
const int PROBE_MIN_PULSE_US=500, PROBE_MAX_PULSE_US=2500;
const uint32_t PROBE_PWM_FREQUENCY=50;
const uint8_t PROBE_PWM_BITS=14;
const uint32_t PROBE_TRAVEL_MS=1800, PROBE_SETTLE_MS=300, PROBE_TIMEOUT_MS=5000;
const bool USE_PROBE_LIMIT_SWITCHES=false;
const int PROBE_INSERT_LIMIT_PIN=0, PROBE_RETRACT_LIMIT_PIN=1; // board-specific examples
const uint32_t PROBE_LIMIT_GRACE_MS=700;
WiFiUDP udp;
ProbeStateMachine probe;
uint32_t lastCommandMs=0, lastSeq=0, lastWifiAttemptMs=0;
bool haveSeq=false, haveSession=false, udpStarted=false, motorsEnabled=false;
bool servoReady=false;
float appliedLeft=0.0f, appliedRight=0.0f;
char activeSession[33]="";
IPAddress ownerIP;
uint16_t ownerPort=0;

void stopMotors(){
  ledcWrite(LEFT_PWM_PIN,0); ledcWrite(RIGHT_PWM_PIN,0);
  motorsEnabled=false; appliedLeft=0.0f; appliedRight=0.0f;
}
bool insertLimit(){return USE_PROBE_LIMIT_SWITCHES && digitalRead(PROBE_INSERT_LIMIT_PIN)==LOW;}
bool retractLimit(){return USE_PROBE_LIMIT_SWITCHES && digitalRead(PROBE_RETRACT_LIMIT_PIN)==LOW;}
void publishProbeStatus(const char* type,uint32_t seq=0,bool accepted=false,bool duplicate=false,const char* commandId=nullptr){
  if(!haveSession || WiFi.status()!=WL_CONNECTED) return;
  JsonDocument reply; reply["type"]=type; reply["protocol"]=1; reply["session_id"]=activeSession;
  reply["seq"]=seq; reply["accepted"]=accepted; reply["duplicate"]=duplicate;
  reply["command_id"]=commandId?commandId:probe.activeCommandId(); reply["probe_state"]=probeStateName(probe.state());
  reply["probe_error"]=probe.error(); reply["position_assumed"]=probe.positionAssumed();
  reply["motors_enabled"]=motorsEnabled; reply["left"]=appliedLeft; reply["right"]=appliedRight;
  udp.beginPacket(ownerIP,ownerPort); serializeJson(reply,udp); udp.endPacket();
}
void startProbeMove(bool insert){
  stopMotors();
  if(!servoReady){probe.setConfigurationError("SERVO_PWM_NOT_READY");return;}
  int angle=insert?PROBE_INSERT_ANGLE:PROBE_RETRACT_ANGLE;
  if(angle<PROBE_MIN_ANGLE || angle>PROBE_MAX_ANGLE){probe.setConfigurationError("SERVO_ANGLE_OUT_OF_LIMITS");return;}
  const uint32_t maxDuty=(1U<<PROBE_PWM_BITS)-1U;
  uint32_t pulse=PROBE_MIN_PULSE_US+(uint32_t)(PROBE_MAX_PULSE_US-PROBE_MIN_PULSE_US)*angle/180U;
  ledcWrite(PROBE_SERVO_PIN,(uint32_t)((uint64_t)pulse*PROBE_PWM_FREQUENCY*maxDuty/1000000U));
}
void setMotor(int pwmPin,int dirPin,float value){
  value=constrain(value,-1.0f,1.0f); digitalWrite(dirPin,value>=0?HIGH:LOW);
  ledcWrite(pwmPin,(int)(fabsf(value)*PWM_MAX));
}
void applyCommand(float left,float right,bool enable){
  if(!enable || !probe.motorAllowed()){stopMotors();return;}
  left=constrain(left,-1.0f,1.0f); right=constrain(right,-1.0f,1.0f);
  setMotor(LEFT_PWM_PIN,LEFT_DIR_PIN,left); setMotor(RIGHT_PWM_PIN,RIGHT_DIR_PIN,right);
  motorsEnabled=true; appliedLeft=left; appliedRight=right;
}
void sendReply(JsonDocument& doc,const char* type,bool accepted=false,uint32_t seq=0){
  JsonDocument reply;
  reply["type"]=type; reply["protocol"]=1; reply["session_id"]=activeSession;
  if(String(type)=="ack"){reply["seq"]=seq;reply["accepted"]=accepted;}
  reply["wifi_connected"]=(WiFi.status()==WL_CONNECTED);
  reply["motors_enabled"]=motorsEnabled; reply["left"]=appliedLeft; reply["right"]=appliedRight;
  reply["probe_state"]=probeStateName(probe.state()); reply["probe_error"]=probe.error();
  reply["position_assumed"]=probe.positionAssumed();
  udp.beginPacket(ownerIP,ownerPort); serializeJson(reply,udp); udp.endPacket();
}
bool packetFromOwner(){return haveSession && udp.remoteIP()==ownerIP && udp.remotePort()==ownerPort;}
void handlePacket(JsonDocument& doc){
  const char* type=doc["type"]|"";
  const char* session=doc["session_id"]|"";
  if(strlen(session)==0 || strlen(session)>32) return;
  if(String(type)=="hello"){
    // A new Pi process announces a fresh session. Switching sessions always stops first.
    bool changed=!haveSession || strcmp(activeSession,session)!=0 || udp.remoteIP()!=ownerIP || udp.remotePort()!=ownerPort;
    if(changed){stopMotors();strlcpy(activeSession,session,sizeof(activeSession));haveSeq=false;lastSeq=0;}
    ownerIP=udp.remoteIP();ownerPort=udp.remotePort();haveSession=true;lastCommandMs=millis();
    sendReply(doc,"hello_ack"); return;
  }
  if((String(type)!="motor_cmd" && String(type)!="probe_cmd") || !packetFromOwner() || strcmp(activeSession,session)!=0) return;
  uint32_t seq=doc["seq"]|0;
  bool accepted=!haveSeq || agrisarthiSequenceIsNewer(seq,lastSeq);
  if(!accepted){
    if(String(type)=="motor_cmd") sendReply(doc,"ack",false,seq);
    else {
      const char* commandId=doc["command_id"]|"";
      const char* action=doc["action"]|"";
      bool duplicateAccepted=false;
      bool duplicate=probe.cachedResult(commandId,action,duplicateAccepted);
      publishProbeStatus("probe_ack",seq,duplicate && duplicateAccepted,duplicate,commandId);
    }
    return;
  }
  haveSeq=true;lastSeq=seq;lastCommandMs=millis();
  if(String(type)=="probe_cmd"){
    const char* commandId=doc["command_id"]|"";
    const char* action=doc["action"]|"";
    ProbeCommandResult result=probe.command(commandId,action,millis(),PROBE_TRAVEL_MS,
      PROBE_SETTLE_MS,PROBE_TIMEOUT_MS,retractLimit());
    if(result.beginInsert) startProbeMove(true);
    if(result.beginRetract) startProbeMove(false);
    stopMotors();
    publishProbeStatus("probe_ack",seq,result.accepted,result.duplicate,commandId);
    return;
  }
  bool enable=doc["enable"]|false;
  float left=doc["left"]|0.0f, right=doc["right"]|0.0f;
  if(!isfinite(left) || !isfinite(right)){sendReply(doc,"ack",false,seq);return;}
  applyCommand(left,right,enable);
  Serial.printf("session=%s seq=%lu enable=%d L=%.2f R=%.2f probe=%s\n",activeSession,(unsigned long)seq,enable,appliedLeft,appliedRight,probeStateName(probe.state()));
  sendReply(doc,"ack",true,seq);
}
void startUdp(){if(!udpStarted){udp.begin(UDP_PORT);udpStarted=true;Serial.printf("UDP ready on %u\n",UDP_PORT);}}
void setup(){
  Serial.begin(115200); pinMode(LEFT_DIR_PIN,OUTPUT); pinMode(RIGHT_DIR_PIN,OUTPUT); pinMode(LEFT_PWM_PIN,OUTPUT); pinMode(RIGHT_PWM_PIN,OUTPUT);
  if(USE_PROBE_LIMIT_SWITCHES){pinMode(PROBE_INSERT_LIMIT_PIN,INPUT_PULLUP);pinMode(PROBE_RETRACT_LIMIT_PIN,INPUT_PULLUP);}
  if(!ledcAttach(LEFT_PWM_PIN,PWM_FREQUENCY,PWM_BITS)||!ledcAttach(RIGHT_PWM_PIN,PWM_FREQUENCY,PWM_BITS)){Serial.println("LEDC setup failed; stopped");while(true){stopMotors();delay(1000);}}
  if(PROBE_MIN_ANGLE<0 || PROBE_MAX_ANGLE>180 || PROBE_MIN_ANGLE>PROBE_MAX_ANGLE ||
     PROBE_INSERT_ANGLE<PROBE_MIN_ANGLE || PROBE_INSERT_ANGLE>PROBE_MAX_ANGLE ||
     PROBE_RETRACT_ANGLE<PROBE_MIN_ANGLE || PROBE_RETRACT_ANGLE>PROBE_MAX_ANGLE ||
     PROBE_MIN_PULSE_US<=0 || PROBE_MAX_PULSE_US<=PROBE_MIN_PULSE_US) probe.setConfigurationError("SERVO_CONFIGURATION_OUT_OF_LIMITS");
  else {
    if(!ledcAttach(PROBE_SERVO_PIN,PROBE_PWM_FREQUENCY,PROBE_PWM_BITS)) probe.setConfigurationError("SERVO_PWM_SETUP_FAILED");
    else {
      servoReady=true;
      const uint32_t maxDuty=(1U<<PROBE_PWM_BITS)-1U;
      uint32_t pulse=PROBE_MIN_PULSE_US+(uint32_t)(PROBE_MAX_PULSE_US-PROBE_MIN_PULSE_US)*PROBE_RETRACT_ANGLE/180U;
      ledcWrite(PROBE_SERVO_PIN,(uint32_t)((uint64_t)pulse*PROBE_PWM_FREQUENCY*maxDuty/1000000U));
    }
  }
  if(servoReady && USE_PROBE_LIMIT_SWITCHES && retractLimit()) probe.setBootRetracted();
  stopMotors(); WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID,WIFI_PASSWORD);lastWifiAttemptMs=millis();lastCommandMs=millis();
}
void loop(){
  uint32_t now=millis();
  if(WiFi.status()!=WL_CONNECTED){
    stopMotors();
    if(udpStarted){udp.stop();udpStarted=false;}
    if(now-lastWifiAttemptMs>=WIFI_RETRY_MS){WiFi.disconnect();WiFi.begin(WIFI_SSID,WIFI_PASSWORD);lastWifiAttemptMs=now;}
    delay(10);return;
  }
  startUdp();
  int n=udp.parsePacket();
  if(n>0){
    char buf[768];int len=udp.read(buf,sizeof(buf)-1);
    if(len>0){buf[len]='\0';JsonDocument doc;auto err=deserializeJson(doc,buf);
      if(!err) handlePacket(doc); else Serial.println("Ignored malformed packet");
    }
  }
  if(probe.tick(now,PROBE_LIMIT_GRACE_MS,USE_PROBE_LIMIT_SWITCHES,insertLimit(),retractLimit())){
    stopMotors();publishProbeStatus("probe_status");
  }
  if(!probe.motorAllowed()) stopMotors();
  if(motorsEnabled && now-lastCommandMs>COMMAND_TIMEOUT_MS){stopMotors();Serial.println("Command timeout; motors stopped");}
  delay(2);
}
