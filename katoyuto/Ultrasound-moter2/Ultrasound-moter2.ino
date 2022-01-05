int phase =0;
int i=1;
long key;
#include "SR04.h"
#define TRIG_PIN 0
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);
}

void loop() {
    key=sr04.Distance();
    Serial.println(key);
    delay(200);
    if(phase == 0){
      if (i >10){
        i = i+1;
      }
      else if (i=10){
        phase = 1;
        forward();
      }
    }
    if (phase == 1){
      Serial.println("jjjjjjjjjjj");
      if (key<5){
        Serial.println("stopping");
        phase = 2;
      }
      else{
        
        Serial.println("kkkkkk");
        forward();
      }
    }
    else if (phase == 2) {
      stoppage();
      Serial.println("stopped");
    }
}
//前進
void forward(){
  ledcWrite(0, 150); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 150);
  ledcWrite(3, 0);
}

//後転
void back(){
  ledcWrite(0, 0);
    ledcWrite(1, 150);
    ledcWrite(2, 0);
    ledcWrite(3, 150);

}

//停止
void stoppage(){
  ledcWrite(0,0);
  ledcWrite(1,0);
  ledcWrite(2,0);
  ledcWrite(3,0);
}
