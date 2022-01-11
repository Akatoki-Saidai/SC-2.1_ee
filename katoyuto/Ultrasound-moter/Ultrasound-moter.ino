long a;
int phase =1;
int previous_distance;
#include "SR04.h"
#define TRIG_PIN 0
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

int forward_phase = 1;
unsigned long previousMillis = 0;

int i=255;

unsigned long Offtime = 5000;
unsigned long Ontime = 2000;


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
  unsigned long currentMillis = millis();
    a=sr04.Distance();
    Serial.println(a);
    switch(phase){
      case 1:
        if (a>=30){
          Serial.println("okkkk");
          stopping();
          phase = 2;
        }
        else {
          Serial.println("katoyuuto");
          rotating();
        }
        break;

      case 2:
        previous_distance = a;
        if (sr04.Distance()-previous_distance >0){
          
          phase = 1;
          Serial.println("transit Phase 1");
        }
        else {
          forward();
          Serial.println("going forward");
        }
        break; 
    }
}
//前進
void forward(){
  ledcWrite(0, 255); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 255);
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

//右旋回
void rightturn(){
  ledcWrite(0, 150);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

//左旋回
void leftturn(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 150);
  ledcWrite(3, 0);
}


//ゆっくり停止
void stopping(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >=100 && i>0){
    ledcWrite(0,i);
    ledcWrite(1,0);
    ledcWrite(2,i);
    ledcWrite(3,0);
    i=i-5;
    Serial.println(i);
    previousMillis = currentMillis;
  }
  else if(i<=0){
    Serial.println("hello");
    stoppage();
}
}

//回転
void rotating(){
  ledcWrite(0, 100);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 100);
}
//反回転
void reverse_rotating(){
  ledcWrite(0, 0);
  ledcWrite(1, 100);
  ledcWrite(2, 100);
  ledcWrite(3, 0);  
}
