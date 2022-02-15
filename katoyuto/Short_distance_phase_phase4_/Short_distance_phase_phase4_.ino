long a = 0;
int phase = 5;
int count = 0;
int previous_distance = 0;
int current_distance;
int EPSILON = 5;
int distance1,distance2;
#include <math.h>
#include "SR04.h"
#define TRIG_PIN 0
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

int forward_phase = 1;
unsigned long previousMillis = 0;

int Priviousdistance;

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
  a=sr04.Distance();
  current_distance = a;
  Serial.println(a);
  switch(phase){
    case 5:
      rotating();
      if(a < 500){
        phase = 0;
        stopping();
      }
      break;
      
    case 0:
        if(abs(current_distance - previous_distance) < EPSILON){
          count++;
        }
        if(count==5){//イズチェック
          phase = 1;
          distance1 = current_distance;
          count = 0;
        }
        previous_distance = current_distance;
       
      break;

    case 1:
      forward();
      delay(5000);
      stopping();
      phase = 2;    
      break;

    case 2:
      if(abs(current_distance - previous_distance) < EPSILON){
        count++;
      }
      if(count==5){//イズチェック
        phase = 3;
        distance2 = current_distance;
        count = 0;
      }
      previous_distance = current_distance;
      break;

    case 3:
      if(distance2-distance1<0){
        phase = 1;
      }else if(distance2-distance1>0){
        phase = 5;
      }else if(distance2<5){
        phase = 4;
      }
      break;

    case 4:
      Serial.println("GOAL!");
      phase = 6;
      break;
      
    default:
      break;
  }
}

//前進
void forward(){
  ledcWrite(0, 127); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 127);
  ledcWrite(3, 0);
}

//ターボ
void forward(){
  ledcWrite(0, 255); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 255);
  ledcWrite(3, 0);
}

//後転
void back(){
  ledcWrite(0, 0);
    ledcWrite(1, 63);
    ledcWrite(2, 0);
    ledcWrite(3, 63);

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
  ledcWrite(0, 63);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

//左旋回
void leftturn(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 63);
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
  ledcWrite(0, 63);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 63);
}
//反回転
void reverse_rotating(){
  ledcWrite(0, 0);
  ledcWrite(1, 63);
  ledcWrite(2, 63);
  ledcWrite(3, 0);  
}
