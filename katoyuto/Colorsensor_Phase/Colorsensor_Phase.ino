int Colorsensor_Phase=1;
int RED;
int BLUE;
int GREEN;
int R;
int G;
int B;
int Length ;
int Distance_to_Goal=0;//ゴールまでの距離何メートル以内であってほしいか
int Color_count = 1;

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
  Serial.begin(115200);

}

void loop() {
  switch(Colorsensor_Phase){
    case 1:
      if (R >= RED, G>= GREEN, B >= BLUE){//ここで色の判定
        if (Color_count == 1){
          Serial.println("Parachase!!");
          back();        //後ろに下がる
          delay(2000);   //ここの時間は考える
          rotating();    //90度回転
          delay(1000);   //ここの時間は考える
          forward();     //前に進む
          delay(2000);   //ここの時間は考える
          Color_count = 2;
          Colorsensor_Phase = 2; 
          
        }
        else if(Color_count == 2){
          back();        //後ろに下がる
          delay(2000);   //ここの時間は考える
          rotating();    //180度回転
          delay(2000);   //ここの時間は考える
          forward();     //前に進む
          delay(2000);   //ここの時間は考える
          
        }
      }
      else{
        Colorsensor_Phase = 3;
      }
      break;
      

    case 2:
      if (Length <= 50){//ここにはCansatがゴール付近にいるのかどうかを判定
        Colorsensor_Phase = 10; //Cansatがゴール付近にいるならば終了
      }
      else{
        Colorsensor_Phase = 1;  //Cansatがゴール付近にいないならばもう一度色の検知から
      }
      break;
      
      
    case 3:
      if (Length <= 50){//ここにはCansatがゴール付近にいるのかどうかを判定
        Colorsensor_Phase = 10;
      }
      else {
        Colorsensor_Phase = 1;
      }
      
    
    default:
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

//回転(時計回り)
void rotating(){
  ledcWrite(0, 100);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 100);
}
//反回転(反時計回り)
void reverse_rotating(){
  ledcWrite(0, 0);
  ledcWrite(1, 100);
  ledcWrite(2, 100);
  ledcWrite(3, 0);
}
