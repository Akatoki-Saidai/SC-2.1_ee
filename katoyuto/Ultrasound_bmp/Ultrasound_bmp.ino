long a;
#include "SR04.h"
#define TRIG_PIN 0
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float Temperature = 13.5; // BMPで取得した温度を入力
  float sound_velo = 331.5 + 0.6 * Temperature;
  a = sr04.Distance()/340 * sound_velo;
  Serial.println(a);
}
