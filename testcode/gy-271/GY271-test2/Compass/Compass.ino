// 方位センサー(HMC5883L)のテスト
#include <Wire.h>
#include "skHMC5883L.h"


#define SENSOR_ADRS     0x1E            // デバイスのI2Cアドレス

char DirectionName[16][4] =  {" N ","NNE"," NE","ENE"," E ","ESE"," SE","SSE"," S ","SSW"," SW","WSW"," W ","WNW"," NW","NNW"} ;

skHMC5883L  Compass(SENSOR_ADRS) ;      // 方位センサーライブラリの生成を行う

void setup()
{
     int ans ;

     // シリアルモニターの設定
     Serial.begin(9600) ;
     // Ｉ２Ｃの初期化
     Wire.begin() ;                     // マスターとする
     // 方位センサーの初期化を行う(動作モードはアイドル状態)
     ans = Compass.Begin() ;
     if (ans == 0) Serial.println("Initialization normal") ;
     else {
          Serial.print("Initialization abnormal ans=") ;
          Serial.println(ans) ;
     }
}
void loop()
{
     int   ans  ;
     float deg ;

     // シングルモードでデータを読み出す
     ans = Compass.SingleRead(&deg,6.6) ;    // 磁気偏角6.6度
     if (ans == 0) {
          // 生のXYZ軸を表示する
          Serial.print("X[") ;
          Serial.print(RowX) ;
          Serial.print("] Y[") ;
          Serial.print(RowY) ;
          Serial.print("] Z[") ;
          Serial.print(RowZ) ;
          Serial.print("]  (") ;
          // １６分割の方位で表示する
          ans = Compass.GetOrientation(deg) ;
          Serial.print((char *)&DirectionName[ans]) ;
          Serial.print(")") ;
          // 角度で表示する
          Serial.println(deg) ;
     } else Serial.println("NG") ;

     delay(1000) ;  // １秒後に繰り返す
}