#include <TinyGPS++.h>
#include <string.h>
#include <SoftwareSerial.h>
#include <Wire.h>     //トワイライトで使用
#include <math.h>     //距離の導出で使用
#include <SD.h>      //SDカード記録用


#define trig 2 //超音波センサー出力ピンを5番に設定
#define echo 4 //超音波センサー入力ピンを7番に設定
#define sepa 7 //パラ分離機構ピンを10番に設定
#define tem A2 //温度センサーの入力ピンをA2番に設定
#define lIN1 3 //左モータードライバのIN1につなぐピンを3番に設定
#define lIN2 5 //左モータードライバのIN2につなぐピンを5番に設定
#define rIN1 6 //右モータードライバのIN1につなぐピンを6番に設定
#define rIN2 9 //右モータードライバのIN2につなぐピンを9番に設定

TinyGPSPlus gps;
SoftwareSerial mySerial(A6,A7);

//ゆっくり止まるための変数
int D = 0;
int B = 80;

//GPS用(緯度経度)の変数
float GPSlat;
float GPSlng;


//ゴールの緯度、経度
float Goallat =1 ;
float Goallng =1 ;


///GPSのデータを利用して、計算するための変数
double latdegree = 40009/360;
double lngdegree = 40075/360;
double Dlat;
double Dlng;
double dlat;
double dlng;


//ゴールとの距離を示す変数
float Distance;

//GPSから超音波でのゴール探索への移行に使う変数
double now_dis;
double now_rad;
double next_dis;
double next_rad;
double delta_rad;

//ゴール判定のための変数
int F = 0;


void setup() {
  
 //超音波センサーの設定
 pinMode(trig, OUTPUT);  
 pinMode(echo, INPUT);
 pinMode(sepa,OUTPUT);
 
  //左モーターの設定 (PWMピンは)
  pinMode(lIN1,OUTPUT);
  pinMode(lIN2,OUTPUT);

  //右モーターの設定 (PWMピンは)
  pinMode(rIN1,OUTPUT);
  pinMode(rIN2,OUTPUT);

  //GPSで使用？
  mySerial.begin(9600);

  //SDカード
  SD.begin(10);
 
}

void loop() {
  //着地関数で着地
  landing();
  //パラ分離関数で分離
  separate();

  //後転して、スタビライザーを倒す
  int t = 10000;
  forward(t);
  stopping();

  //GPSからくるデータとゴールの座標の差が閾値を超えるまで、繰り返す
  while (now_dis > 3) {
    //GPSからくるデータを読み取る
    now_dis = CalculateDis();
    now_rad = CalculateAngel();
    forward(1000);
    next_dis = CalculateDis();
    next_rad = CalculateAngel();
    delta_rad = next_rad - now_rad;
    if (delta_rad > 0 ) {
      levoversion(delta_rad*M_2_PI*1000);
    } else if (delta_rad < 0) {
      dextroversion(abs(delta_rad)*M_2_PI*1000);
    }
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    if (dataFile) {
    char SDchar1[32];
    char SDchar2[32];
    dataFile.seek(dataFile.size());
    snprintf(SDchar1, 32, "%lf", next_dis);
    snprintf(SDchar2, 32, "%lf", next_rad);
    dataFile.println(SDchar1);
    dataFile.println(SDchar2);
    dataFile.close();
  }
    forward(1000);
    
  }

  //超音波センサーを用いてゴールを探す
  while (F == 0) {
   double Searchdis = measuring();

   if (Searchdis > 10 ) {                //センサーからのデータが10mを超えているときは方向転換
      int s = 1000;
      dextroversion(s);
      stopping();
   
   } else if (Searchdis <= 10 && Searchdis != 0 ) {        //10m以内なら、前進
      int u = 10000;
      forward(u);
      stopping();
                                                             
   } else if (Searchdis = 0) {                               //停止用
      F = ++F;
      stopping();
   } 
  }
  while(1);
  
}


//着地するための関数
void landing() {
  int starttime = millis();                //Arduinoが起動した時間
  int counter = 0;                         //着地判定用の変数を初期化

  //カウンタの閾値を超えるまでループ
  do {                    
       int nowtime = millis();            //現在の時刻
       int etime = nowtime - starttime;   //電源が入ってからの経過時間

       //タイマー(緊急用)
       if (etime >= 180000) {             //Arduinoの起動時間が閾値を超えたら、do{}を抜ける
         break;
        } 
        
       double dis1,dis2,Deltadis; //地面からの距離を表す変数
       int temp1;                  //温度を示す変数
       dis1,dis2 = 0;              //初期化
       dis2 = dis1;                //dis2は一回前の測定結果を示す変数
       dis1 = measuring();         //measuring関数から、地面との距離を測定
       Deltadis = dis2-dis1;       //一回前の計測値との差
       
       //地面との距離の差が正かつ閾値より小さいならカウンタを１増やす
       if ((Deltadis > 0 ) && (Deltadis <= 1)) {     
           counter=++counter;
        }
        
      } while (counter <= 50);  
  
}


//パラシュート分離のための関数
void separate() {
  //パラシュート分離ピンに電流を流す
  for (int sec = 1; sec <= 10000; sec++); {
    digitalWrite(sepa, HIGH);      
    delay(50);
  }
  //電流を止める
  digitalWrite(sepa, LOW);  
  forward(1000);
}


//地面(ゴール)との距離を返す関数
double measuring() {
  
    //温度、距離計測用の変数
    unsigned long dur;
    double temp,dis;
    int ans;    
    float tv;   
    
    //気温を測定
    ans = analogRead(tem);      
    tv = map(ans,0,1023,0,5000);  //センサ値を電圧に変換
    temp = tv/100;                //電圧を気温に変換

    //距離を測定
    digitalWrite(trig, LOW); //初期化
    delayMicroseconds(2);
    digitalWrite(trig, HIGH); //超音波出力
    delayMicroseconds(10);
    digitalWrite(trig, LOW);  
    dur = pulseIn(echo, HIGH);  //超音波の反射を入力

    //距離を導出
    if (dur > 0) {       
          dis = dur/2;
          float sspeed = 331.5+0.6 * temp; //音速を導出
          dis = dis * sspeed * 100/1000000; //地面までの距離　（cm）
        }
    
    return dis;
}

//緯度経度から距離を返す関数
float CalculateDis() {
  
  //GPSから送られてくる文字列がメモリの中に保存される間ループする
  while (mySerial.available() > 0 ) {
    char c = mySerial.read(); 
    gps.encode(c);
    if (gps.location.isUpdated()) {
      GPSlat = gps.location.lat();
      GPSlng = gps.location.lng();
    }
    
    //二つの経緯度から距離を導出
    Dlat = GPSlat - Goallat;
    Dlng = GPSlng - Goallng;
    dlat = pow(Dlat, 2.0);
    dlng = pow(Dlng, 2.0);
    float distance2 = latdegree*dlat+lngdegree*dlng;
    float Distance = sqrtf(distance2);

    if (Distance > 0) {
      break;
    }
     
  }
  return Distance;
}


//角度計算用の関数
float CalculateAngel() {
  while (mySerial.available() > 0 ) {
    char c = mySerial.read(); 
    gps.encode(c);
    if (gps.location.isUpdated()) {
      GPSlat = gps.location.lat();
      GPSlng = gps.location.lng();
      break;
    }
  }
    //目標地点までの角度を導出
    Dlat = GPSlat - Goallat;
    Dlng = GPSlng - Goallng;
    float division = Dlng/Dlat;
    float rad = atanf(division);
    
    return rad;
}

/*/進行方向を計算する関数
void CalculateDir() {
  while (mySerial.available() > 0 ) {
    char c = mySerial.read(); 
    gps.encode(c);
    if (gps.location.isUpdated()) {
      GPSlat = gps.location.lat();
      GPSlng = gps.location.lng();
    }

    //目標地点までの角度を導出
    Dlat = GPSlat - Goallat;
    Dlng = GPSlng - Goallng;
    
    if (Dlat> 0 && Dlng > 0) {
      break;
    }
  
} */



//前進用
void forward(int a) {           
  //左モーターは反時計回り、右モーターは時計回り
  digitalWrite(lIN1,LOW);
  digitalWrite(lIN2,HIGH);
  digitalWrite(rIN1,HIGH);
  digitalWrite(rIN2,LOW);
  
  delay(a);
  
}


//後進用
void backing(int b) {
  //左モーターは時計回り、右モーターは反時計回り
  digitalWrite(lIN1,HIGH);
  digitalWrite(lIN2,LOW);
  digitalWrite(rIN1,LOW);
  digitalWrite(rIN2,HIGH);
  
  delay(b);
  
}


//右旋回用
void dextroversion(int c) {
  //左モーターは時計回り、右モーターは時計回り
  digitalWrite(lIN1,HIGH);
  digitalWrite(lIN2,LOW);
  digitalWrite(rIN1,HIGH);
  digitalWrite(rIN2,LOW);
  
  delay(c);
  
}


//左旋回用
void levoversion(int d) {
  //左モーターは反時計回り、右モーターは反時計回り
  digitalWrite(lIN1,LOW);
  digitalWrite(lIN2,HIGH);
  digitalWrite(rIN1,LOW);
  digitalWrite(rIN2,HIGH);
  
  delay(d);
  
  
}


//停止用
void stopping() {
  /*while(D < 1){ 
      for ( B = 80; B > 0; B -= 5 ) {
      digitalWrite(4,HIGH);
      digitalWrite(7,LOW);
      analogWrite(5,B);
      digitalWrite(8,HIGH);
      digitalWrite(10,LOW);
      analogWrite(6,B);
      delay( 100 );
      }
      D++;
    }*/
    //進むのをやめる
    digitalWrite(4,LOW);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    digitalWrite(10,LOW);
}
