#include <TinyGPS++.h>
#include <DFRobot_QMC5883.h>
#include <math.h>

DFRobot_QMC5883 compass;
TinyGPSPlus gps;

int phase = 4;
char key = '0';
const int SAMPLING_RATE = 200;
int phase_state = 0;

//for phase3
int type = 1;
int yeah = 1;
int type_state = 0;
int cutparac = 23;                  //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;            //切り離し時の9V電圧を流す時間，単位はsecond
float time3_1,time3_2,St_Time;      //時間に関するもの
float Accel[6];                     //計測した値をおいておく関数
float Altitude[6];                  //(高度)
float Preac,differ1,Acsum,Acave,RealDiffer1;
float Preal,differ2,Alsum,Alave,RealDiffer2;
int i_3=0;
int j_3=0;
float RealDiffer;

//for MPU6050
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 21 //I2C通信
#define SCL_MPU 22
#endif
MPU9250_asukiaaa mySensor;

//for BMP180
#include <Wire.h> //I2C通信
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#define SDA_BMP 21
#define SCL_BMP 22

//for SD Card
#include <SPI.h> //SDカードはSPI通信
#include <SD.h>
File CanSatLogData;
File SensorData;
const int sck=13 , miso=15 , mosi=14 , ss=27;

//センサー値の格納
double Temperature, Pressure, accelX, accelY, accelZ, magX, magY, magZ, gyroX, gyroY, gyroZ, accelSqrt = 0, gps_latitude, gps_longitude, gps_velovity, altitude = 0;
int gps_time, R, G, B;
long ultra_distance;


//for phase1,2
int mode_average = 0;
int mode_comparison = 0;
int mode_to_bmp = 0;
int count1 = 0;
int count2 = 0;
int count3 = 0;
double altitude_average = 10000;
double altitude_sum = 0;
double alt[5];
unsigned long current_millis;
unsigned long previous_millis;
double altitude_sum_mpu = 0;
double altitude_sum_bmp = 0;
double altitude_sum_gps = 0;
double previous_altitude;
double current_altitude;

//for phase4
//平均値
int count = 0;
//int   count1 = 0;
//int   count2 = 0;
float until_5;

int phase4 = 1;
float v_initial= 38.0;  //[m/s]
float g        = 9.80665;  //[m/s/s]
float rotate_x ;//回転速度(TBD)[deg/s]
float ditermined_dis = 3; //GPSでどれくらい近づくか(TBD)
float angle_radian;
float angle_degree;
  
//you need to set up variables at first
float GOAL_lat = 35.862857820;
float GOAL_lng = 139.607681275;

//variables___GPS
//緯度
float GPSlat_array[5];
float GPSlat_sum = 0;
float GPSlat_data;
//経度
float GPSlng_array[5];
float GPSlng_sum = 0;
float GPSlng_data;
float delta_lng;
//距離
float distance; //直進前後でゴールに近づいているかどうかを表す
float Pre_distance;
//variables___GY-271
float heading_data;
float heading_array[5];
float heading_sum = 0;
float heading;
float rotate_degree = 0;
float rotate_sec = 0;
//Vector norm = compass.readNormalize();
float headingDegrees;
float omega;
float azimuth;

int Colorsensor_Phase=1;
int RED;
int BLUE;
int GREEN;
int Length;
int Distance_to_Goal=0;//ゴールまでの距離何メートル以内であってほしいか
int Color_count = 1;


//for phase5
//those are variables which is used in 超音波
//int count = 0;
int phase_5;
int previous_distance = 0;
int current_distance;
int EPSILON = 5;
int distance1,distance2;
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

volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt timer function
void IRAM_ATTR onTimer1(){
    portENTER_CRITICAL_ISR(&timerMux);
    timeCounter1++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

//緯度経度から距離を返す関数
float CalculateDis(double GOAL_lng, double gps_longitude, double gps_latitudeitude) {
    
   delta_lng=GOAL_lng-gps_longitude;
   omega = acos(sin(gps_latitudeitude*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(gps_latitudeitude*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
   distance = 6378.137*pow(10,3)*omega;
     
  return distance;
}

//角度計算用の関数
float CalculateAngel(double GOAL_lng, double gps_longitude, double gps_latitudeitude) {

    //目標地点までの角度を導出
    delta_lng=GOAL_lng-gps_longitude;
    azimuth=90-(360/(2*M_PI))*atan2((cos((gps_latitudeitude*2*M_PI/360)*2*M_PI/360)*tan((GOAL_lat*2*M_PI/360)*2*M_PI/360)-sin((gps_latitudeitude*2*M_PI/360)*2*M_PI/360)*cos((delta_lng*2*M_PI/360)*2*M_PI/360)),(sin((delta_lng*2*M_PI/360)*2*M_PI/360)));
    if(azimuth<0)
    {
      azimuth += 360;
    }
    
    return azimuth;
}

//前進
void forward(){
  ledcWrite(0, 127); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 127);
  ledcWrite(3, 0);
}

//ターボ
void turbo(){
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


//ゆっくり停止
void stopping(){
  /*unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >=50 && i>0){
    ledcWrite(0,i);
    ledcWrite(1,0);
    ledcWrite(2,i);
    ledcWrite(3,0);
    i=i-5;
    Serial.println(i);
    previousMillis = currentMillis;
  }
  else if(i<=0){
    //Serial.println("hello");
    stoppage();
  }*/
    for(int i=255;i>0;i=i-5){
      ledcWrite(0,i);
      ledcWrite(1,0);
      ledcWrite(2,i);
      ledcWrite(3,0);
      delay(50);//stoppingではdelay使う
    }
}

void setup() {
    Serial1.begin(115200, SERIAL_8N1, 5, 18);
    //SD Card initialization
    SPI.begin(sck,miso,mosi,ss);
    SD.begin(ss,SPI);
    CanSatLogData = SD.open("/CanSatLogData.log", FILE_APPEND);
    SensorData = SD.open("/SensorData.log",FILE_APPEND);
    

    //割り込み関数
    timer1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer1, &onTimer1, true);
    timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
    timerAlarmEnable(timer1);
    //無線通信
    Serial.begin(115200);
    //Serial.begin(115200, SERIAL_8N1, 16, 17); //関数内の引数はデータ通信レート，unknown，RXピン，TXピン
    Serial.write("TESTING: Serial communication\n");
    Serial.write("TESTING: Serial communication\n");
    CanSatLogData.println("START_RECORD");  
    CanSatLogData.flush();
    SensorData.println("gps_time,gps_latitudeitude,gps_longitude,gps_velovity,Temperature,Pressure,accelX,accelY,accelZ,heading,headingDegrees,R,G,B,ultra_distance");
    SensorData.flush();
    
    //for MPU6050
    Wire.begin(SDA_BMP, SCL_BMP);
    mySensor.beginAccel();
    //for BMP
    bmp.begin();
    
    //for motor
      ledcSetup(0, 490, 8);
      ledcSetup(1, 490, 8);
      ledcSetup(2, 490, 8);
      ledcSetup(3, 490, 8);
    
      ledcAttachPin(32, 0);
      ledcAttachPin(33, 1);
      ledcAttachPin(26, 2);
      ledcAttachPin(25, 3);

    //for GY-271

      if (!compass.begin())  //元はwhile
      {
        Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
        delay(500);
      }

      if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
      }
      else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
      }

    //for colorsensor
      Wire.begin();
      Wire.beginTransmission(0x39);
      Wire.write(0x44);
      Wire.write(0x01);
      Wire.endTransmission();
      Wire.beginTransmission(0x39);
      Wire.write(0x42);
      Wire.write(0x12);
      Wire.endTransmission();

}//setup関数閉じ


void loop() {
 if (Serial1.available() > 0) {//これないとGPS使えない
   char c = Serial1.read();
   gps.encode(c);
   if (gps.location.isUpdated()) {//この中でgpsのデータを使わないと，うまく値が表示されない
    if (timeCounter1 > 0) {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
     

        //起動時刻の更新
        unsigned long currentMillis = millis();

        //センサー値のアップデート
        mySensor.accelUpdate();
        //センサー値取得
        //GY271
        Vector norm = compass.readNormalize();

        //BMP180
        Temperature = bmp.readTemperature();
        Pressure = bmp.readPressure();
        if (mySensor.accelUpdate() == 0) {
            accelX = mySensor.accelX();
            accelY = mySensor.accelY();
            accelZ = mySensor.accelZ();
            accelSqrt = mySensor.accelSqrt();
        }
        altitude = bmp.readAltitude();
        //GPS
        char c = Serial1.read();    //GPSチップからのデータを受信
        gps.encode(c);              //GPSチップからのデータの整形
        gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();
        gps_time = gps.time.value();
        gps_velovity = gps.speed.mps();
        ultra_distance = sr04.Distance();
        int i_c;
        Wire.beginTransmission(0x39);
        Wire.write(0x50);
        Wire.endTransmission();
        Wire.requestFrom(0x39, 8);
        for (i_c=1 ; i_c<5; i_c++){
          int c1 = Wire.read();
          int c2 = Wire.read();
          c1 = c1  + c2 * 256;
          switch (i_c) {
            case 1: Serial.print("R =");
                    R = c1;
                    break;
            case 2: Serial.print(" , G =");
                    G = c1;
                    break;
            case 3: Serial.print(" , B =");
                    B = c1;
                    break;
            case 4: Serial.print(" , C =");break; }
          Serial.print(c1);
          if (i_c==4) Serial.println();
        }

        Serial.print(gps_time);
        Serial.print(",");        
        Serial.print(gps_latitude);
        Serial.print(",");
        Serial.print(gps_longitude);
        Serial.print(",");
        Serial.print(gps_velovity);
        Serial.print(",");
        Serial.print(Temperature);
        Serial.print(",");
        Serial.print(Pressure);
        Serial.print(",");
        Serial.print(accelX);
        Serial.print(",");
        Serial.print(accelY);
        Serial.print(",");
        Serial.print(accelZ);
        Serial.print(",");
        Serial.print(heading);
        Serial.print(",");
        Serial.print(headingDegrees);
        Serial.print(",");
        Serial.print(R);
        Serial.print(",");
        Serial.print(G);
        Serial.print(",");
        Serial.print(B);
        Serial.print(",");
        Serial.println(ultra_distance);      
        Serial.flush();

        //各フェーズごとの記述
        switch (phase){
          
           //########## 待機フェーズ ##########
            case 1:
                if(phase_state != 1){
                    //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial.write("Phase1: transition completed\n");    // 地上局へのデータ送信
                    //LogDataの保存
                    CanSatLogData.println(gps_time);
                    CanSatLogData.println("Phase1: transition completed");    
                    CanSatLogData.flush();
                   
                    phase_state = 1;
                }         
                if(mode_to_bmp == 0){ 
                    if(accelZ < -2){//落下開始をまずMPUで判定
                        altitude_sum_mpu += altitude;
                        count1++;
                        if(count1==1){
                            count1=0;
                            CanSatLogData.println(gps_time);
                            CanSatLogData.println("FALL STARTED(by MPU)\n");    
                            CanSatLogData.flush();
                            mode_to_bmp = 1;
                        }
                    }
                }else{
                    switch(mode_comparison){//落下開始をBMPで判定
                        case 0:     
                            previous_millis = millis();
                            altitude_sum_bmp += altitude;
                            count3++;
                            if(count3==5){
                                previous_altitude = altitude_sum_bmp/5;
                                altitude_sum_bmp = 0;
                                count3 = 0;
                                mode_comparison = 1;
                            }
                            break;
                        case 1://500ms後     
                            current_millis = millis();
                            if(current_millis - previous_millis >= 500){
                                altitude_sum_bmp += altitude;
                                count3++;
                                if(count3==5){
                                    current_altitude = altitude_sum_bmp/5;
                                    CanSatLogData.println(currentMillis);
                                    CanSatLogData.println("current_altitude - previous_altitude = \n");  
                                    CanSatLogData.println(current_altitude - previous_altitude);  
                                    if(current_altitude - previous_altitude <= -1.0){
                                        CanSatLogData.println("FALL STARTED(by BMP)\n");                                                      
                                        CanSatLogData.flush();
                                        phase = 2;
                                    }else{
                                        altitude_sum_bmp = 0;
                                        count3 = 0;
                                        mode_comparison = 0;
                                    } 
                                } 
                            }
                            break;
                    }
                }
                break;
            //########## 降下フェーズ ##########
            case 2:
                if(phase_state != 2){
                    //降下フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial.write("Phase2: transition completed\n"); //地上局へのデータ送信
                    
                    //LogDataの保存
                    CanSatLogData.println(gps_time);
                    CanSatLogData.println("Phase2: transition completed");    
                    CanSatLogData.flush();

                    i_3 = 0;
                    j_3 = 0;
                    Preac = 0;         //1個前の加速度を記憶
                    Preal = 0;
                    differ1 = 0.1;     //加速度　移動平均の差
                    differ2 = 0.5;     //高度　　移動平均の差
                    Acave = 0;         //加速度　5個の平均値
                    Alave = 0;         //高度　　5個の平均値
                    Acsum = 0;         //加速度　5個の合計値
                    Alsum = 0;         //高度　　5個の合計値
                    RealDiffer1 = 0;   //1秒前との差を記憶する
                    RealDiffer2 = 0;
                            
                    phase_state = 2;
                }


                if(yeah == 1){     //データを初めから五個得るまで
                    Accel[i_3] = accelSqrt;
                    Altitude[i_3] = altitude;
                    i_3 = i_3 + 1;
                    if(i_3 == 6){        //5個得たらその時点での平均値を出し，次のフェーズへ
                        yeah = 2;
                        i_3 = 0; //iの値をリセット
                        for(j_3=1 ; j_3<6 ; j_3++){   //j_3=0の値は非常に誤差が大きいので1から
                            Acsum = Acsum + Accel[j_3];  
                            Alsum = Alsum + Altitude[j_3];
                        }
                        Acave = Acsum/5;
                        Alave = Alave/5;
                        time3_2 = currentMillis;
                    }
                }else{
                    Preac = Acave;
                    Preal = Alave;  
                    Accel[i_3] = accelSqrt;
                    Altitude[i_3] = altitude;
                    for(j_3=0 ; j_3<5 ; j_3++){
                        Acsum = Acsum + Accel[j_3];
                        Alsum = Alsum + Altitude[j_3];
                    }
                    Acave = Acsum/5;
                    Alave = Alsum/5;
                    RealDiffer1 = Preac - Acave;
                    RealDiffer2 = Preal - Alave;
                    if(i_3 == 5){
                        i_3 = 0;
                        Acsum = 0; 
                        Alsum = 0;
                    }else{
                        i_3 = i_3+1;
                        Acsum = 0; 
                        Alsum = 0;
                    }
                    if(currentMillis - time3_2 > 1000){
                        if( RealDiffer1 < differ1 ){ //移動平均が基準以内の変化量だった時
                            phase = 3;
                        }else if( RealDiffer2 < differ2 ){
                            phase = 3;
                        }else{
                            time3_2 = currentMillis;
                        }
                    }
                 }

                

            break;
            //########## 分離フェーズ ##########
            case 3:
                if(phase_state != 3){
                    //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial.write("Phase3: transition completed\n");
                    //LogDataの保存
                    CanSatLogData.println(gps_time);
                    CanSatLogData.println("Phase3: transition completed");    
                    CanSatLogData.flush();
                    phase_state = 3;
                    time3_1 = currentMillis;                           //phase3　開始時間の保存
                    St_Time = time3_1 + outputcutsecond * 1000;        //基準時間
                    Serial.write("WARNING: The cut-para code has been entered.\n");
                    digitalWrite(cutparac, HIGH); //オン
                    Serial.write("WARNING: 9v voltage is output.\n");
                    //LogDataの保存
                    CanSatLogData.println(currentMillis);
                    CanSatLogData.println("9v voltage is output");
                    CanSatLogData.flush();
                    
                }

                if(currentMillis > St_Time){     //電流を流した時間が基準時間を超えたら
                    digitalWrite(cutparac, LOW); //オフ
                    Serial.write("WARNING: 9v voltage is stop.\n");
                    CanSatLogData.println(currentMillis);
                    CanSatLogData.println("WARNING: 9v voltage is stop.\n");
                    CanSatLogData.flush();
                    phase = 4;
                }
                       
                break;

            //########## 遠距離探索フェーズ ##########
            case 4:
               {if(phase_state != 4){
                    //遠距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial.write("Phase4: transition completed\n");
                    Serial.print("LAT(PHASE4_START):");Serial.println(gps_latitude,9);
                    Serial.print("LONG(PHASE4_START):");Serial.println(gps_longitude,9);
                    
                    //LogDataの保存
                    CanSatLogData.println(gps_time);
                    CanSatLogData.println("Phase4: transition completed");     
                    CanSatLogData.print("LAT(PHASE4_START):");CanSatLogData.println(gps_latitude,9);
                    CanSatLogData.print("LONG(PHASE4_START):");CanSatLogData.println(gps_longitude,9);                       
                    CanSatLogData.flush();

                    phase_state = 4;
                    count1 = 0;
                    count2 = 0;
                }

                /*switch(Colorsensor_Phase){
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
                }*/

                //GY271のデータの取得
                Vector norm = compass.readNormalize();
                switch(phase4){  
                    case 1:
                    Serial.println("phase4 = 1");                    
                    //データとるために一時停止
                    //stopping();
                    Serial.println("stopping!");

                    //変数count カウントが0のときはデータを取るモード，1になったら　移動するモード
                    //GPSとGY271で5回の平均を最初に出す
                    if(count == 0){
                        //CanSatの北からの角度を計算
                        float heading = atan2(norm.YAxis, norm.XAxis);
        
                        //移動平均のために角度と緯度経度のデータをためる
                        heading_array[count1] = heading;
                        GPSlat_array[count1] = gps_latitude;
                        GPSlng_array[count1] = gps_longitude;
                        if(count1 == 4){

                            //集めたデータの和をだす
                            for(count2=0;count2<5;count2++){
                                heading_sum = heading_sum + heading_array[count2];
                                GPSlat_sum  = GPSlat_sum  + GPSlat_array[count2];
                                GPSlng_sum  = GPSlng_sum  + GPSlng_array[count2];
                            }
                            //平均値
                            heading_data = heading_sum / 5;
                            GPSlat_data  = GPSlat_sum  / 5;
                            GPSlng_data   = GPSlat_sum  / 5;
                            //初めのデータをあつめ終わったのでカウントを1にする
                            count = 1;
                            //次にphase4=1にもどったときのためにcount1=0を代入する
                            count1 = 0;
                        }else{
                          //データが5個集まってないときはcount1の変数の数をふやす
                          count1++;
                        }
                    }         

                    //5回の平均を既に取得済みなら．(5回まだ取ってない時にこっちのプログラミに行かなくて済む)
                    else{
                    //緯度・経度からゴールとの距離を計測
                    delta_lng = GOAL_lng - GPSlng_data;
                    distance = 6378.137*pow(10,3)*acos(sin(GPSlat_data*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPSlat_data*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
                    angle_radian = asin((distance*g)/pow(v_initial,2.0))/2.0;
                    angle_degree = angle_radian*360.0/(2.0*M_PI);
                        
                    // Correct for heading < 0deg and heading > 360deg
                    if (heading < 0){
                        heading += 2 * PI;
                    }else if (heading > 2 * PI){
                        heading -= 2 * PI;
                    }
                        
                    //radからdegに変換
                    float headingDegrees = heading * 180/M_PI;
                    //回転する角度rotate_degreeで
                    if(heading > angle_degree){
                        if(heading > 180 + angle_degree){ //右回転
                            rotate_degree = angle_degree - heading + 360;
                            rotate_sec = rotate_degree / rotate_x;
                            //rightturn();
                            Serial.println("turn right");        
                            delay(rotate_sec);
                        }else{//左回転
                            rotate_degree = heading - angle_degree;
                            rotate_sec = rotate_degree / rotate_x; 
                            //leftturn();
                            Serial.println("turn left");
                            delay(rotate_sec);
                        }
                    }else if(angle_degree >= heading){
                        if(angle_degree > 180 + heading){//左回転
                            rotate_degree = heading - angle_degree + 360;
                            rotate_sec = rotate_degree / rotate_x;
                            //leftturn();
                            Serial.println("turn left");
                            delay(rotate_sec);
                    }else{//右回転
                            rotate_degree = angle_degree - heading;
                            rotate_sec = rotate_degree / rotate_x;
                            //rightturn();
                            Serial.println("turn right");        
                            delay(rotate_sec);
                    }
                }
                //stopping();
                Serial.println("stopping!");
    
                //forward();
                Serial.println("forward!");

                //カウンターを変更
                phase4 = 2;

                //階差を求めたいので，求めたデータは過去のデータに．
                Pre_distance = distance;
            }

            case 2:
                Serial.println("phase4 = 2");
                if(count1 < 4){
                    GPSlat_array[count1] = gps_latitude;
                    GPSlng_array[count1] = gps_longitude;
                    count1++;
                }else{
                    phase4 = 3;
                }
      
            case 3:
                Serial.println("phase4 = 3");
                GPSlat_array[count1] = gps_latitude;
                GPSlng_array[count1] = gps_longitude;
                if(count1 < 4){
                    count1++;
                }else{
                    count1 = 0;
                }
      
                //移動平均
                //一回目はcase1で取った四つのデータと新しい一個のデータを使用．
                for(count2=0;count2<5;count2++){
                    GPSlat_sum  = GPSlat_sum  + GPSlat_array[count2];
                    GPSlng_sum  = GPSlng_sum  + GPSlng_array[count2];
                    count2++;
                }
                GPSlat_data  = GPSlat_sum  / 5;
                GPSlng_data   = GPSlat_sum  / 5;
                delta_lng = GOAL_lng - GPSlng_data;
                distance = 6378.137*pow(10,3)*acos(sin(GPSlat_data*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPSlat_data*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
      
                if(Pre_distance < ditermined_dis){
                phase = 5;
                }else if(Pre_distance < distance ){
                    phase4 = 1;
                    count = 0;
                }    
            }
           }

            //########## 近距離探索フェーズ ##########
            case 5:
                {if(phase_state != 5){
                    //近距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
                    Serial.write("Phase5: transition completed\n"); // 地上局へのデータ
                    Serial.print("LAT(PHASE5_START):");Serial.println(gps_latitude,9);
                    Serial.print("LONG(PHASE5_START):");Serial.println(gps_longitude,9);

                    //LogDataの保存
                    CanSatLogData.println(gps_time);
                    CanSatLogData.println("Phase5: transition completed");  
                    CanSatLogData.print("LAT(PHASE5_START):");CanSatLogData.println(gps_latitude,9);
                    CanSatLogData.print("LONG(PHASE5_START):");CanSatLogData.println(gps_longitude,9);
                    CanSatLogData.flush();

                    phase_state = 5;
                    count = 0;
                }

                current_distance = ultra_distance;
                Serial.println(ultra_distance);
                switch(phase_5){
                  case 5:
                    Serial.print("phase(rotating)=");
                    Serial.println(phase_5);
                    rotating();
                    if(ultra_distance < 500){
                      phase_5 = 0;
                      stopping();
                      Serial.println("STOP!");
                    }
                    break;
                    
                  case 0:
                    Serial.print("phase(check_former)=");
                    Serial.println(phase_5);
                      if(abs(current_distance - previous_distance) < EPSILON){
                        count++;
                      }
                      if(count==5){//イズチェック
                        phase_5 = 1;
                        distance1 = current_distance;
                        count = 0;
                      }
                      previous_distance = current_distance;
                     
                    break;
              
                  case 1:
                    Serial.print("phase(forward)=");
                    Serial.println(phase_5);
                    forward();
                    delay(5000);
                    stopping();
                    phase_5 = 2;    
                    break;
              
                  case 2:
                    Serial.print("phase(check_later)=");
                    Serial.println(phase_5);
                    if(abs(current_distance - previous_distance) < EPSILON){
                      count++;
                    }
                    if(count==5){//イズチェック
                      phase_5 = 3;
                      distance2 = current_distance;
                      count = 0;
                    }
                    previous_distance = current_distance;
                    break;
              
                  case 3:
                    Serial.print("phase(judge)=");
                    Serial.println(phase_5);
                    if(distance2-distance1<0){
                      if(distance2<5){
                      phase_5 = 4;
                      }
                      else{
                      phase_5 = 1;
                      }
                    }else if(distance2-distance1>0){
                      phase_5 = 5;
                    }
                    break;
              
                  case 4:
                    Serial.print("phase(goal)=");
                    Serial.println(phase_5);
                    Serial.println("GOAL!");
                    delay(100000000000);
                    phase_5 = 6;
                    break;
                    
                  default:
                    break;
                }

                break;
                }
        }

        //SDカードへデータを保存する
        SensorData.print(gps_time);
        SensorData.print(",");        
        SensorData.print(gps_latitude);
        SensorData.print(",");
        SensorData.print(gps_longitude);
        SensorData.print(",");
        SensorData.print(gps_velovity);
        SensorData.print(",");
        SensorData.print(Temperature);
        SensorData.print(",");
        SensorData.print(Pressure);
        SensorData.print(",");
        SensorData.print(accelX);
        SensorData.print(",");
        SensorData.print(accelY);
        SensorData.print(",");
        SensorData.print(accelZ);
        SensorData.print(",");
        SensorData.print(heading);
        SensorData.print(",");
        SensorData.print(headingDegrees);
        SensorData.print(",");
        SensorData.print(R);
        SensorData.print(",");
        SensorData.print(G);
        SensorData.print(",");
        SensorData.print(B);
        SensorData.print(",");
        SensorData.println(ultra_distance);      
        SensorData.flush();
    }
   }
 }
}//loop関数の閉じ
