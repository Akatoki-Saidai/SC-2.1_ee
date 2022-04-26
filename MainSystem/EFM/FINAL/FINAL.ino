#include <TinyGPS++.h>
#include <DFRobot_QMC5883.h>
#include <math.h>

#define rad2deg(a) ((a) / M_PI * 180.0) /* rad を deg に換算するマクロ関数 */
#define deg2rad(a) ((a) / 180.0 * M_PI) /* deg を rad に換算するマクロ関数 */

double aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

DFRobot_QMC5883 compass;
TinyGPSPlus gps;

int phase = 4;
char key = '0';
const int SAMPLING_RATE = 200;
int phase_state = 0;

// for phase3
int type = 1;
int yeah = 1;
int type_state = 0;
int cutparac = 23;                //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 5;          //切り離し時の9V電圧を流す時間，単位はsecond
double time3_1, time3_2, St_Time; //時間に関するもの
double Accel[6];                  //計測した値をおいておく関数
double Altitude[6];               //(高度)
double Preac, differ1, Acsum, Acave, RealDiffer1;
double Preal, differ2, Alsum, Alave, RealDiffer2;
int i_3 = 0;
int j_3 = 0;
double RealDiffer;

// for MPU6050
#include <MPU9250_asukiaaa.h>
#ifdef _ESP32_HAL_I2C_H_
#define SDA_MPU 21 // I2C通信
#define SCL_MPU 22
#endif
MPU9250_asukiaaa mySensor;

// for BMP180
#include <Wire.h> //I2C通信
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#define SDA_BMP 21
#define SCL_BMP 22

// for SD Card
#include <SPI.h> //SDカードはSPI通信
#include <SD.h>
File CanSatLogData;
File SensorData;
const int sck = 13, miso = 15, mosi = 14, ss = 27;

//センサー値の格納
double Temperature, Pressure, accelX, accelY, accelZ, magX, magY, magZ, gyroX, gyroY, gyroZ, accelSqrt = 0, gps_latitude, gps_longitude, gps_velocity, altitude = 0;
int gps_time, R, G, B;
long ultra_distance;

// for phase1,2
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

// for phase4
int CalibrationCounter = 1;
int calibration = 1;
int sum_count = 0;
double CurrentDistance;
double heading;
double declinationAngle;
double headingDegrees;
double Angle_gy270;
double Angle_Goal;
double Angle_gps;
double Angle_heading;
double rrAngle, llAngle;
double sum_latitude = 0;
double sum_longitude = 0;


// you need to set up variables at first
double GOAL_lat = 35.860595000;
double GOAL_lng = 139.606911667;

// variables___GPS
//緯度
double GPSlat_array[5];
double GPSlat_sum = 0;
double GPSlat_data;
//経度
double GPSlng_array[5];
double GPSlng_sum = 0;
double GPSlng_data;
double delta_lng;
//距離
double distance; //直進前後でゴールに近づいているかどうかを表す
double Pre_distance;
// variables___GY-271
double heading_data;
double heading_array[5];
double heading_sum = 0;
double omega;
double azimuth;

// GPSのデータがちゃんと得られてるかどうか一回だけ表示するための変数
int counter = 0;
// phase4の変数を最初の一回だけ表示するための変数
int counter1 = 0;
int counter2 = 0;
int counter3 = 0;

int Colorsensor_Phase = 1;
int RED;
int BLUE;
int GREEN;
int Length;
int Distance_to_Goal = 0; //ゴールまでの距離何メートル以内であってほしいか
int Color_count = 1;

// for phase5
// those are variables which is used in 超音波
int count = 0;
int phase_5 = 5;
int previous_distance = 0;
int current_distance;
int EPSILON = 5;
int distance1, distance2;

#include "SR04.h"
#define TRIG_PIN 0
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
int forward_phase = 1;
unsigned long previousMillis = 0;
int Priviousdistance;
int i = 255;
unsigned long time1;
unsigned long time2;

double Sum_headingDegrees;

double w = 1; //地磁気センサーの信頼係数

double desiredDistance = 1.5; //遠距離フェーズから近距離フェーズに移行する距離

double pre_gps_latitude, pre_gps_longitude;
int LongDis_phase = 0;

volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt timer function
void IRAM_ATTR onTimer1()
{
  portENTER_CRITICAL_ISR(&timerMux);
  timeCounter1++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  double EarthRadius = 6378.137; //

  //目標地点までの距離を導出
  delta_lng = GOAL_lng - gps_longitude;

  distance = EarthRadius * acos(sin(gps_latitude) * sin(GOAL_lat) + cos(gps_latitude) * cos(GOAL_lat) * cos(delta_lng)) * 1000;

  return distance;
}

//角度計算用の関数
double CalculateAngle(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  //目標地点までの角度を導出
  delta_lng = GOAL_lng - gps_longitude;
  azimuth = rad2deg(atan2(sin(delta_lng), cos(gps_latitude) * tan(GOAL_lat) - sin(gps_latitude) * cos(delta_lng)));

  if (azimuth < 0)
  {
    azimuth += 360;
  }
  else if (azimuth > 360)
  {
    azimuth -= 360;
  }
  return azimuth;
}

//前進
void forward()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 127);
  ledcWrite(2, 0);
  ledcWrite(3, 107);
}

//ターボ
void turbo()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 255);
  ledcWrite(2, 0);
  ledcWrite(3, 235);
}

//後転
void back()
{
  ledcWrite(0, 0);
  ledcWrite(1, 63);
  ledcWrite(2, 0);
  ledcWrite(3, 63);
}

//停止
void stoppage()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

//右旋回
void leftturn()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 127);
  ledcWrite(3, 0);
}

//左旋回
void rightturn()
{
  ledcWrite(0, 127);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

//回転
void rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 70);
  ledcWrite(2, 70);
  ledcWrite(3, 0);
}

//ゆっくり回転
void phase4_rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 60);
  ledcWrite(2, 60);
  ledcWrite(3, 0);
}

//反回転
void reverse_rotating()
{
  ledcWrite(0, 70);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 70);
}

//ゆっくり加速
void accel()
{
  for (int i = 0; i < 127; i = i + 5)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(80); // accelではdelay使う
  }
}

//ゆっくり停止
void stopping()
{
  for (int i = 127; i > 0; i = i - 5)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(80); // stoppingではdelay使う
  }
}

void setup()
{
  Serial1.begin(115200, SERIAL_8N1, 5, 18);

  // mpuが正常動作するためのプログラム
  while (!Serial)
    ;
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_MPU, SCL_MPU);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  uint8_t sensorId;
  if (mySensor.readId(&sensorId) == 0)
  {
    Serial.println("sensorId: " + String(sensorId));
  }
  else
  {
    Serial.println("Cannot read sensorId");
  }

  if (mySensor.accelUpdate() == 0)
  {
    aX = mySensor.accelX() + 0.00;
    aY = mySensor.accelY() + 0.00;
    aZ = mySensor.accelZ() + 0.00;
    aSqrt = mySensor.accelSqrt();
    Serial.println("accelX: " + String(aX));
    Serial.println("accelY: " + String(aY));
    Serial.println("accelZ: " + String(aZ));
    Serial.println("accelSqrt: " + String(aSqrt));
  }
  else
  {
    Serial.println("Cannod read accel values");
  }

  if (mySensor.gyroUpdate() == 0)
  {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.println("gyroX: " + String(gX));
    Serial.println("gyroY: " + String(gY));
    Serial.println("gyroZ: " + String(gZ));
  }
  else
  {
    Serial.println("Cannot read gyro values");
  }

  if (mySensor.magUpdate() == 0)
  {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("horizontal direction: " + String(mDirection));
  }
  else
  {
    Serial.println("Cannot read mag values");
  }

  Serial.println("at " + String(millis()) + "ms");
  Serial.println(""); // Add an empty line
  delay(500);
  // MPUが正常動作するためのプログラム終了

  // SD Card initialization
  SPI.begin(sck, miso, mosi, ss);
  SD.begin(ss, SPI);
  CanSatLogData = SD.open("/CanSatLogData.log", FILE_APPEND);
  SensorData = SD.open("/SensorData.log", FILE_APPEND);

  //割り込み関数
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 1.0E6 / SAMPLING_RATE, true);
  timerAlarmEnable(timer1);
  //無線通信
  Serial.begin(115200);
  // Serial.begin(115200, SERIAL_8N1, 16, 17); //関数内の引数はデータ通信レート，unknown，RXピン，TXピン
  Serial.write("TESTING: Serial communication\n");
  Serial.write("TESTING: Serial communication\n");
  CanSatLogData.println("START_RECORD");
  CanSatLogData.flush();
  SensorData.println("gps_time,gps_latitude,gps_longitude,gps_velocity,Temperature,Pressure,accelX,accelY,accelZ,heading,headingDegrees,R,G,B,ultra_distance");
  SensorData.flush();

  // for BMP
  bmp.begin();

  // for motor
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);

  // for GY-271
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

  if (compass.isHMC())
  {
    Serial.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  }
  else if (compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  leftturn();
  delay(100);
  Serial.println("calibration rotating!");
  while (CalibrationCounter < 551)
  {
    Vector norm = compass.readNormalize();
    rotating(); // testcodeでは手動で回す．
    if (CalibrationCounter == 550)
    {
      stoppage();
      Serial.println("calibration stopping!");
      delay(2000);
      calibration = 2;
      CalibrationCounter = CalibrationCounter + 1;
    }
    else
    {
      CalibrationCounter = CalibrationCounter + 1;
      Serial.print("CalibrationCounter = ");
      Serial.println(CalibrationCounter);
    }
  }
  // for colorsensor
  Wire.begin();
  Wire.beginTransmission(0x39);
  Wire.write(0x44);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(0x39);
  Wire.write(0x42);
  Wire.write(0x12);
  Wire.endTransmission();

} // setup関数閉じ

void loop()
{
  if (Serial1.available() > 0)
  { //これないとGPS使えない
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated())
    { //この中でgpsのデータを使わないと，うまく値が表示されない
      if (timeCounter1 > 0)
      {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);

        //起動時刻の更新
        unsigned long currentMillis = millis();

        //センサー値のアップデート
        mySensor.accelUpdate();

        //センサー値取得
        // BMP180
        Temperature = bmp.readTemperature();
        Pressure = bmp.readPressure();
        if (mySensor.accelUpdate() == 0)
        {
          accelX = mySensor.accelX() + 0;
          accelY = mySensor.accelY() + 0;
          accelZ = mySensor.accelZ() + 0;
          accelSqrt = mySensor.accelSqrt();
        }

        altitude = bmp.readAltitude();


        /*gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();*/

        while(sum_count <3)
        {
          // GPS
          char c = Serial1.read(); // GPSチップからのデータを受信
          gps.encode(c);           // GPSチップからのデータの整形
          sum_latitude += gps.location.lat();
          sum_longitude += gps.location.lng();       
          sum_count++;
        }
        sum_count = 0;
        gps_latitude = sum_latitude/3;
        gps_longitude = sum_longitude/3;
        sum_latitude = 0;
        sum_longitude = 0;
         
        gps_time = gps.time.value();
        gps_velocity = gps.speed.mps();
        //超音波
        ultra_distance = sr04.Distance();
        //カラーセンサー
        int i_c;
        Wire.beginTransmission(0x39);
        Wire.write(0x50);
        Wire.endTransmission();
        Wire.requestFrom(0x39, 8);
        for (i_c = 1; i_c < 5; i_c++)
        {
          int c1 = Wire.read();
          int c2 = Wire.read();
          c1 = c1 + c2 * 256;
          switch (i_c)
          {
          case 1:
            Serial.print("R =");
            R = c1;
            break;
          case 2:
            Serial.print(" , G =");
            G = c1;
            break;
          case 3:
            Serial.print(" , B =");
            B = c1;
            break;
          case 4:
            Serial.print(" , C =");
            break;
          }
          Serial.print(c1);
          if (i_c == 4)
            Serial.println();
        }

        Serial.print(gps_time);
        Serial.print(",");
        Serial.print(gps_latitude, 9);
        Serial.print(",");
        Serial.print(gps_longitude, 9);
        Serial.print(",");
        Serial.print(gps_velocity);
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
        switch (phase)
        {

          //########## 待機フェーズ ##########
        case 1:
          if (phase_state != 1)
          {
            //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase1: transition completed\n"); // 地上局へのデータ送信
            // LogDataの保存
            CanSatLogData.println(gps_time);
            CanSatLogData.println("Phase1: transition completed");
            CanSatLogData.flush();

            phase_state = 1;
          }
          if (mode_to_bmp == 0)
          {
            if (accelZ < -2)
            { //落下開始をまずMPUで判定
              altitude_sum_mpu += altitude;
              count1++;
              if (count1 == 1)
              {
                count1 = 0;
                CanSatLogData.println(gps_time);
                CanSatLogData.println("FALL STARTED(by MPU)\n");
                CanSatLogData.flush();
                mode_to_bmp = 1;
              }
            }
          }
          else
          {
            switch (mode_comparison)
            { //落下開始をBMPで判定
            case 0:
              previous_millis = millis();
              altitude_sum_bmp += altitude;
              count3++;
              if (count3 == 5)
              {
                previous_altitude = altitude_sum_bmp / 5;
                altitude_sum_bmp = 0;
                count3 = 0;
                mode_comparison = 1;
              }
              break;
            case 1: // 500ms後
              current_millis = millis();
              if (current_millis - previous_millis >= 500)
              {
                altitude_sum_bmp += altitude;
                count3++;
                if (count3 == 5)
                {
                  current_altitude = altitude_sum_bmp / 5;
                  CanSatLogData.println(currentMillis);
                  CanSatLogData.println("current_altitude - previous_altitude = \n");
                  CanSatLogData.println(current_altitude - previous_altitude);
                  if (current_altitude - previous_altitude <= -1.0)
                  {
                    CanSatLogData.println("FALL STARTED(by BMP)\n");
                    CanSatLogData.flush();
                    phase = 2;
                  }
                  else
                  {
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
          if (phase_state != 2)
          {
            //降下フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase2: transition completed\n"); //地上局へのデータ送信

            // LogDataの保存
            CanSatLogData.println(gps_time);
            CanSatLogData.println("Phase2: transition completed");
            CanSatLogData.flush();

            i_3 = 0;
            j_3 = 0;
            Preac = 0; // 1個前の加速度を記憶
            Preal = 0;
            differ1 = 0.1;   //加速度　移動平均の差
            differ2 = 0.5;   //高度　　移動平均の差
            Acave = 0;       //加速度　5個の平均値
            Alave = 0;       //高度　　5個の平均値
            Acsum = 0;       //加速度　5個の合計値
            Alsum = 0;       //高度　　5個の合計値
            RealDiffer1 = 0; // 1秒前との差を記憶する
            RealDiffer2 = 0;

            phase_state = 2;
          }

          if (yeah == 1)
          { //データを初めから五個得るまで
            Accel[i_3] = accelSqrt;
            Altitude[i_3] = altitude;
            i_3 = i_3 + 1;
            if (i_3 == 6)
            { // 5個得たらその時点での平均値を出し，次のフェーズへ
              yeah = 2;
              i_3 = 0; // iの値をリセット
              for (j_3 = 1; j_3 < 6; j_3++)
              { // j_3=0の値は非常に誤差が大きいので1から
                Acsum = Acsum + Accel[j_3];
                Alsum = Alsum + Altitude[j_3];
              }
              Acave = Acsum / 5;
              Alave = Alave / 5;
              time3_2 = currentMillis;
            }
          }
          else
          {
            Preac = Acave;
            Preal = Alave;
            Accel[i_3] = accelSqrt;
            Altitude[i_3] = altitude;
            for (j_3 = 0; j_3 < 5; j_3++)
            {
              Acsum = Acsum + Accel[j_3];
              Alsum = Alsum + Altitude[j_3];
            }
            Acave = Acsum / 5;
            Alave = Alsum / 5;
            RealDiffer1 = Preac - Acave;
            RealDiffer2 = Preal - Alave;
            if (i_3 == 5)
            {
              i_3 = 0;
              Acsum = 0;
              Alsum = 0;
            }
            else
            {
              i_3 = i_3 + 1;
              Acsum = 0;
              Alsum = 0;
            }
            if (currentMillis - time3_2 > 1000)
            {
              if (RealDiffer1 < differ1)
              { //移動平均が基準以内の変化量だった時
                phase = 3;
              }
              else if (RealDiffer2 < differ2)
              {
                phase = 3;
              }
              else
              {
                time3_2 = currentMillis;
              }
            }
          }

          break;
        //########## 分離フェーズ ##########
        case 3:
          if (phase_state != 3)
          {
            //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase3: transition completed\n");
            // LogDataの保存
            CanSatLogData.println(gps_time);
            CanSatLogData.println("Phase3: transition completed");
            CanSatLogData.flush();
            phase_state = 3;
            time3_1 = currentMillis;                    // phase3　開始時間の保存
            St_Time = time3_1 + outputcutsecond * 1000; //基準時間
            Serial.write("WARNING: The cut-para code has been entered.\n");
            digitalWrite(cutparac, HIGH); //オン
            Serial.write("WARNING: 9v voltage is output.\n");
            // LogDataの保存
            CanSatLogData.println(currentMillis);
            CanSatLogData.println("9v voltage is output");
            CanSatLogData.flush();
          }

          if (currentMillis > St_Time)
          {                              //電流を流した時間が基準時間を超えたら
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
        {
          if (phase_state != 4){
            //遠距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase4: transition completed\n");
            Serial.print("LAT(PHASE4_START):");
            Serial.println(gps_latitude, 9);
            Serial.print("LONG(PHASE4_START):");
            Serial.println(gps_longitude, 9);

            // LogDataの保存
            CanSatLogData.println(gps_time);
            CanSatLogData.println("Phase4: transition completed");
            CanSatLogData.print("LAT(PHASE4_START):");
            CanSatLogData.println(gps_latitude, 9);
            CanSatLogData.print("LONG(PHASE4_START):");
            CanSatLogData.println(gps_longitude, 9);
            CanSatLogData.flush();

            phase_state = 4;
          }

          CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
          Serial.print("CurrentDistance=");
          Serial.println(CurrentDistance);

          if (desiredDistance >= CurrentDistance)
          {
            // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
            phase = 5;
          }else{
            delay(100);
            accel();
            forward();
            delay(1000);
            stopping();
            // Goalまでの偏角を計算する
            Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);

            Sum_headingDegrees = 0.0;
              for (i = 0; i < 15; i++){
                delay(10);
                Vector norm = compass.readNormalize();
                heading = atan2(norm.YAxis, norm.XAxis);
                declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
                heading += declinationAngle;
                if (heading < 0){
                  heading += 2 * PI;
                }
                if (heading > 2 * PI){
                  heading -= 2 * PI;
                }

                // Convert to degrees
                headingDegrees = heading * 180 / M_PI;

                if (headingDegrees < 0){
                  headingDegrees += 360;
                }

                if (headingDegrees > 360){
                  headingDegrees -= 360;
                }
                Sum_headingDegrees += headingDegrees;
              }
              Angle_gy270 = Sum_headingDegrees / 15;
        }
        // どちらに回ればいいか計算
          rrAngle = -Angle_gy270 + Angle_Goal;
          if (rrAngle < 0){
            rrAngle += 360;
          }
          if (rrAngle > 360){
            rrAngle -= 360;
          }
          llAngle = Angle_gy270 - Angle_Goal;
          if (llAngle < 0){
            llAngle += 360;
          }
          if (llAngle > 360){
            llAngle -= 360;
          }

        if (rrAngle > llAngle){
          //反時計回り
          if (llAngle > 20){
            leftturn();
            delay(100);
            rotating();
            delay(400);
            stoppage();
          }
        }else{
          //時計回り
          if (rrAngle > 20){
            rightturn();
            delay(100);
            reverse_rotating();
            delay(400);
            stoppage();
          }
        }
        break;
        }

        //########## 近距離探索フェーズ ##########
        case 5:
        {
          if (phase_state != 5)
          {
            //近距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase5: transition completed\n"); // 地上局へのデータ
            Serial.print("LAT(PHASE5_START):");
            Serial.println(gps_latitude, 9);
            Serial.print("LONG(PHASE5_START):");
            Serial.println(gps_longitude, 9);

            // LogDataの保存
            CanSatLogData.println(gps_time);
            CanSatLogData.println("Phase5: transition completed");
            CanSatLogData.print("LAT(PHASE5_START):");
            CanSatLogData.println(gps_latitude, 9);
            CanSatLogData.print("LONG(PHASE5_START):");
            CanSatLogData.println(gps_longitude, 9);
            CanSatLogData.flush();

            phase_state = 5;
            count = 0;
          }

          current_distance = ultra_distance;
          Serial.println(ultra_distance);
          switch (phase_5)
          {
          case 5:
            Serial.print("phase(rotating)=");
            Serial.println(phase_5);
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("ROTATING PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.println("searching for the goal...");
            CanSatLogData.flush();
            leftturn();
            delay(100);
            phase4_rotating();
            if (ultra_distance < 600 && ultra_distance != 0)
            {
              phase_5 = 0;
              stoppage();
              Serial.println("STOP!");
              CanSatLogData.println("-----------------------");
              CanSatLogData.println("STOPPAGE PHASE");
              CanSatLogData.print("GPS TIME:");
              CanSatLogData.println(gps_time);
              CanSatLogData.println("goal is detected!");
              CanSatLogData.flush();
            }
            break;

          case 0:
            Serial.print("phase(check_former)=");
            Serial.println(phase_5);
            if (abs(current_distance - previous_distance) < EPSILON)
            {
              count++;
            }
            if (count == 5)
            { //イズチェック
              phase_5 = 1;
              time1 = millis();
              distance1 = current_distance;
              count = 0;
              accel();
            }
            previous_distance = current_distance;

            break;

          case 1:
            Serial.print("phase(forward)=");
            Serial.println(phase_5);
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("FORWARD PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.println("moving for 1000[ms]");
            CanSatLogData.flush();
            time2 = millis();
            forward();
            if (time2 - time1 >= 1000)
            {
              stopping();
              phase_5 = 2;
            }
            break;

          case 2:
            Serial.print("phase(check_later)=");
            Serial.println(phase_5);
            if (abs(current_distance - previous_distance) < EPSILON)
            {
              count++;
            }
            if (count == 5)
            { //イズチェック
              phase_5 = 3;
              distance2 = current_distance;
              count = 0;
            }
            previous_distance = current_distance;
            break;

          case 3:
            Serial.print("phase(judge)=");
            Serial.println(phase_5);
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("JUDGEING PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.flush();
            if (distance2 - distance1 < 0)
            {
              if (distance2 < 100)
              {
                phase_5 = 4;
                CanSatLogData.println("goal!");
                CanSatLogData.flush();
              }
              else
              {
                phase_5 = 1;
                CanSatLogData.println("approaching!");
                CanSatLogData.flush();
              }
            }
            else
            {
              phase_5 = 5;
              CanSatLogData.println("receding...");
              CanSatLogData.flush();
            }
            break;

          case 4:
            Serial.print("phase(goal)=");
            Serial.println(phase_5);
            Serial.println("GOAL!");
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("STOPPING PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.println("conglatulations!");
            phase_5 = 6;
            break;

          default:
            break;
          }

          break;
        }
        }

        // SDカードへデータを保存する
        SensorData.print(gps_time);
        SensorData.print(",");
        SensorData.print(gps_latitude, 9);
        SensorData.print(",");
        SensorData.print(gps_longitude, 9);
        SensorData.print(",");
        SensorData.print(gps_velocity);
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
} // loop関数の閉じ
