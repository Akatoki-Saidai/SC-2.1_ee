  #include <TinyGPS++.h>
  #include <DFRobot_QMC5883.h>
  #include <math.h>
  DFRobot_QMC5883 compass;
  TinyGPSPlus gps;
  int phase = 4;
  int GPSi=0;
  int GPSn=0;
  int GPSj=0;
  // float gpslat[10];
  // float sum_lat;
  // float gpslng[10];
  // float sum_lng;
  float v_initial= 38.0;  //[m/s]
  float g        = 9.80665;  //[m/s/s]
  float rotate_x ;//回転速度(TBD)[deg/s]
  float ditermined_dis = 3; //GPSでどれくらい近づくか(TBD)
  //ゴールとの計算結果を記憶するやつ
  float distance_before;
  float distance_after;
  float angle_radian;
  float angle_degree;

  //you need to set up variables at first
  float GOAL_lat = 35.862857820;
  float GOAL_lng = 139.607681275;

  
  //those are variables which is used in GPS
  float GPS_lat ; //GPS緯度
  float GPS_lng ; //GPS経度
  float delta_lng;
  float distance; //直進前後でゴールに近づいているかどうかを表す
 

  //those are variables which is used in GY-271
  //DFRobot_QMC5883 compass;
  //float norm ;
  float  heading;
  float rotate_degree = 0;
  float rotate_sec = 0;
  //Vector norm = compass.readNormalize();
  float headingDegrees;

void setup() {
  Serial.begin(115200);
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

  //it will be if(phase state !=4)
  Serial.write("phase4: transition completed! \n");

}

void loop() {
  Vector norm = compass.readNormalize();
  if(GPSi == 0){//ゴールまで離れているため，GPSでどれくらい移動するかを計算．
    //stopping(); //GPSで測るため一回停止
    Serial.println("stopping!");
    
    //GPSで値を取得
    //緯度・経度からゴールとの距離を計測
    distance_before = 6378.137*pow(10,3)*acos(sin(GPS_lat*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPS_lat*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));
    angle_radian = asin((distance*g)/pow(v_initial,2.0))/2.0;
    angle_degree = angle_radian*360.0/(2.0*M_PI);

    //回転すべきradを計算
    //CanSatの北からの角度を計算
    float heading = atan2(norm.YAxis, norm.XAxis);

    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
    heading += declinationAngle;
    
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0){
      heading += 2 * PI;
    }else if (heading > 2 * PI){
      heading -= 2 * PI;
    }
    //角度に変換
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
    
    //前に進む
    //forward();

    //カウンターを1にする．
    GPSi=1;
    
  }else{
    //移動後のゴールとの距離
    delta_lng=GOAL_lng-GPS_lng;
    distance_after = 6378.137*pow(10,3)*acos(sin(GPS_lat*2*M_PI/360)*sin(GOAL_lat*2*M_PI/360)+cos(GPS_lat*2*M_PI/360)*cos(GOAL_lat*2*M_PI/360)*cos(delta_lng*2*M_PI/360));

    //(移動後)-(移動前)
    distance = distance_after - distance_before;
    if(distance < 0){
      if(distance_after < ditermined_dis){
        phase = 5;
      }
    }else{//進む方向が間違えてた時
      GPSi=0;
    }
    
  }

}
