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
  float heading_data;
  float heading[5];
  float heading_sum;
  float count1;
  float count2;
  float until_5;
  float rotate_degree = 0;
  float rotate_sec = 0;
  //Vector norm = compass.readNormalize();
  float headingDegrees;
