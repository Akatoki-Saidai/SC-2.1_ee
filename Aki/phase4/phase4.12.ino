  #include <TinyGPS++.h>
  #include <DFRobot_QMC5883.h>
  #include <math.h>
  DFRobot_QMC5883 compass;
  TinyGPSPlus gps;
  int phase = 4;
  char phase4 = 1;
  float v_initial= 38.0;  //[m/s]
  float g        = 9.80665;  //[m/s/s]
  float rotate_x ;//回転速度(TBD)[deg/s]
  float ditermined_dis = 3; //GPSでどれくらい近づくか(TBD)
  float angle_radian;
  float angle_degree;
  
  //you need to set up variables at first
  float GOAL_lat = 90.862857820;
  float GOAL_lng = 200;

  //those are variables which is used in GPS
  //緯度
  float GPSlat_array[5];
  float GPSlat_sum = 0;
  float GPS_lat ; 
  float GPSlat_data;
  //経度
  float GPSlng_array[5];
  float GPSlng_sum = 0;
  float GPS_lng ; //GPS経度
  float GPSlng_data;
  float delta_lng;
  //距離
  float distance; //直進前後でゴールに近づいているかどうかを表す
  float Pre_distance;

  //those are variables which is used in GY-271
  float heading_data;
  float heading_array[5];
  float heading_sum = 0;
  int count = 0;
  int   count1 = 0;
  int   count2 = 0;
  float until_5;
  float heading;
  float rotate_degree = 0;
  float rotate_sec = 0;
  //Vector norm = compass.readNormalize();
  float headingDegrees;

  //GPSのデータがちゃんと得られてるかどうか一回だけ表示するための変数
  int counter  = 0;
  //phase4の変数を最初の一回だけ表示するための変数
  int counter1 = 0;
  int counter2 = 0;
  int counter3 = 0;
  
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1,5,18);
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
  //データの取得
  //GPS
  if(Serial1.available() > 0){   //アップデートの実行
    delay(1000);
    Serial.println("Here1!");
    //delay(1000);
    char c = Serial1.read();    //GPSチップからのデータを受信
    gps.encode(c);   //GPSチップからのデータの整形

    //if(gps.location.isUpdated()){
      delay(1000);
      Serial.println("Here2!");
      GPS_lat = gps.location.lat();
      GPS_lng = gps.location.lng();
      //GPS_time = gps.time.value();
      if(counter == 0){
        Serial.println("You got the data from the GPS");
        counter++;
      }
  
      //GY271
      Vector norm = compass.readNormalize();
    float heading = atan2(norm.YAxis, norm.XAxis);
    /*
    Serial.println("here");
    delay(1000);
    */
    switch(phase4){
      case 1:
      {
        if(counter1 == 0){
          Serial.println("phase4 = 1");
          counter1++;
        }
        delay(1000);
        //データとるために一時停止
        //stopping();
        Serial.println("stopping!");
  
        //GPSとGY271で5回の平均を最初に出す
        if(count == 0){
          //CanSatの北からの角度を計算
          //float heading = atan2(norm.YAxis, norm.XAxis);
          Serial.print("heading = ");
          Serial.println(heading);
          Serial.print("lat = ");
          Serial.print(GPS_lat);
          Serial.print("   lng = ");
          Serial.println(GPS_lng);
          
          heading_array[count1] = heading;
          GPSlat_array[count1] = GPS_lat;
          GPSlng_array[count1] = GPS_lng;
          if(count1 == 4){
            for(count2=0;count2<5;count2++){
              heading_sum = heading_sum + heading_array[count2];
              GPSlat_sum  = GPSlat_sum  + GPSlat_array[count2];
              GPSlng_sum  = GPSlng_sum  + GPSlng_array[count2];
            }
            heading_data = heading_sum / 5;
            GPSlat_data  = GPSlat_sum  / 5;
            GPSlng_data   = GPSlat_sum  / 5;
            count = 1;
            count1 = 0;
          }else{
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
          if (heading_data < 0){
            heading_data += 2 * PI;
          }else if (heading_data > 2 * PI){
            heading_data -=2 * PI;
          }
          //radからdegに変換
          float headingDegrees = heading_data * 180/M_PI;
          
          // Output付け足したところ
          Serial.print(" Heading_data = ");
          Serial.print(heading_data);
          Serial.print(" Degress = ");
          Serial.println(headingDegrees);
          Serial.print("lat = ");
          Serial.print(GPS_lat);
          Serial.print("   lng = ");
          Serial.println(GPS_lng);
          Serial.print("angle_degree = ");
          Serial.println(angle_degree);
          delay(100);
          //Serial.println(")
          //回転する角度rotate_degreeで
        if(heading_data > angle_degree){
          if(heading_data > 180 + angle_degree){ //右回転
            rotate_degree = angle_degree - heading_data + 360;
            rotate_sec = rotate_degree / rotate_x;
            //rightturn();
            Serial.println("turn right");        
            delay(rotate_sec);
          }else{//左回転
            rotate_degree = heading_data - angle_degree;
            rotate_sec = rotate_degree / rotate_x; 
            //leftturn();
            Serial.println("turn left");
            delay(rotate_sec);
          }
        }else if(angle_degree >= heading_data){
          if(angle_degree > 180 + heading_data){//左回転
            rotate_degree = heading_data - angle_degree + 360;
            rotate_sec = rotate_degree / rotate_x;
            //leftturn();
            Serial.println("turn left");
            delay(rotate_sec);
          }else{//右回転
            rotate_degree = angle_degree - heading_data;
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
        counter1 = 0;
  
        //階差を求めたいので，求めたデータは過去のデータに．
        Pre_distance = distance;
        }
        break;
      }
      
      case 2:
        if(counter2 == 0){
          Serial.println("phase4 = 2");
          counter2++;
        }
        delay(1000);
        if(count1 < 4){
          GPSlat_array[count1] = GPS_lat;
          GPSlng_array[count1] = GPS_lng;
          count1++;
        }else{
          phase4 = 3;
          counter2 = 0;
        }
        break;
        
      case 3:
        if(counter3 == 0){
          Serial.println("phase4 = 3");
          counter3++;
        }
        delay(1000);
        GPSlat_array[count1] = GPS_lat;
        GPSlng_array[count1] = GPS_lng;
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
          Serial.println("finished!!!!!!!");
          delay(100000);
        }else if(Pre_distance < distance ){
          phase4 = 1;
          count = 0;
          counter3 = 0;
        }
        break;    
      }
//    }
  }
}
