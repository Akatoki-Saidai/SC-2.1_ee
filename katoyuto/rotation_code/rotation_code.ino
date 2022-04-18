double Angular_velocity = 253; //[deg/sec]
double Angle_of_rotation = 90; //[deg]
double Time_of_rotation;
int Time;
int previousMillis;
int currentMillis;
int i;


void setup() {
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
  
  rotating1();
  delay(5000);
  stoppage();
  delay(1000);
  
}

//前進
void rotating1(){
  ledcWrite(0, 0); //channel, duty
  ledcWrite(1, 30);
  ledcWrite(2, 60);
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
  Time_of_rotation = Angle_of_rotation / Angular_velocity;
  Serial.print(Time_of_rotation*1000);Serial.println("[ms]");
  ledcWrite(0, 30);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 30);
  delay(Time_of_rotation*1000*1.5);
}
//反回転
void reverse_rotating(){
  ledcWrite(0, 0);
  ledcWrite(1, 63);
  ledcWrite(2, 63);
  ledcWrite(3, 0);  
}
