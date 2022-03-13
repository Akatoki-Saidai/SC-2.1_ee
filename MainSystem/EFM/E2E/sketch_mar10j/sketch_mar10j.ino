int phase=1;
void setup() {
  // put your setup code here, to run once:
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
  if (phase ==1){
    rotating();
    Serial.println("rottating");
    delay(10000);
    stopping();
    phase=2;
  }
  else if (phase==2){
    rightturn();
    Serial.println("rightturn");
    delay(10000);
    stopping();
    phase=3;
  }
  else if (phase==3){
    leftturn();
    Serial.println("leftturn");
    delay(10000);
    stopping();
    phase=4;
  }
  
  


}
//前進
void forward(){
  ledcWrite(0, 127); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 127);
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

//ゆっくり停止
void stopping(){
  int i=63;
  while (i>0){
    ledcWrite(0,i);
    ledcWrite(1,0);
    ledcWrite(2,i);
    ledcWrite(3,0);
    delay(100);
    i=  i-5;
  }
}
