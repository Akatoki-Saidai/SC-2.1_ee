int phase = 1;
int forward_phase = 1;
unsigned long previousMillis = 0;

int i=255;

unsigned long Offtime = 5000;
unsigned long Ontime = 2000;

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
  unsigned long currentMillis = millis();
  switch(phase){
    case 1:
      if (forward_phase == 1){
        stoppage();
        Serial.println("phase = 1");
        previousMillis = currentMillis;
        forward_phase = 2;
      }
      else if ((forward_phase == 2)&&(currentMillis - previousMillis >=500)){
        forward();
        Serial.println("moving");
        previousMillis = currentMillis;
        forward_phase = 3;
      }
      else if((forward_phase == 3) && (currentMillis - previousMillis >=5000)){
        Serial.println("transit stopping phase");
        previousMillis = currentMillis;
        phase = 2;
      }
      break;
      
    case 2:
      stopping();
      break;
      
  }
}


//前進
void forward(){
  ledcWrite(0, 255); //channel, duty
  ledcWrite(1, 0);
  ledcWrite(2, 255);
  ledcWrite(3, 0);
}

//後転
void back(){
  ledcWrite(0, 0);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
    ledcWrite(3, 255);

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
  ledcWrite(0, 150);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

//左旋回
void leftturn(){
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 150);
  ledcWrite(3, 0);
}


//ゆっくり停止
void stopping(){
  unsigned long currentMillis = millis();
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
    Serial.println("hello");
    stoppage();
}
}
