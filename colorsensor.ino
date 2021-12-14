#include <Wire.h>
void setup() {
     Serial.begin(115200);
     Wire.begin();
     Wire.beginTransmission(0x39);
     Wire.write(0x44);
     Wire.write(0x01);
     Wire.endTransmission();
     Wire.beginTransmission(0x39);
     Wire.write(0x42);
     Wire.write(0x12);
     Wire.endTransmission();
}
void loop() {
     int i;
     Wire.beginTransmission(0x39);
     Wire.write(0x50);
     Wire.endTransmission();
     Wire.requestFrom(0x39, 8);
     for (i=1 ; i<5; i++){
       int c1 = Wire.read();
       int c2 = Wire.read();
       c1 = c1  + c2 * 256;
       switch (i) {
         case 1: Serial.print("R =");   break;
         case 2: Serial.print(" , G =");break;
         case 3: Serial.print(" , B =");break;
         case 4: Serial.print(" , C =");break; }
       Serial.print(c1);
       if (i==4) Serial.println();
    }    
    delay(5000);
}
