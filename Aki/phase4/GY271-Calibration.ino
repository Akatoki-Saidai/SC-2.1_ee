/*!
 * @file QMC5883_readRaw.cpp
 * @brief show raw data
 * @n 3-Axis Digital Compass IC
 *
 * @copyright  [DFRobot](http://www.dfrobot.com), 2017
 * @copyright GNU Lesser General Public License
 *
 * @author [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 */

#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass;
int phase4 = 1;
float minX;
float maxX;
float minY;
float maxY;
float minZ;
float maxZ;
float offsetX;
float offsetY;
float offsetZ;
float mag_X;
float mag_Y;
float mag_Z;
double theta;
unsigned long current_Millis;
unsigned long previous_Millis;
unsigned long decided_Millis;

void setup() {
  Serial.begin(115200);

  // Initialize Initialize QMC5883
  while (!compass.begin()){
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  if(compass.isHMC() ){
    Serial.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  }else if(compass.isQMC()){
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS); 
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  delay(1000);
  decided_Millis = millis()+10000;
  Serial.println(millis());
  Serial.println(decided_Millis);
}

void loop() {
  Vector mag = compass.readRaw();
  switch(phase4){
    case 1:
    {
      maxX = mag.XAxis;
      maxY = mag.YAxis;
      maxZ = mag.ZAxis;
      minX = mag.XAxis;
      minY = mag.YAxis;
      minZ = mag.ZAxis;
      phase4 = 2;
    }

    case 2:
    {
    //rotating();
    if(mag.XAxis > maxX) maxX = mag.XAxis;
    if(mag.YAxis > maxY) maxY = mag.YAxis;
    if(mag.ZAxis > maxZ) maxZ = mag.ZAxis;

    if(mag.XAxis < minX) minX = mag.XAxis;
    if(mag.YAxis < minY) minY = mag.YAxis;
    if(mag.ZAxis < minZ) minZ = mag.ZAxis;

    current_Millis = millis();
    //Serial.println(current_Millis);
    if (current_Millis > decided_Millis){
      Serial.println(current_Millis);
      Serial.println(decided_Millis);
      phase4 = 3;
      //stopping();
      offsetX = (maxX + minX)/2;
      offsetY = (maxY + minY)/2;
      offsetZ = (maxZ + minZ)/2;
      Serial.print("max=  ");
      Serial.print(maxX);
      Serial.print(",");
      Serial.print(maxY);
      Serial.print(",");
      Serial.println(maxZ);

      Serial.print("min=  ");
      Serial.print(minX);
      Serial.print(",");
      Serial.print(minY);
      Serial.print(",");
      Serial.println(minZ);
      
      Serial.print("off=  ");
      Serial.print(offsetX);
      Serial.print(",");
      Serial.print(offsetY);
      Serial.print(",");
      Serial.println(offsetZ);
    }
    break;
    }
    case 3:
    {
    current_Millis = millis();
    //Serial.println(current_Millis);
    /*
    mag_X = mag.XAxis;
    mag_Y = mag.YAxis;
    mag_Z = mag.ZAxis;
    Serial.print(mag_X);
    Serial.print(",");
    Serial.print(mag_Y);
    Serial.print(",");
    Serial.println(mag_Z);
    */
    mag_X = mag.XAxis - offsetX;
    mag_Y = mag.YAxis - offsetY;
    mag_Z = mag.ZAxis - offsetZ;
    Serial.print(mag_X);
    Serial.print(",");
    Serial.print(mag_Y);
    Serial.print(",");
    Serial.println(mag_Z);
    
    //theta = atan2(mag_Y,mag_X);
    //Serial.println(theta);
    

    //delay(10000);
    break;
    }
  }
}
