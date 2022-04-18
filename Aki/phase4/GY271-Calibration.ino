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
float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;
float minZ = 0;
float maxZ = 0;
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;
float mag_X;
float mag_Y;
float mag_Z;
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
  unsigned long decided_Millis = millis()+5000;
  Serial.println(millis());
  Serial.println(decided_Millis);
}

void loop() {
  Vector mag = compass.readRaw();
  switch(phase4){
    case 1:
    //rotating();
    if(mag.XAxis > maxX)  {
      maxX = mag.XAxis};
    if(mag.YAxis > maxY)  {
      maxY = mag.YAxis};
    if(mag.ZAxis > maxZ)  {
      maxZ = mag.ZAxis};
  
    if(mag.XAxis < minX)  {
      minX = mag.XAxis};
    if(mag.YAxis > minY)  {
      minY = mag.YAxis};
    if(mag.ZAxis > minZ)  {
      minZ = mag.ZAxis};
    
    current_Millis = millis();
    Serial.println(current_Millis);
    if (current_Millis > decided_Millis){
      Serial.println(current_Millis);
      phase4 = 2;
      //stopping();
      offsetX = maxX + minX;
      offsetY = maxY + minY;
      offsetZ = maxZ + minZ;
    }else{
      Serial.println("here");
    }
    break;
  
    case 2:
    current_Millis = millis();
    Serial.println(current_Millis);
    mag_X = mag.XAxis - offsetX;
    mag_Y = mag.YAxis - offsetY;
    mag_Z = mag.ZAxis - offsetZ;
    Serial.print(mag_X);
    Serial.print(",");
    Serial.print(mag_Y);
    Serial.print(",");
    Serial.println(mag_Z);
    break;
}
}
