#include <Wire.h>  // enables I2C communication between arduino and BNO055
#include <Adafruit_Sensor.h> // general sensor library
#include <Adafruit_BNO055.h> //lib to control and read IMU
#include <math.h> //needed dor atan,sin,cos,M_PI

Adafruit_BNO055 bno = Adafruit_BNO055(55); //object to communicate with the sensor, 55 is ID(usefull if you have multiple sensor)

void setup() {
  Serial.begin(9600);
  if (!bno.begin()) { //srnsor not detected
    Serial.println("BNO055 not detected!");
    while (1);
  }
  delay(1000); // wait 1 sec for the sensor to stabilize 
  bno.setExtCrystalUse(true); //uses external crystal for more accurate orientation measurements
}

void loop() {
  // Read quaternion
  imu::Quaternion q = bno.getQuat(); // read the quaternion(w,x,y,z)

  // Print raw quaternion values
  Serial.print("w: "); Serial.print(q.w());
  Serial.print(" x: "); Serial.print(q.x());
  Serial.print(" y: "); Serial.print(q.y());
  Serial.print(" z: "); Serial.println(q.z());

  //(yaw= atan2(2(wz+xy),1-2(y^2+z^2))), atan2= calculate angle in radion 
  double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  yaw = yaw * 180.0 / M_PI;  // convert radians to degrees 180/pi

  double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  pitch = pitch * 180.0 / M_PI; // Convert to degrees

  //Serial.print("Pitch: "); Serial.println(pitch, 2);

  //Optional: keep yaw in 0-360 range
  if (yaw < 0) yaw += 360.0;

  Serial.print("Yaw: "); Serial.println(yaw);
  Serial.println("--------------------");

  delay(500);
}