/*!
 Highly modified from various test files all cobbled together to produce something that does at least work
 Can't say its pretty though.
 */

#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include "DFRobot_BMP280.h"
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>


typedef DFRobot_BMP280_IIC    BMP;
BMP   bmp(&Wire, BMP::eSdo_high);
#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure
DFRobot_QMC5883 compass;
FreeSixIMU sixDOF = FreeSixIMU();
float angles[3]; // yaw pitch roll
int accval[3]; //Raw acc
float gyroval[3]; // Raw gyro

int modepin = 2;
int Rpin = 3;
int Gpin = 5;
int Bpin = 6;

//Range values for sensor
//The min constrain here is based on the ambient magnetic field at my location to get rid of some noise
//It really needs proper compensation but this works for now
int SENSOR_MIN = 2000;
int SENSOR_MAX = 32767;

int PWM_MIN = 0;
int PWM_MAX = 255;
//int PWM_MAX = 168;

//Having trouble with the serial output being corrupted at high refresh rates
//But at the speed the serial data is fine, the RGB outputs arent smooth.
//The attempt here is to just run the serial data every Nth cycle
int throttle = 1;

void setup()
{
  pinMode(modepin, INPUT_PULLUP);
  analogWrite(Rpin, 0);
  analogWrite(Gpin, 0);
  analogWrite(Bpin, 0);
  Serial.begin(9600);
  //Magnetometer init
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }
  //Pressure sensor init
  bmp.reset();
  bmp.begin();
  //IMU init
  sixDOF.init();
  
  // delay(1000);
  
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / PI);
  //
  // Using socal declination of 11'37E
  float declinationAngle = (11 + (37.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  //Flash our LEDs to signal that we're about to begin
  rgbflash();
}
void rgbflash() {
  analogWrite(Rpin, 255);
  delay(100);
  analogWrite(Rpin, 0);
  delay(100);
  analogWrite(Gpin, 255);
  delay(100);
  analogWrite(Gpin, 0);
  delay(100);
  analogWrite(Bpin, 255);
  delay(100);
  analogWrite(Bpin, 0);
}

//Colors for each axis
void maglight1(Vector mag) {
  //Normalize our data to 0-255 for analog output
  int x = constrain(abs(mag.XAxis), SENSOR_MIN, SENSOR_MAX);
  int y = constrain(abs(mag.YAxis), SENSOR_MIN, SENSOR_MAX);
  int z = constrain(abs(mag.ZAxis), SENSOR_MIN, SENSOR_MAX);
  int xnorm = map(x, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  int ynorm = map(y, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  int znorm = map(z, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  //And now simply write the value out to our RGB leds
  //Serial.print("DATA: ");
  //Serial.print(xnorm);
  //Serial.print("   ");
  //Serial.print(ynorm);
  //Serial.print("   ");
  //Serial.println(znorm);
  analogWrite(Rpin, abs(xnorm));
  analogWrite(Gpin, abs(ynorm));
  analogWrite(Bpin, abs(znorm));
}

void polaritymode(Vector mag) {
  //This time we only care about the Z axis and we are looking for negative and positive values
  //North or south pole determines red or blue LED
  int outpin = Rpin;
  int otherpin = Bpin;
  if (mag.ZAxis < 0) { outpin = Bpin; otherpin = Rpin; }
  //And now the same as maglight1 but just for z data
  int z = constrain(abs(mag.ZAxis), SENSOR_MIN, SENSOR_MAX);
  int znorm = map(z, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  analogWrite(outpin, znorm);
  analogWrite(otherpin, 0);
}

struct sensordata {
  long magx;
  long magy;
  long magz;
  float magh;
  long accelx;
  long accely;
  long accelz;
  long gyrox;
  long gyroy;
  long gyroz;
  long yaw;
  long pitch;
  long roll;
  float envtemp;
  long envpress;
  float envalt;
  long rgbxnorm;
  long rgbynorm;
  long rgbznorm;
};
typedef struct sensordata _SD;


void sensormonitorfeed() {
  // Feed function for the SensorMonitor app
  // Simply sends our struct over serial to the receiving side.
  
  //Mag data
  Vector mag = compass.readRaw();
  int magx = constrain(abs(mag.XAxis), SENSOR_MIN, SENSOR_MAX);
  int magy = constrain(abs(mag.YAxis), SENSOR_MIN, SENSOR_MAX);
  int magz = constrain(abs(mag.ZAxis), SENSOR_MIN, SENSOR_MAX);
  int magxnorm = map(magx, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  int magynorm = map(magy, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  int magznorm = map(magz, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  compass.getHeadingDegrees();
  float h = mag.HeadingDegress;

  _SD sensordata;
  sensordata.magx = mag.XAxis;
  sensordata.magy = mag.YAxis;
  sensordata.magz = mag.ZAxis;
  sensordata.magh = h;
  sixDOF.gyro.readGyro(gyroval);
  sixDOF.acc.readAccel(accval);
  sixDOF.getAngles(angles);
  sensordata.accelx = accval[0];
  sensordata.accely = accval[1];
  sensordata.accelz = accval[2];
  sensordata.gyrox = gyroval[0];
  sensordata.gyroy = gyroval[1];
  sensordata.gyroz = gyroval[2];
  sensordata.yaw = angles[0];
  sensordata.pitch = angles[1];
  sensordata.roll = angles[2];
  sensordata.envtemp = bmp.getTemperature();
  sensordata.envpress = (long)bmp.getPressure();
  sensordata.envalt = bmp.calAltitude(SEA_LEVEL_PRESSURE, sensordata.envpress);
  //RGB data depends on whether we are in 3-axis or polarity mode
  if (digitalRead(modepin) == HIGH) { //3-axis light show
    sensordata.rgbxnorm = magxnorm;
    sensordata.rgbynorm = magynorm;
    sensordata.rgbznorm = magznorm;
  } else { //Polarity mode
    //In polarity mode we are only using the red and blue leds (x and z)
    sensordata.rgbxnorm = 0;
    sensordata.rgbynorm = 0;
    sensordata.rgbznorm = 0;
    //same as maglight1 but just for z data
    int pz = constrain(abs(mag.ZAxis), SENSOR_MIN, SENSOR_MAX);
    int pznorm = map(pz, SENSOR_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
    if (mag.ZAxis < 0) sensordata.rgbznorm = pznorm;
    else sensordata.rgbxnorm = pznorm;

  }
  //Help with corrupted data?
  Serial.flush();
  Serial.write((uint8_t *) &sensordata, (uint16_t) sizeof(sensordata));
  Serial.print("\r\n");
}

void loop()
{
  Vector mag = compass.readRaw();
  //Light show
  if (digitalRead(modepin) == HIGH) maglight1(mag);
  else polaritymode(mag);
  if (throttle == 3) {
    throttle = 0;
    sensormonitorfeed();
  } else throttle++;
  
  delay(20);
}
