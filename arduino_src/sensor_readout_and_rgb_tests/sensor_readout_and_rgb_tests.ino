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
int SENSOR_MIN = -32767;
int AMBIENT_MIN = 2000;
int SENSOR_MAX = 32767;
bool ENABLE_AMBIENT_MIN_CAL = false; //Try and figure out a noise floor at startup. Overrides AMBIENT_MIN.
bool ENABLE_MAG_CALIBRATION = false;
Vector mag = {0,0,0};
//Magnetic calibration using this page as a guide:
// https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
int mag_bias[3] = {0,0,0};
float mag_scale[3] = {0.0f,0.0f,0.0f};
int mag_max[3] = {-32767,-32767,-32767};
int mag_min[3] = {32767,32767,32767};
int mag_temp[3] = {0,0,0};
int zmag_polarity_bias = 0;
int mag_cal_duration = 15; //In seconds

int PWM_MIN = 0;
int PWM_MAX = 255;
//int PWM_MAX = 168;

//Having trouble with the serial output being corrupted at high refresh rates
//But at the speed the serial data is fine, the RGB outputs arent smooth.
//The attempt here is to just run the serial data every Nth cycle
int throttle = 1;

//Calibrate our magnetometer for dur seconds
void magcal(int dur) {  
  //Turn on red LED to indicate we are calibrating
  analogWrite(Rpin, 255);
  //Calibrate for dur seconds
  for (int t=0; t<dur*10; t++) {
    //Figure out the min and max of our sensor on all three axis
    mag = compass.readRaw();
    mag_temp[0] = mag.XAxis;
    mag_temp[1] = mag.YAxis;
    mag_temp[2] = mag.ZAxis;
    for (int j=0; j<3; j++) {
       if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
       if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
    } 
    delay(100);
  }
  //Calculate the bias and scaling factors
  //To use these correction values we subtract the bias factor and multiply by the scale factor
  mag_bias[0] = (mag_max[0] + mag_min[0])/2;
  mag_bias[1] = (mag_max[1] + mag_min[1])/2;
  mag_bias[2] = (mag_max[2] + mag_min[2])/2;
  int mag_scale_tmp[3] = {
    (mag_max[0] - mag_min[0])/2,
    (mag_max[1] - mag_min[1])/2,
    (mag_max[2] - mag_min[2])/2
  };
  float avg_rad = (mag_scale_tmp[0] + mag_scale_tmp[1] + mag_scale_tmp[2]) / 3.0f;
  mag_scale[0] = avg_rad/((float)mag_scale_tmp[0]);
  mag_scale[1] = avg_rad/((float)mag_scale_tmp[1]);
  mag_scale[2] = avg_rad/((float)mag_scale_tmp[2]);
  //Try and figure out a dynamic noise floor
  //Currently we just ignore anything below 2000 but maybe we can fine tune that at runtime
  //Maybe try just using the largest value we find in our min/max cal?
  //int m = max(abs(mag_min[0]), abs(mag_min[1]));
  //m = max(m, abs(mag_min[2]));
  //m = max(m, mag_max[0]);
  //m = max(m, mag_max[1]);
  //m = max(m, mag_max[2]);
  //Or maybe add up all the mins and maxes and take the average?
  int m = (abs(mag_min[0]) + abs(mag_min[1]) + abs(mag_min[2]) + mag_max[0] + mag_max[1] + mag_max[2])/6;
  if (ENABLE_AMBIENT_MIN_CAL) AMBIENT_MIN = m;
  //Indicate we are checking zero
  analogWrite(Bpin, 255);
  delay(5000);
  mag = compass.readRaw();
  zmag_polarity_bias = mag.ZAxis;
  //Indicate we are done calibrating 
  analogWrite(Rpin, 0);
  analogWrite(Bpin, 0);
  delay(500);
  rgbflash(Rpin, 1);
  rgbflash(Gpin, 1);
  rgbflash(Bpin, 1);
}

Vector applymagcal(Vector mag) {
  if (!ENABLE_MAG_CALIBRATION) return mag;
  Vector out = {
    constrain(mag.XAxis, SENSOR_MIN, SENSOR_MAX),
    constrain(mag.YAxis, SENSOR_MIN, SENSOR_MAX),
    constrain(mag.ZAxis, SENSOR_MIN, SENSOR_MAX)
  };
  //Add our bias factor to recenter our data to 0
  out.XAxis = out.XAxis - mag_bias[0];
  out.YAxis = out.YAxis - mag_bias[1];
  out.ZAxis = out.ZAxis - mag_bias[2];
  //And multiply by the scaling factor to align the min/max range
  out.XAxis = mag.XAxis * mag_scale[0];
  out.YAxis = mag.YAxis * mag_scale[1];
  out.ZAxis = mag.ZAxis * mag_scale[2];
  
  return out;
}

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
  sixDOF.init(true);
  
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
  //Calibrate our magnetic sensor
  if (ENABLE_MAG_CALIBRATION) magcal(mag_cal_duration);
}

void rgbflash(int pin, int num) {
  for (int i=0; i<num; i++) {
    analogWrite(pin, 255);
    delay(100);
    analogWrite(pin, 0);
    delay(100);
  }
}
// Thought this would look nicer but it changes a LOT
//magnetic sensor data
struct MD {
  long magx;
  long magy;
  long magz;
  float magh;
  long rgbxnorm;
  long rgbynorm;
  long rgbznorm;
};
//IMU sensor data
struct ID {
  long accelx;
  long accely;
  long accelz;
  long gyrox;
  long gyroy;
  long gyroz;
  long yaw;
  long pitch;
  long roll;
};
//Environmental sensor data
struct ED {
  float envtemp;
  long envpress;
  float envalt;
};
struct _SD {
  struct MD magneticData;
  struct ID imuData;
  struct ED envData;
};

struct MD getMagneticData() {
  struct MD returndata;
  
  mag = compass.readRaw();
  //Apply our calibration factors to the magnetic data
  Vector mag2 = applymagcal(mag);
  int rawmagx = constrain(abs(mag.XAxis), AMBIENT_MIN, SENSOR_MAX);
  int rawmagy = constrain(abs(mag.YAxis), AMBIENT_MIN, SENSOR_MAX);
  int rawmagz = constrain(abs(mag.ZAxis), AMBIENT_MIN, SENSOR_MAX);
  int magxnorm = map(rawmagx, AMBIENT_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  int magynorm = map(rawmagy, AMBIENT_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  int magznorm = map(rawmagz, AMBIENT_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
  compass.getHeadingDegrees();
  float h = mag.HeadingDegress;

  returndata.magx = mag2.XAxis;
  returndata.magy = mag2.YAxis;
  returndata.magz = mag2.ZAxis;
  returndata.magh = h;
  //RGB data depends on whether we are in 3-axis or polarity mode
  if (digitalRead(modepin) == HIGH) { //3-axis light show
    returndata.rgbxnorm = magxnorm;
    returndata.rgbynorm = magynorm;
    returndata.rgbznorm = magznorm;
  } else { //Polarity mode
    //In polarity mode we are only using the red and blue leds (x and z)
    returndata.rgbxnorm = 0;
    returndata.rgbynorm = 0;
    returndata.rgbznorm = 0;
    //same as maglight1 but just for z data
    int adjusted_z = mag2.ZAxis - zmag_polarity_bias;
    int pz = constrain(abs(adjusted_z), AMBIENT_MIN, SENSOR_MAX);
    int pznorm = map(pz, AMBIENT_MIN, SENSOR_MAX, PWM_MIN, PWM_MAX);
    if (adjusted_z < 0) returndata.rgbznorm = pznorm;
    else returndata.rgbxnorm = pznorm;
  }

  return returndata;
};

struct ID getIMUData() {
  struct ID returndata;
  
  sixDOF.gyro.readGyro(gyroval);
  sixDOF.acc.readAccel(accval);
  //Sensor drift, especially for the yaw axis, is really bad for some reason with getAngles()
  sixDOF.getAngles(angles);
  //sixDOF.getYawPitchRoll(angles);
  returndata.accelx = accval[0];
  returndata.accely = accval[1];
  returndata.accelz = accval[2];
  returndata.gyrox = gyroval[0];
  returndata.gyroy = gyroval[1];
  returndata.gyroz = gyroval[2];
  returndata.yaw = angles[0];
  returndata.pitch = angles[1];
  returndata.roll = angles[2];

  return returndata;
}

struct ED getEnvData() {
  struct ED returndata;
  
  returndata.envtemp = bmp.getTemperature();
  returndata.envpress = (long)bmp.getPressure();
  returndata.envalt = bmp.calAltitude(SEA_LEVEL_PRESSURE, returndata.envpress);

  return returndata;
} 

void sendOverSerial(struct _SD sensordata) {
  // Feed function for the SensorMonitor app
  // Simply sends our nested structs over serial to the receiving side.
  Serial.flush();
  Serial.write((uint8_t *) &sensordata, (uint16_t) sizeof(sensordata));
  Serial.print("ZENDZ\r\n"); // Think this might fix the issues with connection dropouts
}


void loop()
{
  //Get our sensor data and set up the structs
  //Most of the loop only needs magnetic data for the RGB output
  struct MD magdata = getMagneticData();  
  //Light show  
  analogWrite(Rpin, magdata.rgbxnorm);
  analogWrite(Gpin, magdata.rgbynorm);
  analogWrite(Bpin, magdata.rgbznorm);
  //Send data over serial 
  if (throttle == 3) {
    throttle = 0;
    
    struct ID imudata = getIMUData();
    struct ED envdata = getEnvData();
    struct _SD sensordata;
    sensordata.magneticData = magdata;
    sensordata.imuData = imudata;
    sensordata.envData = envdata;
    sendOverSerial(sensordata);
  } else throttle++;
  
  delay(20);
}
