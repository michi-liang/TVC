#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFunLSM6DS3.h>
#include <Servo.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);
File logFile;
bool sdAvailable = false;

// Servos
Servo servoY;   
Servo servoZ;   
Servo servoParachute;

const int servoYPin = 10;
const int servoZPin = 9;
const int parachutePin = 8;

// IMU Calibration
float gyroY_offset = -5.25f;
float gyroZ_offset = -0.28f;
float accelX_offset = -0.0075f;
float accelY_offset = 0.0f;
float accelZ_offset = 0.02f;

// IMU orientation offsets
float IMU_OFFSET_Y = -90;
float IMU_OFFSET_Z = 0;

const float alpha = 0.98f;

// Servo Calibration
float ANGLE_CENTER_Y = 70.0f;
float ANGLE_CENTER_Z = 95.0f;
float RANGE_DEG = 20.0f;
float ANGLE_MIN_Y, ANGLE_MAX_Y;
float ANGLE_MIN_Z, ANGLE_MAX_Z;

// PID Constants
float KP_ANGLE = 0.8f;
float KD_RATE  = 0.15f;

// Servo to motor angle conversion (Assuming linear)
const float SERVO_TO_MOTOR = 4.0f / 20.0f;
const float MOTOR_TO_SERVO = 1.0f / SERVO_TO_MOTOR;

// State Machine
unsigned long lastMicros;
float estAngleY = 0;
float estAngleZ = 0;

// IMU Safety Constants
float lastGY = 0, lastGZ = 0;
unsigned long lastIMUChange = 0;
const unsigned long IMU_TIMEOUT = 500; // ms

// Flight state machine
enum FlightState { PRELAUNCH, BOOST, APOGEE, RECOVERY };
FlightState state = PRELAUNCH;

unsigned long boostStartTime = 0;
const unsigned long boostMaxTime = 2200;

// Launch stuff
bool launched = false;
unsigned long launchTime = 0;

// Acceleration needed to trigger launch
const float LAUNCH_ACC_THRESHOLD = 1.50f;
float estXVel = 0;

float posX = 0, posY = 0, posZ = 0;

// Range for servos
void setCenters() {
  ANGLE_MIN_Y = ANGLE_CENTER_Y - RANGE_DEG;
  ANGLE_MAX_Y = ANGLE_CENTER_Y + RANGE_DEG;
  ANGLE_MIN_Z = ANGLE_CENTER_Z - RANGE_DEG;
  ANGLE_MAX_Z = ANGLE_CENTER_Z + RANGE_DEG;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  myIMU.begin();
  delay(500);

  servoY.attach(servoYPin);
  servoZ.attach(servoZPin);
  servoParachute.attach(parachutePin);

  setCenters();

  servoY.write(ANGLE_CENTER_Y);
  servoZ.write(ANGLE_CENTER_Z);
  servoParachute.write(55);

  lastMicros = micros();

  // SD init 
  if (SD.begin(BUILTIN_SDCARD)) {
    logFile = SD.open("flight.csv", FILE_WRITE);
    if (logFile) {
      sdAvailable = true;
      logFile.println("time_s,state,posX,posY,posZ,angleY,angleZ,imuTemp");
      logFile.flush();
    }
  }

  Serial.println("time_s,state,posX,posY,posZ,angleY,angleZ,imuTemp");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // IMU Read
  float gy = myIMU.readFloatGyroY() - gyroY_offset;
  float gz = myIMU.readFloatGyroZ() - gyroZ_offset;
  float ax = myIMU.readFloatAccelX() - accelX_offset;
  float ay = myIMU.readFloatAccelY() - accelY_offset;
  float az = myIMU.readFloatAccelZ() - accelZ_offset;

  // IMU Safety
  if (fabs(gy - lastGY) < 1e-4 && fabs(gz - lastGZ) < 1e-4) {
    if (millis() - lastIMUChange > IMU_TIMEOUT) {
      servoY.write(ANGLE_CENTER_Y);
      servoZ.write(ANGLE_CENTER_Z);
      myIMU.begin();      // attempt recovery
      lastIMUChange = millis();
      return;
    }
  } else {
    lastGY = gy;
    lastGZ = gz;
    lastIMUChange = millis();
  }

  // Angle
  float accAngleY = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG + IMU_OFFSET_Y;
  float accAngleZ = atan2( ay, sqrt(ax*ax + az*az)) * RAD_TO_DEG + IMU_OFFSET_Z;

  estAngleY += gy * dt;
  estAngleZ += gz * dt;
  estAngleY = alpha * estAngleY + (1 - alpha) * accAngleY;
  estAngleZ = alpha * estAngleZ + (1 - alpha) * accAngleZ;

  // State Machine
  if (state == PRELAUNCH && az > LAUNCH_ACC_THRESHOLD) {
    state = BOOST;
    boostStartTime = millis();
    launchTime = millis();
    launched = true;
  }

  if (state == BOOST && millis() - boostStartTime >= boostMaxTime) {
    state = APOGEE;
  }

  //TVC
  if (state == BOOST) {
    float errorY = -estAngleY;
    float errorZ = -estAngleZ;

    float motorCmdY = KP_ANGLE * errorY - KD_RATE * gy;
    float motorCmdZ = KP_ANGLE * errorZ - KD_RATE * gz;

    servoY.write(constrain(
      ANGLE_CENTER_Y + motorCmdY * MOTOR_TO_SERVO,
      ANGLE_MIN_Y, ANGLE_MAX_Y));

    servoZ.write(constrain(
      ANGLE_CENTER_Z + motorCmdZ * MOTOR_TO_SERVO,
      ANGLE_MIN_Z, ANGLE_MAX_Z));
  } else {
    servoY.write(ANGLE_CENTER_Y);
    servoZ.write(ANGLE_CENTER_Z);
  }

  // State Machine
  switch (state) {
    case PRELAUNCH:
      servoParachute.write(55);
      estXVel = 0;
      posX = posY = posZ = 0;
      break;

    case BOOST:
      servoParachute.write(55);
      estXVel += ax * dt;
      break;

    case APOGEE:
      servoParachute.write(83);
      estXVel += ax * dt;
      break;

    case RECOVERY:
      servoParachute.write(83);
      estXVel = 0;
      break;
  }

  float dX = estXVel * dt;
  posX += dX;
  posY += dX * sin(estAngleZ * DEG_TO_RAD);
  posZ += dX * sin(estAngleY * DEG_TO_RAD);

  // Output
  Serial.print(state); Serial.print(",");
  Serial.print(posX,2); Serial.print(",");
  Serial.print(posY,2); Serial.print(",");
  Serial.print(posZ,2); Serial.print(",");
  Serial.print(estAngleY,2); Serial.print(",");
  Serial.print(estAngleZ,2); Serial.print(",");
  Serial.println(myIMU.readTempC(),2);

// SD 
  if (sdAvailable) {
    float t = (millis() - launchTime) * 0.001f;
    logFile.print(t,3); logFile.print(",");
    logFile.print(state); logFile.print(",");
    logFile.print(posX,2); logFile.print(",");
    logFile.print(posY,2); logFile.print(",");
    logFile.print(posZ,2); logFile.print(",");
    logFile.print(estAngleY,2); logFile.print(",");
    logFile.print(estAngleZ,2); logFile.print(",");
    logFile.println(myIMU.readTempC(),2);
    logFile.flush();
  }

  delay(20);
}


