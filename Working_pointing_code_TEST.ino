#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <Servo.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Servos
Servo servoY;
Servo servoZ;
const int servoYPin = 10;
const int servoZPin = 9;

// Calibration
float gyroY_offset = -5.25f;
float gyroZ_offset = -0.28f;

float accelX_offset = -0.0075f;
float accelY_offset = 0.0f;
float accelZ_offset = 0.02f; 

// IMU ORIENTATION OFFSETS 
float IMU_OFFSET_Y = -90; 
float IMU_OFFSET_Z = 0; 

// Complementary filter
const float alpha = 0.98f;

// Centers and range
float ANGLE_CENTER_Y = 70.0f; 
float ANGLE_CENTER_Z = 95.0f; 
float RANGE_DEG = 20.0f;

// Computed limits
float ANGLE_MIN_Y, ANGLE_MAX_Y;
float ANGLE_MIN_Z, ANGLE_MAX_Z;

void setCenters() {
  ANGLE_MIN_Y = ANGLE_CENTER_Y - RANGE_DEG;
  ANGLE_MAX_Y = ANGLE_CENTER_Y + RANGE_DEG;
  ANGLE_MIN_Z = ANGLE_CENTER_Z - RANGE_DEG;
  ANGLE_MAX_Z = ANGLE_CENTER_Z + RANGE_DEG;
}

// integration state
unsigned long lastMicros;
float angleY = 0;
float angleZ = 180;

// IMU freeze detection
float lastGY = 0.0f, lastGZ = 0.0f;
unsigned long lastChangeTime = 0;
const unsigned long reinitTimeout = 1500; // ms

void setup() {
  Serial.begin(115200);
  Wire.begin();

  myIMU.begin();
  delay(500);

  servoY.attach(servoYPin);
  servoZ.attach(servoZPin);

  setCenters();

  // initialize servo to centers
  servoY.write(ANGLE_CENTER_Y);
  servoZ.write(ANGLE_CENTER_Z);

  lastMicros = micros();

  Serial.println("Ready");
  Serial.print("Y limits: "); Serial.print(ANGLE_MIN_Y); Serial.print(" - "); Serial.println(ANGLE_MAX_Y);
  Serial.print("Z limits: "); Serial.print(ANGLE_MIN_Z); Serial.print(" - "); Serial.println(ANGLE_MAX_Z);
}

void loop() {
  // timing
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastMicros = now;

  // raw reads
  float raw_gx = myIMU.readFloatGyroX();
  float raw_gy = myIMU.readFloatGyroY();
  float raw_gz = myIMU.readFloatGyroZ();

  float raw_ax = myIMU.readFloatAccelX();
  float raw_ay = myIMU.readFloatAccelY();
  float raw_az = myIMU.readFloatAccelZ();

  // freeze detection
  if (fabs(raw_gy - lastGY) < 0.0001f && fabs(raw_gz - lastGZ) < 0.0001f) {
    if (millis() - lastChangeTime > reinitTimeout) {
      Serial.println("IMU frozen, attempting reinit");
      myIMU.begin();
      delay(200);
      lastChangeTime = millis();
      return; // skip one loop to restart
    }
  } else {
    lastGY = raw_gy;
    lastGZ = raw_gz;
    lastChangeTime = millis();
  }

  // apply offsets
  float gy = raw_gy - gyroY_offset;
  float gz = raw_gz - gyroZ_offset;

  float ax = raw_ax - accelX_offset;
  float ay = raw_ay - accelY_offset;
  float az = raw_az - accelZ_offset;

  // angles
  float accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  float accAngleZ = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;

  // apply IMU mounting orientation offsets
  accAngleY += IMU_OFFSET_Y;
  accAngleZ += IMU_OFFSET_Z;


  // integrate gyro
  angleY += gy * dt;
  angleZ += gz * dt;

  // complementary filter
  angleY = alpha * angleY + (1.0f - alpha) * (ANGLE_CENTER_Y + accAngleY);
  angleZ = alpha * angleZ + (1.0f - alpha) * (ANGLE_CENTER_Z + accAngleZ);

  angleY = constrain(angleY, ANGLE_MIN_Y, ANGLE_MAX_Y);
  angleZ = constrain(angleZ, ANGLE_MIN_Z, ANGLE_MAX_Z);

  // compute servo outputs
  float servoY_cmd = angleY;
  float servoZ_cmd = angleZ;

  // Servo limit
  servoY_cmd = constrain(servoY_cmd, ANGLE_MIN_Y, ANGLE_MAX_Y);
  servoZ_cmd = constrain(servoZ_cmd, ANGLE_MIN_Z, ANGLE_MAX_Z);

  // write
  servoY.write((int)round(servoY_cmd));
  servoZ.write((int)round(servoZ_cmd));

  // debug
  Serial.print("srvY:"); Serial.print(servoY_cmd,2);
  Serial.print(" srvZ:"); Serial.print(servoZ_cmd,2);
  Serial.print(" | angY:"); Serial.print(angleY,2);
  Serial.print(" angZ:"); Serial.print(angleZ,2);
  Serial.print(" | GY:"); Serial.print(gy,2); Serial.print(" GZ:"); Serial.print(gz,2);
  Serial.print(" | AX:"); Serial.print(ax,3); Serial.print(" AY:"); Serial.print(ay,3); Serial.print(" AZ:"); Serial.println(az,3);

  // display used for tuning imu & gyro
  // Serial.print("| "); Serial.print(gy);
  // Serial.print("| "); Serial.print(gz);
  // Serial.print("| "); Serial.print(ax);
  // Serial.print("| "); Serial.print(ay);
  // Serial.print("| "); Serial.println(az);

  // // display used for centering servo
  // Serial.print("| "); Serial.print(angleY); Serial.print("| "); Serial.print(accAngleY);
  // Serial.print("| "); Serial.print(angleZ); Serial.print("| "); Serial.println(accAngleZ);

  delay(20); // ~50 Hz
}


