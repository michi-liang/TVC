#include <Wire.h>
#include <SparkFunLSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

unsigned long sampleCount = 0;

float ax_avg = 0, ay_avg = 0, az_avg = 0;
float gx_avg = 0, gy_avg = 0, gz_avg = 0;

// Print timing
const unsigned long PRINT_INTERVAL = 500; // ms
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();


  Serial.println("IMU Running Average");
}

void loop() {
  // Read IMU
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  sampleCount++;

  // Update running averages
  ax_avg += (ax - ax_avg) / sampleCount;
  ay_avg += (ay - ay_avg) / sampleCount;
  az_avg += (az - az_avg) / sampleCount;

  gx_avg += (gx - gx_avg) / sampleCount;
  gy_avg += (gy - gy_avg) / sampleCount;
  gz_avg += (gz - gz_avg) / sampleCount;

  // Periodic print
  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;

    Serial.print("Samples: ");
    Serial.print(sampleCount);

    Serial.print(" | Accel avg (g): ");
    Serial.print(ax_avg, 4); Serial.print(", ");
    Serial.print(ay_avg, 4); Serial.print(", ");
    Serial.print(az_avg, 4);

    Serial.print(" | Gyro avg (dps): ");
    Serial.print(gx_avg, 4); Serial.print(", ");
    Serial.print(gy_avg, 4); Serial.print(", ");
    Serial.println(gz_avg, 4);
  }
}
