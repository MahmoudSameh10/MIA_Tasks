#include <Wire.h>

const int MPU6050_ADDR = 0x68; // I2C address of the MPU-6050
const int PWR_MGMT_1 = 0x6B;   // Power management register
const int GYRO_ZOUT_H = 0x47;  // Gyro Z high byte register
const int GYRO_ZOUT_L = 0x48;  // Gyro Z low byte register
const float GYRO_SENSITIVITY = 131.0; // Sensitivity for +/- 250 deg/s

float yaw = 0.0;
unsigned long prevTime;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // Wake up the MPU6050 (exit sleep mode)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
  
  prevTime = millis();
}

void loop() {
  int16_t gyroZ = readGyroZ();
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
  prevTime = currentTime;
  
  float gyroZrate = gyroZ / GYRO_SENSITIVITY; // Convert to deg/s
  yaw += gyroZrate * dt; // Integrate to get yaw
  
  Serial.print("Yaw: ");
  Serial.println(yaw);
  
  delay(100); // Adjust delay as necessary
}

int16_t readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  
  int16_t gyroZ = Wire.read() << 8 | Wire.read(); // Combine high and low bytes
  return gyroZ;
}
