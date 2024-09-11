class PID {
  public:
    // PID parameters
    float Kp, Ki, Kd;
    float prevError, integral;
    
    PID(float Kp, float Ki, float Kd) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
      prevError = 0;
      integral = 0;
    }

    // PID calculation
    float calculate(float setpoint, float measuredValue, float dt) {
      float error = setpoint - measuredValue;
      integral += error * dt;
      float derivative = (error - prevError) / dt;
      float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
      prevError = error;
      return output;
    }
};

// Exponential Smoothing Filter function
float smoothSpeed(float targetSpeed, float currentSpeed, float alpha) {
  return ((alpha * targetSpeed) + ((1 - alpha) * currentSpeed));
}

// Motor control parameters
const int motorPin = 9;  // PWM pin for motor control
float motorSpeed = 0;    // Current motor speed

// PID and filter parameters
PID motorPID(1.0, 0.5, 0.1);  // PID Kp, Ki, Kd
float targetSpeed = 100;      // Desired speed
float filteredSpeed = 0;      // Smoothed speed
float alpha = 0.1;            // Exponential smoothing factor
unsigned long lastTime;

void setup() {
  pinMode(motorPin, OUTPUT);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Convert time to seconds

  // Get motor's actual speed (use sensors like encoders here)
  float actualSpeed = analogRead(A0);  // Replace with actual speed measurement

  // PID calculation
  float pidOutput = motorPID.calculate(targetSpeed, actualSpeed, dt);

  // Apply soft start using exponential smoothing
  filteredSpeed = smoothSpeed(pidOutput, filteredSpeed, alpha);

  // Map filtered speed to PWM range (0-255) and control the motor
  int motorPWM = map(filteredSpeed, 0, 255, 0, 255);
  analogWrite(motorPin, motorPWM);

  lastTime = currentTime;

  delay(100);  // Control loop delay (adjust as needed)
}
