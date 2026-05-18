#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "DFRobot_BMI160.h"

// --- PIN DEFINITIONS ---
const int MOTOR1_PIN = 13; // Front-Left (CW)
const int MOTOR2_PIN = 12; // Front-Right (CCW)
const int MOTOR3_PIN = 14; // Rear-Right (CW)
const int MOTOR4_PIN = 27; // Rear-Left (CCW)

// --- SENSOR CONFIG ---
DFRobot_BMI160 bmi160;
const int8_t MY_BMI160_I2C_ADDR = 0x69;

// --- MOTOR OBJECTS & CONSTANTS ---
Servo motor1, motor2, motor3, motor4;
const int MIN_PULSE = 1000;
const int MAX_PULSE = 2000;

// --- FLIGHT CONTROL VARIABLES ---
int baseThrottle = 1000; // Controlled via Serial Monitor
float roll = 0, pitch = 0;
unsigned long lastTime;
unsigned long lastPrintTime = 0;

// --- PID TUNING PARAMETERS (Baseline Values) ---
// Adjust these slowly during tuning.
// Start with P, keep I and D at 0, then increase slowly.
float kP_roll = 1.2, kI_roll = 0.01, kD_roll = 0.4;
float kP_pitch = 1.2, kI_pitch = 0.01, kD_pitch = 0.4;

// --- PID INTERNAL STATES ---
float roll_error, roll_error_integral, roll_error_derivative, roll_last_error = 0;
float pitch_error, pitch_error_integral, pitch_error_derivative, pitch_last_error = 0;

// Target Setpoints (0 degrees = absolute flat hover)
float targetRoll = 0.0;
float targetPitch = 0.0;

// --- MOTOR WRITER FUNCTION ---
void writeMotors(int m1, int m2, int m3, int m4)
{
  motor1.writeMicroseconds(constrain(m1, MIN_PULSE, MAX_PULSE));
  motor2.writeMicroseconds(constrain(m2, MIN_PULSE, MAX_PULSE));
  motor3.writeMicroseconds(constrain(m3, MIN_PULSE, MAX_PULSE));
  motor4.writeMicroseconds(constrain(m4, MIN_PULSE, MAX_PULSE));
}

void stopMotors()
{
  baseThrottle = MIN_PULSE;
  writeMotors(MIN_PULSE, MIN_PULSE, MIN_PULSE, MIN_PULSE);
  Serial.println("-> Motors Stopped & Reset to 1000µs.");
}

void calibrateESCs()
{
  Serial.println("\n--- STARTING CALIBRATION ---");
  Serial.println("1. Unplug your LiPo battery NOW.");
  Serial.println("2. Sending MAX throttle (2000µs)...");
  writeMotors(MAX_PULSE, MAX_PULSE, MAX_PULSE, MAX_PULSE);
  Serial.println("3. Plug in your LiPo battery. Wait for the 2 short beeps.");
  Serial.println("4. Press ANY KEY in the Serial Monitor immediately after those beeps.");

  while (!Serial.available())
  {
  }
  Serial.read();

  Serial.println("5. Dropping throttle to MIN (1000µs)...");
  stopMotors();
  Serial.println("=== CALIBRATION COMPLETE ===");
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);
  delay(500);

  if (bmi160.I2cInit(MY_BMI160_I2C_ADDR) != 0)
  {
    Serial.println("BMI160 init failed!");
    while (1)
      ;
  }
  Serial.println("BMI160 OK");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);

  motor1.attach(MOTOR1_PIN, MIN_PULSE, MAX_PULSE);
  motor2.attach(MOTOR2_PIN, MIN_PULSE, MAX_PULSE);
  motor3.attach(MOTOR3_PIN, MIN_PULSE, MAX_PULSE);
  motor4.attach(MOTOR4_PIN, MIN_PULSE, MAX_PULSE);

  stopMotors();
  lastTime = micros();
}

void loop()
{
  // 1. SERIAL TERMINAL THROTTLE INPUT
  if (Serial.available() > 0)
  {
    char input = Serial.peek();
    if (input == 'C' || input == 'c')
    {
      Serial.read();
      calibrateESCs();
    }
    else
    {
      int inputThrottle = Serial.parseInt();
      if (inputThrottle >= MIN_PULSE && inputThrottle <= MAX_PULSE)
      {
        baseThrottle = inputThrottle;
        Serial.print("-> Base Throttle Set to: ");
        Serial.println(baseThrottle);

        // Reset integral states when throttle changes to prevent sudden power surges
        roll_error_integral = 0;
        pitch_error_integral = 0;
      }
    }
  }

  // 2. SENSOR READING & TIME TRACKING
  int16_t accel[3] = {0};
  int16_t gyro[3] = {0};

  if (bmi160.getAccelData(accel) == 0 && bmi160.getGyroData(gyro) == 0)
  {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // Convert raw data
    float gyroX = gyro[0] / 16.4;
    float gyroY = gyro[1] / 16.4;
    float accX = accel[0] / 16384.0;
    float accY = accel[1] / 16384.0;
    float accZ = accel[2] / 16384.0;

    // Complementary Filter
    float accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
    float accRoll = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    pitch = 0.98 * (pitch + gyroX * dt) + 0.02 * accPitch;
    roll = 0.98 * (roll + gyroY * dt) + 0.02 * accRoll;

    // 3. PID COMPUTATION LOOP
    // Roll Math
    roll_error = targetRoll - roll;
    roll_error_integral += roll_error * dt;
    roll_error_integral = constrain(roll_error_integral, -50, 50); // Windup guard
    roll_error_derivative = (roll_error - roll_last_error) / dt;
    float roll_output = (kP_roll * roll_error) + (kI_roll * roll_error_integral) + (kD_roll * roll_error_derivative);
    roll_last_error = roll_error;

    // Pitch Math
    pitch_error = targetPitch - pitch;
    pitch_error_integral += pitch_error * dt;
    pitch_error_integral = constrain(pitch_error_integral, -50, 50); // Windup guard
    pitch_error_derivative = (pitch_error - pitch_last_error) / dt;
    float pitch_output = (kP_pitch * pitch_error) + (kI_pitch * pitch_error_integral) + (kD_pitch * pitch_error_derivative);
    pitch_last_error = pitch_error;

    // 4. MOTOR MIXING MATRIX
    int motor1_speed = MIN_PULSE;
    int motor2_speed = MIN_PULSE;
    int motor3_speed = MIN_PULSE;
    int motor4_speed = MIN_PULSE;

    // Only apply corrections if the drone is actively getting throttle command
    if (baseThrottle > 1050)
    {
      motor1_speed = baseThrottle - pitch_output + roll_output; // Front-Left
      motor2_speed = baseThrottle - pitch_output - roll_output; // Front-Right
      motor3_speed = baseThrottle + pitch_output - roll_output; // Rear-Right
      motor4_speed = baseThrottle + pitch_output + roll_output; // Rear-Left
    }
    else
    {
      // Idle state safety clear
      roll_error_integral = 0;
      pitch_error_integral = 0;
    }

    // Write mixed values to hardware
    writeMotors(motor1_speed, motor2_speed, motor3_speed, motor4_speed);
  }

  // 5. SLOW DEBUG TELEMETRY (200ms)
  if (millis() - lastPrintTime > 200)
  {
    lastPrintTime = millis();
    Serial.print("Ang[R:");
    Serial.print(roll, 1);
    Serial.print(" P:");
    Serial.print(pitch, 1);
    Serial.print("] | BaseThrot: ");
    Serial.print(baseThrottle);
    Serial.print(" | Motors M1:");
    Serial.print(motor1.readMicroseconds());
    Serial.print(" M2:");
    Serial.print(motor2.readMicroseconds());
    Serial.print(" M3:");
    Serial.print(motor3.readMicroseconds());
    Serial.print(" M4:");
    Serial.println(motor4.readMicroseconds());
  }
}