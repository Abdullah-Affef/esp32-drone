#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "DFRobot_BMI160.h"

// --- PIN DEFINITIONS ---
const int MOTOR1_PIN = 13; // Front-Left
const int MOTOR2_PIN = 12; // Front-Right
const int MOTOR3_PIN = 14; // Rear-Right
const int MOTOR4_PIN = 27; // Rear-Left

// --- SENSOR CONFIG ---
DFRobot_BMI160 bmi160;
const int8_t MY_BMI160_I2C_ADDR = 0x69; // Renamed to avoid library conflicts

// --- MOTOR OBJECTS & CONSTANTS ---
Servo motor1, motor2, motor3, motor4;
const int MIN_PULSE = 1000;
const int MAX_PULSE = 2000;

// --- LOOP & FILTER VARIABLES ---
unsigned long lastTime;
float roll = 0, pitch = 0;
unsigned long lastPrintTime = 0;

// --- HELPER FUNCTIONS ---

void writeToAllMotors(int value)
{
  motor1.writeMicroseconds(value);
  motor2.writeMicroseconds(value);
  motor3.writeMicroseconds(value);
  motor4.writeMicroseconds(value);
}

void stopMotors()
{
  writeToAllMotors(MIN_PULSE);
  Serial.println("-> Motors Stopped (1000µs).");
}

void calibrateESCs()
{
  Serial.println("\n--- STARTING CALIBRATION ---");
  Serial.println("1. Unplug your LiPo battery NOW.");
  Serial.println("2. Sending MAX throttle (2000µs)...");
  writeToAllMotors(MAX_PULSE);
  Serial.println("3. Plug in your LiPo battery. Wait for the 2 short beeps.");
  Serial.println("4. Press ANY KEY in the Serial Monitor immediately after those beeps.");

  while (!Serial.available())
  {
  }
  Serial.read(); // Clear buffer

  Serial.println("5. Dropping throttle to MIN (1000µs)...");
  stopMotors();
  Serial.println("=== CALIBRATION COMPLETE ===");
}

// --- MAIN SETUP ---
void setup()
{
  Serial.begin(115200);

  // Initialize physical I2C pins
  Wire.begin(21, 22);
  delay(500);

  Serial.println("=== Full Drone Hardware Integration ===");

  // Use the verified working address
  if (bmi160.I2cInit(MY_BMI160_I2C_ADDR) != 0)
  {
    Serial.println("BMI160 init failed! Check connections.");
    while (1)
      ;
  }
  Serial.println("BMI160 OK");

  // Configure ESC PWM Timers safely for the ESP32
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

// --- MAIN LOOP ---
void loop()
{
  // 1. READ SERIAL MONITOR THROTTLE INPUT
  if (Serial.available() > 0)
  {
    char input = Serial.peek();
    if (input == 'C' || input == 'c')
    {
      Serial.read(); // Consume the 'c' character
      calibrateESCs();
    }
    else
    {
      int throttle = Serial.parseInt();
      if (throttle >= MIN_PULSE && throttle <= MAX_PULSE)
      {
        writeToAllMotors(throttle);
        Serial.print("-> Global Throttle Set to: ");
        Serial.println(throttle);
      }
    }
  }

  // 2. READ IMU DATA & CALCULATE INTERPRETED ORIENTATION
  int16_t accel[3] = {0};
  int16_t gyro[3] = {0};

  if (bmi160.getAccelData(accel) == 0 && bmi160.getGyroData(gyro) == 0)
  {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0; // Calculate time chunk since last reading
    lastTime = currentTime;

    // Convert raw Gyro data into Degrees per Second (°/s)
    float gyroX = gyro[0] / 16.4;
    float gyroY = gyro[1] / 16.4;

    // Convert raw Accelerometer data to G-forces
    float accX = accel[0] / 16384.0;
    float accY = accel[1] / 16384.0;
    float accZ = accel[2] / 16384.0;

    // Compute absolute tilt angles based on gravity vector
    float accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
    float accRoll = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

    // Complementary Filter: Blends fast Gyro tracking with stable Accelerometer gravity tracking
    pitch = 0.98 * (pitch + gyroX * dt) + 0.02 * accPitch;
    roll = 0.98 * (roll + gyroY * dt) + 0.02 * accRoll;
  }

  // 3. PRINT SLOWLY (Every 200ms) so the screen doesn't lag out
  if (millis() - lastPrintTime > 200)
  {
    lastPrintTime = millis();
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" | Pitch: ");
    Serial.println(pitch);
  }
}