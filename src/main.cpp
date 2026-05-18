#include <Arduino.h>
#include <ESP32Servo.h>

// Define GPIO pins for the 4 ESCs
const int MOTOR1_PIN = 13; // Front-Left
const int MOTOR2_PIN = 12; // Front-Right
const int MOTOR3_PIN = 14; // Rear-Right
const int MOTOR4_PIN = 27; // Rear-Left

// Create Servo objects for each ESC
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Standard PWM pulse widths for generic ESCs (in microseconds)
const int MIN_PULSE = 1000; // 1ms = Full Throttle Off
const int MAX_PULSE = 2000; // 2ms = Full Throttle On

// --- HELPER FUNCTIONS (Placed first so PlatformIO knows they exist) ---

// Helper function to send the same throttle to all 4 motors
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
  Serial.println("Motors Stopped (1000µs sent).");
}

// Standard ESC Calibration Sequence
void calibrateESCs()
{
  Serial.println("\n--- STARTING CALIBRATION ---");
  Serial.println("1. STEP 1: Disconnect your LiPo battery NOW if it's connected.");
  Serial.println("2. Sending MAX throttle (2000µs)...");
  writeToAllMotors(MAX_PULSE);

  Serial.println("3. STEP 2: Plug in your LiPo battery.");
  Serial.println("4. You should hear a musical chime, then 2 short beeps.");
  Serial.println("5. IMMEDIATELY after those 2 beeps, type 'D' and press Enter to drop throttle to minimum.");

  while (!Serial.available())
  {
    // Wait for user to plug in battery and press a key
  }
  Serial.read(); // Clear buffer

  Serial.println("6. Dropping throttle to MIN (1000µs)...");
  stopMotors();

  Serial.println("7. You should hear several beeps indicating cell count, then a long beep.");
  Serial.println("=== CALIBRATION COMPLETE ===");
}

// --- STANDARD ARDUINO LOOPS ---

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println("=== ESP32 Quadcopter Motor Test & Calibration ===");
  Serial.println("Instructions:");
  Serial.println("1. To CALIBRATE: Unplug LiPo -> Send 'C' -> Plug in LiPo -> Wait for beeps -> Press any key.");
  Serial.println("2. To TEST MOTORS: Type a throttle value between 1000 and 1200 (e.g., 1050) to spin slowly.");

  // Allow allocation of all timers for ESP32 PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach motors with standard 50Hz frequency and pulse limits
  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);

  motor1.attach(MOTOR1_PIN, MIN_PULSE, MAX_PULSE);
  motor2.attach(MOTOR2_PIN, MIN_PULSE, MAX_PULSE);
  motor3.attach(MOTOR3_PIN, MIN_PULSE, MAX_PULSE);
  motor4.attach(MOTOR4_PIN, MIN_PULSE, MAX_PULSE);

  // Write minimum signal to initialize ESCs safely
  stopMotors();
}

void loop()
{
  if (Serial.available() > 0)
  {
    char input = Serial.peek();

    // Check if user wants to run calibration sequence
    if (input == 'C' || input == 'c')
    {
      Serial.read(); // Clear character from buffer
      calibrateESCs();
    }
    else
    {
      // Otherwise, parse the input as a direct PWM throttle value
      int throttle = Serial.parseInt();
      if (throttle >= MIN_PULSE && throttle <= MAX_PULSE)
      {
        Serial.print("Setting throttle to: ");
        Serial.println(throttle);
        writeToAllMotors(throttle);
      }
      else if (throttle != 0)
      {
        Serial.println("Invalid input! Enter a value between 1000 and 2000.");
      }
    }
  }
}