// TB6612FNG Motor Driver Pin Defines
#define PWMA 25
#define AIN1 14
#define AIN2 13
#define PWMB 4
#define BIN1 16
#define BIN2 17
#define STBY 5

// Sensor Pin Defines (5 channels)
#define S1_PIN 35
#define S2_PIN 34
#define S3_PIN 39
#define S4_PIN 36
#define S5_PIN 27

// Encoder Pins
#define ENCODER_LA 18  // Left Motor Encoder C1
#define ENCODER_LB 19  // Left Motor Encoder C2
#define ENCODER_RA 21  // Right Motor Encoder C1
#define ENCODER_RB 22  // Right Motor Encoder C2

// Push button and LED
#define LED_PIN 2      // Built-in LED

// PWM Settings for ESP32
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

int s[5];  
int total;
int sensor_position;
int threshold = 2000;  // Adjust based on your sensor readings
bool isBlackLine = true;  // true for black line on white surface
float avg;
int position[5] = {0, 1, 2, 3, 4};  // Position weights
float set_point = 2.0;  // Center position

// Encoder variables
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
volatile long last_left_count = 0;
volatile long last_right_count = 0;
unsigned long last_rpm_time = 0;
float left_rpm = 0;
float right_rpm = 0;
long left_pulses_per_interval = 0;
long right_pulses_per_interval = 0;

// Encoder specifications
#define PULSES_PER_REVOLUTION 20  
#define GEAR_RATIO 1              

// Motor calibration
float left_motor_calibration = 1.0;   
float right_motor_calibration = 1.0;  

void IRAM_ATTR leftEncoderISR() {
  encoder_left_count++;
}

void IRAM_ATTR rightEncoderISR() {
  encoder_right_count++;
}

void setup() {
  // Motor driver pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH);
  
  // PWM Setup (ESP32 Core 3.0+ syntax)
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
  
  // Sensor pins
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(S3_PIN, INPUT);
  pinMode(S4_PIN, INPUT);
  pinMode(S5_PIN, INPUT);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Encoder pins
  pinMode(ENCODER_LA, INPUT_PULLUP);
  pinMode(ENCODER_LB, INPUT_PULLUP);
  pinMode(ENCODER_RA, INPUT_PULLUP);
  pinMode(ENCODER_RB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_LA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RA), rightEncoderISR, RISING);
  
  Serial.begin(115200);
  Serial.println("ESP32 Line Following Robot - MAX SPEED");
  Serial.println("Robot will start in 3 seconds...");
  delay(3000);
  Serial.println("Robot STARTED!");
  
  encoder_left_count = 0;
  encoder_right_count = 0;
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_rpm_time >= 100) {
    calculate_rpm();
    last_rpm_time = current_time;
  }

  digitalWrite(LED_PIN, HIGH);
  PID_LINE_FOLLOW();
}

void calculate_rpm() {
  left_pulses_per_interval = encoder_left_count - last_left_count;
  right_pulses_per_interval = encoder_right_count - last_right_count;
  
  last_left_count = encoder_left_count;
  last_right_count = encoder_right_count;
  
  left_rpm = (float)left_pulses_per_interval / PULSES_PER_REVOLUTION * 600.0 / GEAR_RATIO;
  right_rpm = (float)right_pulses_per_interval / PULSES_PER_REVOLUTION * 600.0 / GEAR_RATIO;
  
  // Auto-calibration Logic
  static int calibration_samples = 0;
  static float calibration_ratio_sum = 0;
  
  // Only calibrate if moving reasonably fast and straight
  if (abs(left_pulses_per_interval) > 5 && abs(right_pulses_per_interval) > 5) {
    if (left_pulses_per_interval > right_pulses_per_interval) {
      float ratio = (float)right_pulses_per_interval / left_pulses_per_interval;
      calibration_ratio_sum += ratio;
      calibration_samples++;
      
      if (calibration_samples >= 20) { 
        left_motor_calibration = calibration_ratio_sum / calibration_samples;
        calibration_samples = 0;
        calibration_ratio_sum = 0;
      }
    } else if (right_pulses_per_interval > left_pulses_per_interval) {
      float ratio = (float)left_pulses_per_interval / right_pulses_per_interval;
      calibration_ratio_sum += ratio;
      calibration_samples++;
      
      if (calibration_samples >= 20) {
        right_motor_calibration = calibration_ratio_sum / calibration_samples;
        calibration_samples = 0;
        calibration_ratio_sum = 0;
      }
    }
  }
}

void Sensor_reading() {
  sensor_position = 0;
  total = 0;
  int sensor_pins[5] = {S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN};
  
  for (byte i = 0; i < 5; i++) {
    int val = analogRead(sensor_pins[i]);
    
    // Thresholding
    if (isBlackLine) {
      s[i] = (val < threshold) ? 1 : 0;
    } else {
      s[i] = (val > threshold) ? 1 : 0;
    }
    
    sensor_position += s[i] * position[i];
    total += s[i];
  }
  
  if (total > 0) {
    avg = (float)sensor_position / total;
  }
}

void PID_LINE_FOLLOW() {
  // --- TUNING FOR MAXIMUM SPEED ---
  // Base Speed: 240 (Near max 255)
  // KP: 150.0 (High Kp to force reverse on turns even at high forward speed)
  // KD: 250.0 (Very high Kd to dampen momentum and prevent oscillation)
  
  float kp = 150.0;        
  float kd = 250.0;        
  
  float PID_Value;
  float P, D;
  float error = 0;
  float previous_error = 0;
  
  // MAX SPEED SETTING
  int base_speed = 240;   
  int left_motor_speed;
  int right_motor_speed;
  
  unsigned long last_debug = 0;

  while (true) {
    Sensor_reading();
    
    // 1. Calculate Error
    error = set_point - avg; 
    
    P = error * kp;
    D = kd * (error - previous_error);
    PID_Value = P + D;
    previous_error = error;

    // 2. Calculate Speeds
    left_motor_speed = base_speed - PID_Value; 
    right_motor_speed = base_speed + PID_Value;

    // 3. Constrain
    // Allow negative speeds (reverse) for sharp turning
    left_motor_speed = constrain(left_motor_speed, -255, 255);
    right_motor_speed = constrain(right_motor_speed, -255, 255);

    // 4. Debugging
    if (millis() - last_debug > 300) {
       Serial.print("Avg: "); Serial.print(avg);
       Serial.print(" | Err: "); Serial.print(error);
       Serial.print(" | L_Spd: "); Serial.print(left_motor_speed);
       Serial.print(" | R_Spd: "); Serial.println(right_motor_speed);
       last_debug = millis();
    }

    // 5. Motor Control
    motor(left_motor_speed, right_motor_speed);
    
    delay(2); // Small stability delay
  }
}

void motor(int left_speed, int right_speed) {
  // Apply Auto-calibration
  left_speed = left_speed * left_motor_calibration;
  right_speed = right_speed * right_motor_calibration;
  
  // --- LEFT MOTOR (Motor A) ---
  // Logic swapped for your specific wiring so Positive = Forward
  if (left_speed > 0) {
    digitalWrite(AIN1, LOW);   
    digitalWrite(AIN2, HIGH);  
  } else if (left_speed < 0) {
    digitalWrite(AIN1, HIGH);  
    digitalWrite(AIN2, LOW);   
    left_speed = -left_speed;
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  // --- RIGHT MOTOR (Motor B) ---
  if (right_speed > 0) {
    digitalWrite(BIN1, HIGH); 
    digitalWrite(BIN2, LOW);  
  } else if (right_speed < 0) {
    digitalWrite(BIN1, LOW); 
    digitalWrite(BIN2, HIGH); 
    right_speed = -right_speed;
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }

  // Final Safety Constrain
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  ledcWrite(PWMA, left_speed);
  ledcWrite(PWMB, right_speed);
}