// Calibration Sketch to Find Accurate Thresholds
#define NUM_SENSORS 8
const int sensorPins[NUM_SENSORS] = {26, 27, 36, 39, 34, 35, 32, 33}; // enter pin number as per your sensor connection with esp32

// Arrays to store calibration data
int whiteValues[NUM_SENSORS] = {0}; // Store values on white
int blackValues[NUM_SENSORS] = {0}; // Store values on black
int thresholds[NUM_SENSORS] = {0};  // Calculated thresholds

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Sensor Calibration...");
  Serial.println("Place all sensors on a WHITE surface.");
  delay(5000); // Give time to position

  // Read white values
  for (int i = 0; i < NUM_SENSORS; i++) {
    whiteValues[i] = analogRead(sensorPins[i]);
  }
  Serial.println("White values recorded.");

  Serial.println("Now place all sensors on a BLACK line.");
  delay(5000); // Give time to position

  // Read black values
  for (int i = 0; i < NUM_SENSORS; i++) {
    blackValues[i] = analogRead(sensorPins[i]);
  }
  Serial.println("Black values recorded.");

  // Calculate and print thresholds
  Serial.println("Calculated Thresholds for your code:");
  Serial.print("int thresholds[NUM_SENSORS] = {");
  for (int i = 0; i < NUM_SENSORS; i++) {
    thresholds[i] = (whiteValues[i] + blackValues[i]) / 2;
    Serial.print(thresholds[i]);
    if (i < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("};");
}

void loop() {
  // Nothing to do here
}
