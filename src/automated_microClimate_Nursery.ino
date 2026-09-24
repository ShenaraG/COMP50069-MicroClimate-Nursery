#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#include "soc/soc.h"
#include "soc/gpio_reg.h"

#include <math.h>
#include <ctype.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

const int DHT_PIN = 15;
const int LDR_PIN = 34;

const int LED1_PIN = 25;
const int LED2_PIN = 26;
const int LED3_PIN = 27;

const int SERVO_PIN = 18;
const int BUTTON_PIN = 19;

const int OLED_SDA = 21;
const int OLED_SCL = 22;

// ============================================================
// DHT22
// ============================================================

#define DHT_TYPE DHT22

DHT dhtSensor(DHT_PIN, DHT_TYPE);

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);

bool oledReady = false;

// ============================================================
// SERVO
// ============================================================

Servo ventServo;

// ============================================================
// SYSTEM MODES
// ============================================================

enum SystemMode {
  AUTO_MODE,
  MANUAL_MODE,
  SAFETY_MODE
};

enum FaultType {
  NO_FAULT,
  DHT_FAULT,
  LDR_FAULT
};

SystemMode currentMode = AUTO_MODE;
FaultType currentFault = NO_FAULT;

// ============================================================
// SENSOR VALUES
// ============================================================

float temperature = NAN;
float humidity = NAN;

int lightLevel = 0;

bool sensorDataReady = false;

// ============================================================
// CONFIGURATION VALUES
// ============================================================

// Temperature ventilation hysteresis
float tempOnThreshold = 26.5;
float tempOffThreshold = 25.5;

// Critical temperature hysteresis
float criticalTempOn = 35.0;
float criticalTempOff = 34.0;

// Low humidity threshold
float lowHumidityThreshold = 35.0;

// LDR hysteresis
int lightDarkOnThreshold = 2200;
int lightBrightOffThreshold = 1800;

// Servo positions
const int VENT_CLOSED = 0;
const int VENT_FULL_OPEN = 90;
const int VENT_SAFETY = 30;
const int LOW_HUMIDITY_VENT_CAP = 45;

// ============================================================
// CONTROL STATES
// ============================================================

bool growLightsOn = false;

bool ventilationActive = false;
bool criticalHeatActive = false;

int currentVentAngle = 0;

// ============================================================
// SAFETY RECOVERY
// ============================================================

const int REQUIRED_VALID_READINGS = 3;

int validRecoveryReadings = 0;

// ============================================================
// TIMING
// ============================================================

const unsigned long SENSOR_INTERVAL = 2500;
const unsigned long STATUS_INTERVAL = 5000;
const unsigned long OLED_INTERVAL = 500;
const unsigned long BUTTON_DEBOUNCE = 250;

unsigned long lastSensorRead = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastButtonHandled = 0;

// ============================================================
// BUTTON INTERRUPT
// ============================================================

volatile bool buttonPressedFlag = false;

// ============================================================
// UART
// ============================================================

String serialBuffer = "";

// ============================================================
// REGISTER LEVEL LED MASK
// ============================================================

const uint32_t LED_MASK =
  (1UL << LED1_PIN) |
  (1UL << LED2_PIN) |
  (1UL << LED3_PIN);

// ============================================================
// BUTTON INTERRUPT
// ============================================================

/**
 * @brief Records a manual override button press.
 *
 * The interrupt only sets a flag. Debouncing and mode
 * switching are handled later in the main loop.
 *
 * @return Nothing.
 */
void IRAM_ATTR buttonISR() {
  buttonPressedFlag = true;
}

// ============================================================
// LED CONTROL
// ============================================================

/**
 * @brief Controls all three supplementary lighting LEDs.
 *
 * ESP32 GPIO registers are used for the digital output.
 *
 * @param state true turns LEDs ON, false turns LEDs OFF.
 * @return Nothing.
 */
void setGrowLights(bool state) {

  if (state) {
    REG_WRITE(GPIO_OUT_W1TS_REG, LED_MASK);
  }
  else {
    REG_WRITE(GPIO_OUT_W1TC_REG, LED_MASK);
  }

  growLightsOn = state;
}

// ============================================================
// SERVO CONTROL
// ============================================================

/**
 * @brief Moves the ventilation servo.
 *
 * @param angle Required angle from 0 to 90 degrees.
 * @return Nothing.
 */
void setVentAngle(int angle) {

  angle = constrain(angle, 0, 90);

  currentVentAngle = angle;

  ventServo.write(angle);
}

// ============================================================
// SENSOR VALIDATION
// ============================================================

/**
 * @brief Validates DHT22 readings.
 *
 * @param temp Temperature in degrees Celsius.
 * @param hum Relative humidity percentage.
 * @return true if both readings are valid, otherwise false.
 */
bool validateDHT(float temp, float hum) {

  if (isnan(temp) || isnan(hum)) {
    return false;
  }

  if (temp < -40.0 || temp > 80.0) {
    return false;
  }

  if (hum < 0.0 || hum > 100.0) {
    return false;
  }

  return true;
}

/**
 * @brief Checks whether the LDR value is inside the ESP32 ADC range.
 *
 * @param value ADC value.
 * @return true if the value is between 0 and 4095.
 */
bool validateLDR(int value) {

  return value >= 0 && value <= 4095;
}

// ============================================================
// TEMPERATURE HYSTERESIS
// ============================================================

/**
 * @brief Updates normal ventilation and critical heat states.
 *
 * @param temp Current temperature.
 * @return Nothing.
 */
void updateTemperatureStates(float temp) {

  // ----------------------------------------------------------
  // Critical heat hysteresis
  // ----------------------------------------------------------

  if (!criticalHeatActive &&
      temp >= criticalTempOn) {

    criticalHeatActive = true;
  }

  else if (criticalHeatActive &&
           temp <= criticalTempOff) {

    criticalHeatActive = false;
  }

  // ----------------------------------------------------------
  // Normal ventilation hysteresis
  // ----------------------------------------------------------

  if (!ventilationActive &&
      temp >= tempOnThreshold) {

    ventilationActive = true;
  }

  else if (ventilationActive &&
           temp <= tempOffThreshold) {

    ventilationActive = false;
  }
}

// ============================================================
// PROPORTIONAL SERVO CALCULATION
// ============================================================

/**
 * @brief Calculates vent angle according to temperature.
 *
 * The vent opens progressively as temperature rises.
 *
 * @param temp Current temperature.
 * @return Vent angle from 0 to 90 degrees.
 */
int calculateVentAngle(float temp) {

  if (criticalHeatActive) {
    return VENT_FULL_OPEN;
  }

  if (!ventilationActive) {
    return VENT_CLOSED;
  }

  float temperatureRange =
    criticalTempOn - tempOnThreshold;

  if (temperatureRange <= 0.0) {
    return VENT_CLOSED;
  }

  float ratio =
    (temp - tempOnThreshold) /
    temperatureRange;

  ratio = constrain(
    ratio,
    0.0f,
    1.0f
  );

  int angle =
    round(ratio * VENT_FULL_OPEN);

  /*
     When ventilation is still active because of hysteresis,
     keep a small physical opening instead of instantly
     returning to 0 degrees.
  */

  const int MIN_ACTIVE_VENT = 8;

  if (angle < MIN_ACTIVE_VENT) {
    angle = MIN_ACTIVE_VENT;
  }

  return angle;
}

// ============================================================
// SAFETY MODE
// ============================================================

/**
 * @brief Activates the predefined safe physical posture.
 *
 * LEDs are OFF and vent is placed at 30 degrees.
 *
 * @param fault Sensor fault responsible for Safety Mode.
 * @return Nothing.
 */
void enterSafetyMode(FaultType fault) {

  if (currentMode != SAFETY_MODE ||
      currentFault != fault) {

    validRecoveryReadings = 0;
  }

  currentMode = SAFETY_MODE;
  currentFault = fault;

  setGrowLights(false);

  setVentAngle(VENT_SAFETY);
}

// ============================================================
// MANUAL OVERRIDE
// ============================================================

/**
 * @brief Applies the manual override actuator positions.
 *
 * Vent is fully open and LEDs are switched OFF.
 *
 * @return Nothing.
 */
void applyManualMode() {

  setGrowLights(false);

  setVentAngle(VENT_FULL_OPEN);
}

// ============================================================
// AUTOMATIC CONTROL
// ============================================================

/**
 * @brief Applies the autonomous nursery control hierarchy.
 *
 * Priority:
 * 1. Critical temperature
 * 2. Temperature / humidity conflict
 * 3. Normal light control
 *
 * @return Nothing.
 */
void applyAutomaticControl() {

  if (!sensorDataReady) {
    return;
  }

  updateTemperatureStates(temperature);

  // ==========================================================
  // PRIORITY 1: CRITICAL TEMPERATURE
  // ==========================================================

  if (criticalHeatActive) {

    setGrowLights(false);

    setVentAngle(VENT_FULL_OPEN);

    return;
  }

  // ==========================================================
  // PRIORITY 2: TEMPERATURE / HUMIDITY CONTROL
  // ==========================================================

  if (ventilationActive) {

    int requestedAngle =
      calculateVentAngle(temperature);

    /*
       CONFLICT:

       High temperature requests increased ventilation.

       Very low humidity requests moisture conservation.

       Therefore, when humidity is below the configurable
       threshold, normal ventilation is capped at 45 degrees.

       Critical heat has already been checked above and
       therefore overrides this humidity restriction.
    */

    if (humidity < lowHumidityThreshold &&
        requestedAngle > LOW_HUMIDITY_VENT_CAP) {

      requestedAngle =
        LOW_HUMIDITY_VENT_CAP;
    }

    setVentAngle(requestedAngle);
  }

  else {

    setVentAngle(VENT_CLOSED);
  }

  // ==========================================================
  // PRIORITY 3: NORMAL LIGHT CONTROL
  // ==========================================================

  /*
     Temperature ventilation and normal lighting are independent.
     Only CRITICAL HEAT above forces the LEDs OFF.

     Higher ADC = darker
     Lower ADC = brighter

     Hysteresis:

     > 2200 = LEDs ON
     < 1800 = LEDs OFF
     1800-2200 = retain previous state
  */

  if (lightLevel > lightDarkOnThreshold) {

    setGrowLights(true);
  }

  else if (lightLevel < lightBrightOffThreshold) {

    setGrowLights(false);
  }
}

// ============================================================
// SAFETY RECOVERY
// ============================================================

/**
 * @brief Processes recovery from Safety Mode.
 *
 * Three consecutive valid sensor readings are required
 * before the system returns to AUTO mode.
 *
 * @return Nothing.
 */
void processSafetyRecovery() {

  if (currentMode != SAFETY_MODE) {
    return;
  }

  validRecoveryReadings++;

  Serial.print("Safety recovery: ");
  Serial.print(validRecoveryReadings);
  Serial.print("/");
  Serial.println(REQUIRED_VALID_READINGS);

  if (validRecoveryReadings >=
      REQUIRED_VALID_READINGS) {

    Serial.println(
      "Sensor recovered. Returning to AUTO mode."
    );

    currentFault = NO_FAULT;

    currentMode = AUTO_MODE;

    validRecoveryReadings = 0;

    applyAutomaticControl();
  }
}

// ============================================================
// SENSOR READING
// ============================================================

/**
 * @brief Reads DHT22 and LDR sensors.
 *
 * Sensor values are validated before control logic is executed.
 * Invalid readings immediately activate Safety Mode.
 *
 * @return Nothing.
 */
void readSensors() {

  // ----------------------------------------------------------
  // Read LDR
  // ----------------------------------------------------------

  int newLightLevel =
    analogRead(LDR_PIN);

  // ----------------------------------------------------------
  // Read DHT22
  // ----------------------------------------------------------

  float newHumidity =
    dhtSensor.readHumidity();

  float newTemperature =
    dhtSensor.readTemperature();

  // ----------------------------------------------------------
  // Validate readings
  // ----------------------------------------------------------

  bool dhtValid =
    validateDHT(
      newTemperature,
      newHumidity
    );

  bool ldrValid =
    validateLDR(
      newLightLevel
    );

  // Store latest readings
  temperature = newTemperature;
  humidity = newHumidity;
  lightLevel = newLightLevel;

  // ==========================================================
  // DHT22 FAILURE
  // ==========================================================

  if (!dhtValid) {

    sensorDataReady = false;

    /*
       IMPORTANT:
       Any invalid reading breaks the sequence of
       consecutive valid recovery readings.
    */

    validRecoveryReadings = 0;

    Serial.println(
      "SAFETY: DHT22 SENSOR FAILURE"
    );

    enterSafetyMode(DHT_FAULT);

    return;
  }

  // ==========================================================
  // LDR FAILURE
  // ==========================================================

  if (!ldrValid) {

    sensorDataReady = false;

    /*
       Any invalid reading breaks the recovery sequence.
    */

    validRecoveryReadings = 0;

    Serial.println(
      "SAFETY: LDR SENSOR FAILURE"
    );

    enterSafetyMode(LDR_FAULT);

    return;
  }

  // Both readings are valid
  sensorDataReady = true;

  // ==========================================================
  // SAFETY RECOVERY
  // ==========================================================

  if (currentMode == SAFETY_MODE) {

    processSafetyRecovery();

    return;
  }

  // ==========================================================
  // NORMAL MODE OPERATION
  // ==========================================================

  if (currentMode == AUTO_MODE) {

    applyAutomaticControl();
  }

  else if (currentMode == MANUAL_MODE) {

    applyManualMode();
  }
}

// ============================================================
// BUTTON HANDLING
// ============================================================

/**
 * @brief Processes the manual override button.
 *
 * Button toggles AUTO and MANUAL modes.
 * Safety Mode cannot be overridden by the button.
 *
 * @return Nothing.
 */
void handleButton() {

  if (!buttonPressedFlag) {
    return;
  }

  noInterrupts();

  buttonPressedFlag = false;

  interrupts();

  unsigned long now =
    millis();

  // ----------------------------------------------------------
  // Debounce
  // ----------------------------------------------------------

  if (now - lastButtonHandled <
      BUTTON_DEBOUNCE) {

    return;
  }

  lastButtonHandled = now;

  // ----------------------------------------------------------
  // Safety has highest priority
  // ----------------------------------------------------------

  if (currentMode == SAFETY_MODE) {

    Serial.println(
      "Button ignored: SAFETY MODE has priority."
    );

    return;
  }

  // ----------------------------------------------------------
  // AUTO -> MANUAL
  // ----------------------------------------------------------

  if (currentMode == AUTO_MODE) {

    currentMode = MANUAL_MODE;

    Serial.println(
      "MANUAL OVERRIDE ACTIVATED"
    );

    applyManualMode();
  }

  // ----------------------------------------------------------
  // MANUAL -> AUTO
  // ----------------------------------------------------------

  else if (currentMode == MANUAL_MODE) {

    currentMode = AUTO_MODE;

    Serial.println(
      "Returned to AUTO MODE"
    );

    if (sensorDataReady) {

      applyAutomaticControl();
    }
  }
}

// ============================================================
// MODE NAME
// ============================================================

/**
 * @brief Returns the readable system mode name.
 *
 * @param mode System mode.
 * @return Text representing the mode.
 */
const char* modeName(SystemMode mode) {

  switch (mode) {

    case AUTO_MODE:
      return "AUTO";

    case MANUAL_MODE:
      return "MANUAL";

    case SAFETY_MODE:
      return "SAFETY";

    default:
      return "UNKNOWN";
  }
}

// ============================================================
// FAULT NAME
// ============================================================

/**
 * @brief Returns the readable sensor fault name.
 *
 * @param fault Sensor fault.
 * @return Text representing the fault.
 */
const char* faultName(FaultType fault) {

  switch (fault) {

    case DHT_FAULT:
      return "DHT22";

    case LDR_FAULT:
      return "LDR";

    case NO_FAULT:
    default:
      return "NONE";
  }
}

// ============================================================
// OLED DISPLAY
// ============================================================

/**
 * @brief Updates the OLED with current system information.
 *
 * @return Nothing.
 */
void updateOLED() {

  if (!oledReady) {
    return;
  }

  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setCursor(0, 0);

  // ==========================================================
  // SAFETY SCREEN
  // ==========================================================

  if (currentMode == SAFETY_MODE) {

    display.println("MODE: SAFETY");

    display.print("FAULT: ");
    display.println(
      faultName(currentFault)
    );

    display.println(
      "Intervention needed"
    );

    display.println("LEDs: OFF");

    display.print("Vent: ");
    display.print(currentVentAngle);
    display.println(" deg");

    display.print("Recovery: ");
    display.print(validRecoveryReadings);
    display.print("/");
    display.println(
      REQUIRED_VALID_READINGS
    );

    display.println("Check sensor");

    display.display();

    return;
  }

  // ==========================================================
  // MANUAL SCREEN
  // ==========================================================

  if (currentMode == MANUAL_MODE) {

    display.println("MODE: MANUAL");

    display.println(
      "MANUAL OVERRIDE"
    );

    if (sensorDataReady) {

      display.print("T: ");
      display.print(temperature, 1);
      display.println(" C");

      display.print("H: ");
      display.print(humidity, 1);
      display.println(" %");

      display.print("Light: ");
      display.println(lightLevel);
    }

    else {

      display.println(
        "Waiting sensors..."
      );
    }

    display.println("LEDs: OFF");

    display.print("Vent: ");
    display.print(currentVentAngle);
    display.println(" deg");

    display.display();

    return;
  }

  // ==========================================================
  // AUTO SCREEN
  // ==========================================================

  display.println("MODE: AUTO");

  if (!sensorDataReady) {

    display.println(
      "Waiting sensors..."
    );

    display.display();

    return;
  }

  display.print("T: ");
  display.print(temperature, 1);
  display.println(" C");

  display.print("H: ");
  display.print(humidity, 1);
  display.println(" %");

  display.print("Light: ");
  display.println(lightLevel);

  display.print("LEDs: ");
  display.println(
    growLightsOn ? "ON" : "OFF"
  );

  display.print("Vent: ");
  display.print(currentVentAngle);
  display.println(" deg");

  // ----------------------------------------------------------
  // Current automatic condition
  // ----------------------------------------------------------

  if (criticalHeatActive) {

    display.println(
      "CRITICAL HEAT"
    );
  }

  else if (
    ventilationActive &&
    humidity < lowHumidityThreshold
  ) {

    display.println(
      "DRY: VENT LIMITED"
    );
  }

  else if (ventilationActive) {

    display.println(
      "COOLING ACTIVE"
    );
  }

  else if (growLightsOn) {

    display.println(
      "LOW LIGHT"
    );
  }

  else {

    display.println(
      "NORMAL"
    );
  }

  display.display();
}

// ============================================================
// UART STATUS
// ============================================================

/**
 * @brief Prints complete nursery status through UART.
 *
 * @return Nothing.
 */
void printStatus() {

  Serial.println();

  Serial.println(
    "========== NURSERY STATUS =========="
  );

  Serial.print("Mode: ");
  Serial.println(
    modeName(currentMode)
  );

  if (sensorDataReady) {

    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" C");

    Serial.print("Humidity: ");
    Serial.print(humidity, 1);
    Serial.println(" %");
  }

  else {

    Serial.println(
      "Temperature/Humidity: INVALID"
    );
  }

  Serial.print("Light ADC: ");
  Serial.println(lightLevel);

  Serial.print("LEDs: ");
  Serial.println(
    growLightsOn ? "ON" : "OFF"
  );

  Serial.print("Vent angle: ");
  Serial.print(currentVentAngle);
  Serial.println(" deg");

  Serial.print("Vent ON threshold: ");
  Serial.print(tempOnThreshold, 1);
  Serial.println(" C");

  Serial.print("Vent OFF threshold: ");
  Serial.print(tempOffThreshold, 1);
  Serial.println(" C");

  Serial.print("Critical ON: ");
  Serial.print(criticalTempOn, 1);
  Serial.println(" C");

  Serial.print("Critical OFF: ");
  Serial.print(criticalTempOff, 1);
  Serial.println(" C");

  Serial.print("Dark ON ADC: ");
  Serial.println(
    lightDarkOnThreshold
  );

  Serial.print("Bright OFF ADC: ");
  Serial.println(
    lightBrightOffThreshold
  );

  Serial.print("Low humidity: ");
  Serial.print(
    lowHumidityThreshold,
    1
  );
  Serial.println(" %");

  if (currentMode == SAFETY_MODE) {

    Serial.print("Fault: ");
    Serial.println(
      faultName(currentFault)
    );
  }

  Serial.println(
    "===================================="
  );
}

// ============================================================
// UART HELP
// ============================================================

/**
 * @brief Displays available UART commands.
 *
 * @return Nothing.
 */
void printHelp() {

  Serial.println();

  Serial.println(
    "UART COMMANDS"
  );

  Serial.println(
    "T30   -> set vent ON temp to 30C"
  );

  Serial.println(
    "L2200 -> set dark LED threshold"
  );

  Serial.println(
    "U35   -> set low humidity to 35%"
  );

  Serial.println(
    "S     -> print system status"
  );

  Serial.println(
    "H     -> print this help"
  );

  Serial.println();
}

// ============================================================
// UART COMMAND PROCESSING
// ============================================================

/**
 * @brief Processes a UART command.
 *
 * @param command Command received through Serial.
 * @return Nothing.
 */
void processSerialCommand(String command) {

  command.trim();

  if (command.length() == 0) {
    return;
  }

  char commandType =
    toupper(command.charAt(0));

  // ==========================================================
  // STATUS COMMAND
  // ==========================================================

  if (commandType == 'S') {

    printStatus();

    return;
  }

  // ==========================================================
  // HELP COMMAND
  // ==========================================================

  if (commandType == 'H') {

    printHelp();

    return;
  }

  float value =
    command.substring(1).toFloat();

  // ==========================================================
  // TEMPERATURE CONFIGURATION
  // ==========================================================

  if (commandType == 'T') {

    if (value >= 20.0 &&
        value <= 34.0) {

      tempOnThreshold = value;

      tempOffThreshold =
        value - 1.0;

      Serial.print(
        "Vent ON temperature set to "
      );

      Serial.print(
        tempOnThreshold,
        1
      );

      Serial.println(" C");

      Serial.print(
        "Vent OFF temperature set to "
      );

      Serial.print(
        tempOffThreshold,
        1
      );

      Serial.println(" C");

      if (
        currentMode == AUTO_MODE &&
        sensorDataReady
      ) {

        applyAutomaticControl();
      }
    }

    else {

      Serial.println(
        "Invalid T value. Use 20-34."
      );
    }

    return;
  }

  // ==========================================================
  // LIGHT CONFIGURATION
  // ==========================================================

  if (commandType == 'L') {

    int newThreshold =
      round(value);

    if (newThreshold >= 500 &&
        newThreshold <= 4000) {

      lightDarkOnThreshold =
        newThreshold;

      lightBrightOffThreshold =
        newThreshold - 400;

      Serial.print(
        "Dark ON threshold: "
      );

      Serial.println(
        lightDarkOnThreshold
      );

      Serial.print(
        "Bright OFF threshold: "
      );

      Serial.println(
        lightBrightOffThreshold
      );

      if (
        currentMode == AUTO_MODE &&
        sensorDataReady
      ) {

        applyAutomaticControl();
      }
    }

    else {

      Serial.println(
        "Invalid L value. Use 500-4000."
      );
    }

    return;
  }

  // ==========================================================
  // HUMIDITY CONFIGURATION
  // ==========================================================

  if (commandType == 'U') {

    if (value >= 10.0 &&
        value <= 90.0) {

      lowHumidityThreshold =
        value;

      Serial.print(
        "Low humidity threshold set to "
      );

      Serial.print(
        lowHumidityThreshold,
        1
      );

      Serial.println(" %");

      if (
        currentMode == AUTO_MODE &&
        sensorDataReady
      ) {

        applyAutomaticControl();
      }
    }

    else {

      Serial.println(
        "Invalid U value. Use 10-90."
      );
    }

    return;
  }

  Serial.println(
    "Unknown command. Type H for help."
  );
}

// ============================================================
// NON-BLOCKING UART INPUT
// ============================================================

/**
 * @brief Reads UART characters without blocking execution.
 *
 * @return Nothing.
 */
void handleSerialInput() {

  while (Serial.available() > 0) {

    char incoming =
      Serial.read();

    if (incoming == '\n') {

      processSerialCommand(
        serialBuffer
      );

      serialBuffer = "";
    }

    else if (incoming != '\r') {

      if (serialBuffer.length() < 32) {

        serialBuffer += incoming;
      }
    }
  }
}

// ============================================================
// SETUP
// ============================================================

/**
 * @brief Initializes sensors, outputs, interrupt, OLED and UART.
 *
 * @return Nothing.
 */
void setup() {

  Serial.begin(115200);

  Serial.println();

  Serial.println(
    "Automated Micro-Climate Nursery"
  );

  Serial.println(
    "System starting..."
  );

  // ==========================================================
  // LED OUTPUTS
  // ==========================================================

  pinMode(
    LED1_PIN,
    OUTPUT
  );

  pinMode(
    LED2_PIN,
    OUTPUT
  );

  pinMode(
    LED3_PIN,
    OUTPUT
  );

  setGrowLights(false);

  // ==========================================================
  // LDR ADC
  // ==========================================================

  pinMode(
    LDR_PIN,
    INPUT
  );

  analogReadResolution(12);

  // ==========================================================
  // MANUAL BUTTON
  // ==========================================================

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );

  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    buttonISR,
    FALLING
  );

  // ==========================================================
  // DHT22
  // ==========================================================

  dhtSensor.begin();

  // ==========================================================
  // I2C / OLED
  // ==========================================================

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );

  oledReady =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDRESS
    );

  if (oledReady) {

    Serial.println(
      "OLED initialized."
    );

    display.clearDisplay();

    display.setTextSize(1);

    display.setTextColor(
      SSD1306_WHITE
    );

    display.setCursor(0, 0);

    display.println(
      "Nursery Controller"
    );

    display.println(
      "Starting..."
    );

    display.display();
  }

  else {

    Serial.println(
      "WARNING: OLED initialization failed."
    );
  }

  // ==========================================================
  // SERVO PWM
  // ==========================================================

  ventServo.setPeriodHertz(50);

  ventServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  setVentAngle(
    VENT_CLOSED
  );

  // ==========================================================
  // INITIAL MODE
  // ==========================================================

  currentMode = AUTO_MODE;

  currentFault = NO_FAULT;

  printHelp();

  /*
     Allow approximately 2.5 seconds before the first
     scheduled DHT22 reading.
  */

  lastSensorRead =
    millis();

  lastStatusPrint =
    millis();

  lastOLEDUpdate =
    millis();

  Serial.println(
    "AUTO MODE active."
  );

  Serial.println(
    "Waiting for first sensor reading..."
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

/**
 * @brief Executes non-blocking system tasks.
 *
 * Sensors, OLED, UART and button handling operate using
 * millis() scheduling rather than delay().
 *
 * @return Nothing.
 */
void loop() {

  unsigned long now =
    millis();

  // UART
  handleSerialInput();

  // Manual override button
  handleButton();

  // ==========================================================
  // SENSOR SCHEDULE
  // ==========================================================

  if (
    now - lastSensorRead >=
    SENSOR_INTERVAL
  ) {

    lastSensorRead = now;

    readSensors();
  }

  // ==========================================================
  // OLED SCHEDULE
  // ==========================================================

  if (
    now - lastOLEDUpdate >=
    OLED_INTERVAL
  ) {

    lastOLEDUpdate = now;

    updateOLED();
  }

  // ==========================================================
  // UART LIVE STATUS
  // ==========================================================

  if (
    now - lastStatusPrint >=
    STATUS_INTERVAL
  ) {

    lastStatusPrint = now;

    printStatus();
  }
}