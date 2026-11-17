#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// Some ESP32 board packages do not define LED_BUILTIN. GPIO 2 is common on
// ESP32 DevKit boards; check your exact board if its built-in LED is elsewhere.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Wiring notes:
// - Connect VL53L0X SDA/SCL to the pins below and share GND with the ESP32.
// - Power the sensor according to its breakout-board specification; not every
//   bare VL53L0X module accepts 5 V on its supply pin.
// - Drive a 5 V active buzzer with a transistor/MOSFET, a gate/base resistor,
//   and a pull-down; connect the buzzer supply ground to ESP32 ground.
// - A flyback diode is needed for an inductive buzzer, but not usually for a
//   piezo element. Verify the buzzer type and polarity before wiring.
// WS2812B wiring (8 LEDs): connect DIN through a 5 V logic-level shifter
// (74AHCT125/74HCT245 recommended); share grounds; use an external 5 V supply,
// a 330-470 ohm DIN resistor, and a bulk capacitor at the strip power input.

// ESP32-WROOM-32D pin assignments. Change these constants and wiring together.
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
// Logic signal for a transistor/MOSFET; never drive a 5 V buzzer directly.
constexpr uint8_t BUZZER_PIN = 32;
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;
constexpr bool STATUS_LED_ACTIVE_LOW = false;
constexpr uint8_t RGB_DATA_PIN = 13;
constexpr uint16_t RGB_LED_COUNT = 8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ;
// Conservative software brightness limit; increase only after checking supply
// capacity and strip temperature.
constexpr uint8_t RGB_STRIP_BRIGHTNESS = 40;
constexpr uint32_t RGB_ERROR_BLINK_INTERVAL_MS = 500;

// Polling cadence and recovery policy.
constexpr uint32_t MEASUREMENT_INTERVAL_MS = 100;
constexpr uint32_t SENSOR_RETRY_INTERVAL_MS = 2000;
constexpr uint32_t ERROR_LOG_INTERVAL_MS = 2000;
constexpr uint32_t STALE_DATA_TIMEOUT_MS = 500;
constexpr Adafruit_VL53L0X::VL53L0X_Sense_config_t TOF_RANGE_PROFILE =
    Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE;
// Values above the normal VL53L0X range are treated as invalid/sentinel data.
constexpr uint16_t MAX_PLAUSIBLE_DISTANCE_MM = 4000;
// Onboard LED blink half-period is mapped from near (fast) to far (slow).
constexpr uint16_t LED_SPEED_NEAR_MM = 50;
constexpr uint16_t LED_SPEED_FAR_MM = 1000;
constexpr uint32_t LED_FAST_INTERVAL_MS = 10;
constexpr uint32_t LED_SLOW_INTERVAL_MS = 3000;
constexpr uint32_t LED_ERROR_INTERVAL_MS = 10000;
constexpr uint32_t ALERT_BEEP_INTERVAL_MS = 500;
constexpr uint32_t WARNING_BEEP_INTERVAL_MS = 100;
constexpr uint32_t BEEP_DURATION_MS = 100;
constexpr uint32_t ERROR_BEEP_INTERVAL_MS = 1000;
constexpr uint32_t ERROR_BEEP_DURATION_MS = 250;
constexpr uint16_t WARNING_DISTANCE_MM = 180;
constexpr uint16_t SAFE_DISTANCE_MM = 300;
constexpr uint8_t MAX_CONSECUTIVE_MEASUREMENT_ERRORS = 3;

static_assert(I2C_SDA_PIN != I2C_SCL_PIN, "I2C SDA and SCL must use different pins");

Adafruit_VL53L0X tof;
VL53L0X_RangingMeasurementData_t measurement;
Adafruit_NeoPixel rgbStrip(RGB_LED_COUNT, RGB_DATA_PIN, NEO_GRB + NEO_KHZ800);

enum class SystemState : uint8_t {
  SAFE,
  ALERT,
  WARNING,
  SENSOR_ERROR
};

SystemState currentState = SystemState::SENSOR_ERROR;
bool stateReported = false;
bool sensorReady = false;
bool errorLogged = false;
bool hasValidMeasurement = false;
uint8_t consecutiveMeasurementErrors = 0;
uint32_t lastMeasurementAt = 0;
uint32_t lastSensorRetryAt = 0;
uint32_t lastErrorLogAt = 0;
uint32_t lastValidMeasurementAt = 0;
uint32_t lastStatusLedToggleAt = 0;
uint16_t latestDistanceMm = SAFE_DISTANCE_MM;
bool statusLedOn = false;
bool rgbErrorOn = true;
uint32_t lastRgbErrorToggleAt = 0;

uint32_t statusLedIntervalMs(SystemState state, uint16_t distanceMm) {
  if (state == SystemState::SENSOR_ERROR) {
    return LED_ERROR_INTERVAL_MS;
  }

  const uint16_t boundedDistance =
      (distanceMm < LED_SPEED_NEAR_MM) ? LED_SPEED_NEAR_MM :
      (distanceMm > LED_SPEED_FAR_MM) ? LED_SPEED_FAR_MM : distanceMm;
  return map(boundedDistance,
             LED_SPEED_NEAR_MM, LED_SPEED_FAR_MM,
             LED_FAST_INTERVAL_MS, LED_SLOW_INTERVAL_MS);
}
void fillRgbStrip(uint8_t red, uint8_t green, uint8_t blue) {
  rgbStrip.fill(rgbStrip.Color(red, green, blue));
  rgbStrip.show();
}

void showStateColor(SystemState state) {
  switch (state) {
    case SystemState::SAFE:
      fillRgbStrip(0, 255, 0);
      break;
    case SystemState::ALERT:
      fillRgbStrip(255, 127, 0);
      break;
    case SystemState::WARNING:
      fillRgbStrip(255, 0, 0);
      break;
    case SystemState::SENSOR_ERROR:
      rgbErrorOn = true;
      lastRgbErrorToggleAt = millis();
      fillRgbStrip(0, 0, 255);
      break;
  }
}

void updateRgbErrorBlink(uint32_t now) {
  if (currentState != SystemState::SENSOR_ERROR ||
      now - lastRgbErrorToggleAt < RGB_ERROR_BLINK_INTERVAL_MS) {
    return;
  }

  lastRgbErrorToggleAt = now;
  rgbErrorOn = !rgbErrorOn;
  if (rgbErrorOn) {
    fillRgbStrip(255, 70, 0);
  } else {
    fillRgbStrip(0, 0, 0);
  }
}

const char *stateName(SystemState state) {
  switch (state) {
    case SystemState::SAFE:
      return "SAFE";
    case SystemState::ALERT:
      return "ALERT";
    case SystemState::WARNING:
      return "WARNING";
    case SystemState::SENSOR_ERROR:
      return "SENSOR_ERROR";
  }
  return "SENSOR_ERROR";
}

SystemState classifyDistance(uint16_t distanceMm) {
  // Inclusive boundaries: >=30 cm SAFE, <=10 cm WARNING.
  if (distanceMm >= SAFE_DISTANCE_MM) {
    return SystemState::SAFE;
  }
  if (distanceMm <= WARNING_DISTANCE_MM) {
    return SystemState::WARNING;
  }
  return SystemState::ALERT;
}

void setState(SystemState newState) {
  if (!stateReported || newState != currentState) {
    currentState = newState;
    stateReported = true;
    // Serial.print('[');
    // Serial.print(millis());
    // Serial.print(" ms] State changed: ");
    // Serial.println(stateName(currentState));
    showStateColor(currentState);
  }
}

void updateStatusLed(SystemState state, uint16_t distanceMm, uint32_t now) {
  // Closer obstacles blink faster; errors use a fixed rapid blink.
  const uint32_t interval = statusLedIntervalMs(state, distanceMm);

  if (now - lastStatusLedToggleAt >= interval) {
    lastStatusLedToggleAt = now;
    statusLedOn = !statusLedOn;
    const uint8_t outputLevel = statusLedOn ? HIGH : LOW;
    digitalWrite(STATUS_LED_PIN,
                 STATUS_LED_ACTIVE_LOW ? !outputLevel : outputLevel);
    // Serial.print('[');
    // Serial.print(now);
    // Serial.print(" ms] Status LED=");
    // Serial.print(statusLedOn ? "ON" : "OFF");
    // Serial.print(", half-period=");
    // Serial.print(interval);
    // Serial.println(" ms");
  }
}

void updateBuzzer(SystemState state, uint32_t now) {
  static bool scheduleStarted = false;
  static bool beepIsOn = false;
  static uint32_t lastBeepAt = 0;
  static uint32_t beepStartedAt = 0;
  static SystemState scheduledState = SystemState::SENSOR_ERROR;

  if (state == SystemState::SAFE) {
    if (beepIsOn) {
      // Serial.print('[');
      // Serial.print(now);
      // Serial.println(" ms] Buzzer=OFF (SAFE)");
    }
    digitalWrite(BUZZER_PIN, LOW);
    scheduleStarted = false;
    beepIsOn = false;
    scheduledState = state;
    return;
  }

  // SENSOR_ERROR has a separate fault tone instead of becoming silent.
  const uint32_t interval = (state == SystemState::WARNING)
                                ? WARNING_BEEP_INTERVAL_MS
                                : (state == SystemState::ALERT)
                                      ? ALERT_BEEP_INTERVAL_MS
                                      : ERROR_BEEP_INTERVAL_MS;
  const uint32_t duration = (state == SystemState::SENSOR_ERROR)
                                ? ERROR_BEEP_DURATION_MS
                                : BEEP_DURATION_MS;

  if (!scheduleStarted || state != scheduledState) {
    scheduledState = state;
    scheduleStarted = true;
    beepIsOn = true;
    lastBeepAt = now;
    beepStartedAt = now;
    digitalWrite(BUZZER_PIN, HIGH);
    // Serial.print('[');
    // Serial.print(now);
    // Serial.print(" ms] Buzzer=ON, state=");
    // Serial.println(stateName(state));
    return;
  }

  if (beepIsOn && now - beepStartedAt >= duration) {
    digitalWrite(BUZZER_PIN, LOW);
    beepIsOn = false;
    // Serial.print('[');
    // Serial.print(now);
    // Serial.println(" ms] Buzzer=OFF");
  }

  if (!beepIsOn && now - lastBeepAt >= interval) {
    digitalWrite(BUZZER_PIN, HIGH);
    beepIsOn = true;
    lastBeepAt = now;
    beepStartedAt = now;
    // Serial.print('[');
    // Serial.print(now);
    // Serial.print(" ms] Buzzer=ON, state=");
    // Serial.println(stateName(state));
  }
}

void logMeasurementError(uint8_t rangeStatus, uint32_t now) {
  if (!errorLogged || now - lastErrorLogAt >= ERROR_LOG_INTERVAL_MS) {
    Serial.print("ToF measurement error, RangeStatus=");
    Serial.print(rangeStatus);
    Serial.print(", consecutive errors=");
    Serial.println(consecutiveMeasurementErrors + 1);
    lastErrorLogAt = now;
    errorLogged = true;
  }
}

void handleMeasurementFailure(uint8_t rangeStatus, uint32_t now) {
  setState(SystemState::SENSOR_ERROR);
  logMeasurementError(rangeStatus, now);

  if (consecutiveMeasurementErrors < MAX_CONSECUTIVE_MEASUREMENT_ERRORS) {
    ++consecutiveMeasurementErrors;
  }

  // Retry full initialization after repeated bad measurements. The error state
  // and its audible fault pattern remain active during the recovery interval.
  if (consecutiveMeasurementErrors >= MAX_CONSECUTIVE_MEASUREMENT_ERRORS) {
    Serial.print('[');
    Serial.print(now);
    Serial.println(" ms] Too many measurement errors; sensor reinitialization scheduled");
    sensorReady = false;
    lastSensorRetryAt = now;
  }
}

bool initializeTof() {
  Wire.beginTransmission(VL53L0X_I2C_ADDR);
  const uint8_t i2cError = Wire.endTransmission();
  if (i2cError == 0) {
    Serial.println("VL53L0X responded at I2C address 0x29");
  } else {
    Serial.print("No I2C response at 0x29; Wire error=");
    Serial.println(i2cError);
  }

  return tof.begin(VL53L0X_I2C_ADDR, true, &Wire, TOF_RANGE_PROFILE);
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_LOW ? HIGH : LOW);

  rgbStrip.begin();
  rgbStrip.setBrightness(RGB_STRIP_BRIGHTNESS);
  rgbStrip.clear();
  rgbStrip.show();

  Serial.begin(115200);
  Serial.println();
  Serial.println("=== Active Door Guard: serial diagnostics ===");
  Serial.println("Serial Monitor baud: 115200");
  Serial.print("I2C SDA/SCL: GPIO ");
  Serial.print(I2C_SDA_PIN);
  Serial.print('/');
  Serial.println(I2C_SCL_PIN);
  Serial.print("Status LED pin: GPIO ");
  Serial.println(STATUS_LED_PIN);
  Serial.print("Buzzer control pin: GPIO ");
  Serial.println(BUZZER_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  // ESP32 Arduino Wire timeout prevents a stuck I2C transfer from blocking forever.
  Wire.setTimeOut(50);

  Serial.println("Active door guard starting");
  setState(SystemState::SENSOR_ERROR);

  sensorReady = initializeTof();
  lastSensorRetryAt = millis();
  if (sensorReady) {
    Serial.println("VL53L0X initialized");
  } else {
    Serial.println("VL53L0X initialization failed; retrying");
  }
}

void loop() {
  const uint32_t now = millis();

  if (!sensorReady && now - lastSensorRetryAt >= SENSOR_RETRY_INTERVAL_MS) {
    lastSensorRetryAt = now;
    Serial.print('[');
    Serial.print(now);
    Serial.println(" ms] Retrying VL53L0X initialization");
    sensorReady = initializeTof();
    if (sensorReady) {
      consecutiveMeasurementErrors = 0;
      hasValidMeasurement = false;
      Serial.println("VL53L0X initialization succeeded; waiting for a valid reading");
    } else {
      Serial.println("VL53L0X initialization failed; retry in 2 seconds");
    }
  }

  // A stale SAFE reading must never be treated as current sensor evidence.
  if (sensorReady && hasValidMeasurement &&
      now - lastValidMeasurementAt >= STALE_DATA_TIMEOUT_MS) {
    Serial.print('[');
    Serial.print(now);
    Serial.println(" ms] ToF data stale; entering SENSOR_ERROR and retrying sensor");
    setState(SystemState::SENSOR_ERROR);
    sensorReady = false;
    lastSensorRetryAt = now;
    hasValidMeasurement = false;
  }

  if (sensorReady && now - lastMeasurementAt >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurementAt = now;
    // The application scheduler and buzzer are nonblocking, but this library
    // measurement call is synchronous. Wire timeout limits I2C waits; it cannot
    // guarantee recovery from every internal library/device lockup.
    tof.rangingTest(&measurement, false);

    if (measurement.RangeStatus == 0 &&
      measurement.RangeMilliMeter <= MAX_PLAUSIBLE_DISTANCE_MM) {
      const uint16_t distanceMm = measurement.RangeMilliMeter;
      errorLogged = false;
      consecutiveMeasurementErrors = 0;
      hasValidMeasurement = true;
      lastValidMeasurementAt = millis();
      latestDistanceMm = distanceMm;
      setState(classifyDistance(distanceMm));
      Serial.print('[');
      Serial.print(lastValidMeasurementAt);
      Serial.print(" ms] Distance=");
      Serial.print(distanceMm / 10.0f, 1);
      Serial.print(" cm, RangeStatus=");
      Serial.print(measurement.RangeStatus);
      Serial.print(", state=");
      Serial.print(stateName(currentState));
      Serial.print(", LED half-period=");
      Serial.print(statusLedIntervalMs(currentState, distanceMm));
      Serial.println(" ms");
    } else {
      // Even a nominal status is rejected if the result looks like a sentinel.
      handleMeasurementFailure(measurement.RangeStatus, millis());
    }
  }

  updateBuzzer(currentState, millis());
  updateStatusLed(currentState, latestDistanceMm, millis());
  updateRgbErrorBlink(millis());
}