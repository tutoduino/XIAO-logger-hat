/**
 * @brief Zigbee temperature and humidity sensor 
 * XIAO ESP32C6 + XIAO LOGGER HAT
 * https://tutoduino.fr/
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

// ─── Libraries ────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <Arduino.h>
#include "Zigbee.h"
#include <SensirionI2cSht4x.h>

// ─── Zigbee endpoints ─────────────────────────────────────────────────────────
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define ANALOG_DEVICE_ENDPOINT_NUMBER 1

// ─── Timing ───────────────────────────────────────────────────────────────────
#define uS_TO_S_FACTOR 1000000ULL  // µs → s
#define TIME_TO_SLEEP 300          // deep-sleep duration (seconds)
#define JOIN_TIMEOUT_MS 60000      // max time to join network (ms)
#define POST_JOIN_DELAY_MS 3000    // settle time after joining (ms)
#define REPORT_WAIT_MS 10000       // max wait for report ACKs (ms)
#define KEEPALIVE_EXTRA_S 60       // extra margin on top of sleep (seconds)

// ─── Pins ─────────────────────────────────────────────────────────────────────
const uint8_t PIN_LED = LED_BUILTIN;
const uint8_t PIN_BUTTON = BOOT_PIN;

// ─── Zigbee objects ───────────────────────────────────────────────────────────
ZigbeeTempSensor zbTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

// ─── Sensor objects ───────────────────────────────────────────────────────────
SensirionI2cSht4x sht40;

// ─── Report-tracking state ────────────────────────────────────────────────────
// Each successful report call decrements the counter; we wait until it
// reaches 0 (or the REPORT_WAIT_MS timeout) before sleeping.
volatile int8_t pendingReports = 0;
volatile bool needResend = false;

// ─── Helpers ──────────────────────────────────────────────────────────────────
void goToSleep() {
  Serial.printf("Going to sleep for %d seconds.\r\n", TIME_TO_SLEEP);
  Serial.flush();
  esp_deep_sleep_start();
}

// 3.7 V Li-Ion battery voltage
const float minVoltage = 3.0;
const float maxVoltage = 4.2;

// Mapp float values to percentage
uint8_t mapFloat(float x, float in_min, float in_max) {
  float val;
  val = (x - in_min) * (100) / (in_max - in_min);
  if (val < 0) {
    val = 0;
  } else if (val > 100) {
    val = 100;
  }
  return (uint8_t)val;
}

// Get battery voltage
float getVbatt() {
  uint16_t Vbatt = 0;
  float returnedVbatt=0.0;
  delay(100);
  Vbatt = analogRead(D1);  // Read and accumulate ADC voltage
  Serial.print("Battery voltage: ");
  returnedVbatt = 2.0 * Vbatt / 16 / 1000.0;
  Serial.print(returnedVbatt);
  Serial.println(" V");
  return (returnedVbatt);  // Adjust for 1:2 divider and convert to volts
}

// ─── Zigbee response callbacks ────────────────────────────────────────────────
void onTempSensorResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status) {
  Serial.printf("[TempSensor] cmd=%d status=%d\r\n", (int)command, (int)status);

  if (command == ZB_CMD_REPORT_ATTRIBUTE) {
    if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
      pendingReports--;
    } else if (status == ESP_ZB_ZCL_STATUS_FAIL) {
      needResend = true;
    }
  }
}

// ─── Send readings and wait for ACKs ─────────────────────────────────────────
void sendReports(float temp, float humi, uint8_t batPercent) {
  zbTempSensor.setTemperature(temp);
  zbTempSensor.setHumidity(humi);
  zbTempSensor.setBatteryPercentage(batPercent);

  const int maxRetries = 3;
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    pendingReports = 2;  // expecting one ACK from each endpoint
    needResend = false;

    Serial.printf("Sending reports (attempt %d/%d)...\r\n", attempt, maxRetries);
    zbTempSensor.report();

    unsigned long t0 = millis();
    while (pendingReports > 0 && (millis() - t0) < REPORT_WAIT_MS) {
      if (needResend) {
        Serial.println("Coordinator signalled failure – resending.");
        needResend = false;
        pendingReports = 2;
        zbTempSensor.report();
        t0 = millis();  // reset timeout after resend
      }
      delay(100);
    }

    if (pendingReports <= 0) {
      Serial.println("All reports acknowledged.");
      return;
    }

    Serial.printf("Report attempt %d timed out (pending=%d).\r\n",
                  attempt, pendingReports);
  }

  Serial.println("All retries exhausted – sleeping anyway.");
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  uint8_t battPercentage;

  Serial.begin(115200);
  Serial.println("\r\n=== XIAO HAT Zigbee sensor starting ===");
  delay(3000);
  // LED off (active-low on XIAO)
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // Pin D1 (ADC) used for battery voltage monitoring
  pinMode(D1, INPUT);
  //analogReadResolution(12);

  // Boot button → factory-reset Zigbee if held at startup
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  if (digitalRead(PIN_BUTTON) == LOW) {
    Serial.println("Boot button held – performing Zigbee factory reset!");
    Zigbee.factoryReset();  // clears stored network credentials
  }

  // Configure deep-sleep wakeup timer now; used in all exit paths
  esp_sleep_enable_timer_wakeup((uint64_t)TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // ── I2C & sensors ──────────────────────────────────────────────────────────
  Wire.begin();

  // SHT40 temperature & humidity
  sht40.begin(Wire, SHT40_I2C_ADDR_44);
  sht40.softReset();
  delay(500);

  float temp = 0.0f, humi = 0.0f;
  int16_t shtError = sht40.measureLowestPrecision(temp, humi);
  if (shtError != 0) {
    char errMsg[64];
    errorToString(shtError, errMsg, sizeof(errMsg));
    Serial.printf("SHT40 error: %s – will report 0 values.\r\n", errMsg);
    temp = 0.0f;
    humi = 0.0f;
    // Do NOT return here; continue so the device can still sleep properly
  } else {
    Serial.printf("SHT40 → Temp: %.2f °C  Humi: %.1f %%\r\n", temp, humi);
  }

  float vBat;
  vBat = getVbatt();
  battPercentage = mapFloat(vBat, minVoltage, maxVoltage);
  Serial.print("Battery = ");
  Serial.print(battPercentage);
  Serial.println(" %");

  // ── Zigbee configuration ───────────────────────────────────────────────────
  zbTempSensor.setManufacturerAndModel("Z-XIAO", "HAT 02");
  zbTempSensor.setMinMaxValue(-20, 80);
  zbTempSensor.setTolerance(1);
  zbTempSensor.addHumiditySensor(0, 100, 1);
  zbTempSensor.onDefaultResponse(onTempSensorResponse);
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY);
  Zigbee.addEndpoint(&zbTempSensor);

  // Keep-alive must comfortably exceed the sleep period so the coordinator
  // doesn't mark us as offline between wake-ups.
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive =
    (TIME_TO_SLEEP + KEEPALIVE_EXTRA_S) * 1000;

  Zigbee.setTimeout(10000);  // 10 s to start Zigbee stack

  Serial.println("Starting Zigbee stack...");
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start – rebooting.");
    ESP.restart();
  }
  Serial.println("Zigbee stack started.");

  // ── Network join ──────────────────────────────────────────────────────────
  Serial.print("Joining network");
  unsigned long joinStart = millis();
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(300);
    if (millis() - joinStart > JOIN_TIMEOUT_MS) {
      Serial.println("\r\nJoin timed out – rebooting.");
      ESP.restart();
    }
  }
  Serial.println("\r\nConnected to Zigbee network.");

  // Give the coordinator a moment to fully register the device
  delay(POST_JOIN_DELAY_MS);

  // ── Report sensor values ──────────────────────────────────────────────────
  sendReports(temp, humi, battPercentage);

  // ── Sleep ─────────────────────────────────────────────────────────────────
  goToSleep();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // Everything runs in setup(); the device never reaches loop().
}
