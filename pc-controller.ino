#define WIFI_SSID "__REPLACE__"   // WiFi SSID
#define WIFI_PSW "__REPLACE__"      // WiFi password
#define SERVER_PORT 80             // Server port
#define LOG_TIME 3000              // Log time in ms
#define LED_COUNT 100              // Number of LEDs in the strip
#define FAN_FREQ 25000             // fan pwm frequency 25000
#define FAN_RES 8                  // 8 bit resolution
#define FAN_PWM 100                // fan pwm value
#define FAN_MAX_RPM 3000           // fan max fan RPM
#define MAX_VOLTAGE 3.3            // max voltage of the dc fan, 12v

#include "clases.h"

// ESP-WROOM-32
// GPIO_NUM_1 DOES NOT WORK
// FIRST 3 ON BOTH SIDES ARE SHARED WITH FLASH MEMORY, CAN'T BE USED AS REGULAR GPIO

// TODO ARGB CONTROLLER ON GPIO_NUM_23 PIN

// SHOUD BE ANALOG-TO-DIGITAL CONVERTER(ADC) PIN
std::vector<VoltageInputSettings> voltage_input_settings = {
  // VoltageInputSettings(GPIO_NUM_36, MAX_VOLTAGE),
  // VoltageInputSettings(GPIO_NUM_39, MAX_VOLTAGE),
  // VoltageInputSettings(GPIO_NUM_34, MAX_VOLTAGE),
  // VoltageInputSettings(GPIO_NUM_35, MAX_VOLTAGE),
};

// SHOULD BE GPIO PIN THAT IS NOT SHARED WITH FLASH MEMORY
std::vector<TachoInputSettings> tacho_input_settings = {
  // LEFT SIDE PINS
  TachoInputSettings(GPIO_NUM_33),
  TachoInputSettings(GPIO_NUM_26),
  TachoInputSettings(GPIO_NUM_14),
  TachoInputSettings(GPIO_NUM_13),
  // RIGHT SIDE PINS
  // TachoInputSettings(GPIO_NUM_22), // sometimes high rpm spikes
  // TachoInputSettings(GPIO_NUM_21), // wrong values
  TachoInputSettings(GPIO_NUM_18),
  TachoInputSettings(GPIO_NUM_17),
  TachoInputSettings(GPIO_NUM_4),
  // TachoInputSettings(GPIO_NUM_2), // hangs when connected, wrong values when disconnected
};

// SHOULD BE GPIO PIN THAT IS NOT SHARED WITH FLASH MEMORY
std::vector<FanSettings> fan_setings = {
  // LEFT SIDE PINS 
  FanSettings(0, GPIO_NUM_32, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(1, GPIO_NUM_25, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(2, GPIO_NUM_27, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(3, GPIO_NUM_12, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  // RIGHT SIDE PINS
  FanSettings(4, GPIO_NUM_3, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(5, GPIO_NUM_19, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(6, GPIO_NUM_5, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(7, GPIO_NUM_16, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(8, GPIO_NUM_0, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
  FanSettings(9, GPIO_NUM_15, FAN_FREQ, FAN_RES, 100, FAN_MAX_RPM),
};

PcController controller = PcController();

void setup() {
  Serial.begin(115200);
  delay(1000);
  controller.setup(fan_setings, tacho_input_settings, voltage_input_settings);
}

void loop() {
  delay(100);
  controller.loop();
  controller.log();
}