#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>

#ifndef WIFI_SSID
#define WIFI_SSID "_________"    // WiFi SSID
#endif

#ifndef WIFI_PSW
#define WIFI_PSW "_________"     // WiFi password
#endif

#ifndef SERVER_PORT
#define SERVER_PORT 80           // Server port
#endif

#ifndef LOG_TIME
#define LOG_TIME 3000            // Log time in ms
#endif

class FanSettings {
public:
  uint8_t channel;     // 0-7, total of 8 channels
  uint16_t freq;       // pwm frequency 25000
  uint8_t resolution;  // 8 bit resolution
  uint8_t pwm;         // pwm fan speed (0 <= value <= 255) in 8 bit resolution
  uint8_t pin;         // GPIO pin number
  uint16_t maxRPM;     // max fan rpm
  FanSettings(int channel, int pin, int freq, int resolution, int pwm, int maxRPM) {
    this->channel = channel;
    this->freq = freq;
    this->resolution = resolution;
    this->pwm = pwm;
    this->pin = pin;
    this->maxRPM = maxRPM;
  }
};

class VoltageInputSettings {
public:
  uint8_t pin;          // ANALOG pin number
  float maxVoltage;     // max voltage
  VoltageInputSettings(int pin, float maxVoltage = 12) {
    this->pin = pin;
    this->maxVoltage = maxVoltage;
  }
};

class TachoInputSettings {
public:
  uint8_t pin;          // GPIO pin number
  TachoInputSettings(int pin) {
    this->pin = pin;
  }
};

// DC fan voltage monitoring 0-13v reduced to 0-3.3v
class VoltageInput {
private:
  uint8_t pin;          // pin number
  float maxVoltage;     // max voltage

public:
  VoltageInput(int pin, float maxVoltage = 12) {
    this->pin = pin;
    this->maxVoltage = maxVoltage;
    pinMode(pin, INPUT);
  }

  // returns voltage value based on resolution
  // 4 bit resolution will return 0-15 value
  // 8 bit resolution will return 0-255 value
  // 12 bit resolution will return 0-4095 value
  int getVoltageResolution(int resolution = 8) {
    int value = analogRead(this->pin); // returns 0-4095
    return (value / 4095) * pow(2, resolution);
  }

  // returns 0 - maxVoltage value
  float getVoltage() {
    int value = analogRead(this->pin); // returns 0-4095
    float voltage = (value / 4095) * this->maxVoltage;
    if (voltage < 0) {
      return 0;
    }
    return voltage;
  }

  int getPin() { return pin; }

  String toJSON() {
    String json = "{";
    json += "\"pin\": " + String(pin) + ",";
    json += "\"voltage\": " + String(getVoltage()) + ",";
    json += "\"voltage_resolution\": " + String(getVoltageResolution(4));
    json += "}";
    return json;
  }
};

class TachoInput {
private:  
  uint8_t pin;                                // GPIO pin number
  unsigned int rpm;                           // current rpm
  unsigned int pulses;                        // pulses counter
  unsigned int pulses_per_rev;                // pulses per revolution
  unsigned int cycle;                         // how often mesure pulses in ms
  unsigned long prev_time;                    // last rpm mesurment time

  static void IRAM_ATTR pulse_counter(void *arg) {
    try {
      TachoInput *tacho = (TachoInput *)arg;
      tacho->setPulses(tacho->getPulses() + 1);
    } catch (const std::exception &e) {
      Serial.println(e.what());
    }
  }

public:
  TachoInput(int pin, int cycle = 1000, int pulses_per_rev = 2) {
    this->pin = pin;
    this->prev_time = millis();
    this->rpm = 0;
    this->pulses = 0;
    this->pulses_per_rev = pulses_per_rev;
    this->cycle = cycle;
    pinMode(pin, INPUT);
  }

  int getPulses() { return pulses; }

  void setPulses(int pulse) { pulses = pulse; }

  int getPin() { return pin; }

  int getRPM() { return rpm; }

  void watch() {
    unsigned long curent_time = millis();
    unsigned long delta_time = curent_time - prev_time;
    if (delta_time > cycle) {
      detachInterrupt(digitalPinToInterrupt(pin)); 
      if (getPulses() != 0) {
        float revolutions = (float)getPulses() / (float)pulses_per_rev;
        float seconds_passed = (float)delta_time / (float)1000;
        rpm = (revolutions / seconds_passed) * 60;
      } else {
        rpm = 0;
      }
      setPulses(0);
      prev_time = curent_time;
      attachInterruptArg(digitalPinToInterrupt(pin), pulse_counter, this, FALLING);
    }
  }

  String toJSON() {
    String json = "{";
    json += "\"pin\": " + String(pin) + ",";
    json += "\"rpm\": " + String(rpm) + ",";
    json += "\"pulses\": " + String(pulses) + "";
    json += "}";
    return json;
  }
};

class Fan {
private:
  uint8_t pin;         // GPIO pin number
  uint8_t channel;     // 0-7, total of 8 channels
  uint16_t freq;       // pwm frequency 25000
  uint8_t resolution;  // 8 bit resolution
  uint8_t pwm;         // pwm fan speed (0 <= value <= 255) in 8 bit resolution
  uint16_t maxRPM;     // max fan rpm

public:
  Fan(int channel, int pin, int freq, int resolution, int pwm, int maxRPM) {
    this->channel = channel;
    this->freq = freq;
    this->resolution = resolution;
    this->pwm = pwm;
    this->pin = pin;
    this->maxRPM = maxRPM;
    ledcSetup(channel, freq, resolution);
    setSpeed(pwm);
  }

  int getResolution() { return this->resolution; }

  void setSpeed(int pwm) {
    this->pwm = pwm;
    if(pwm == 0) {
      // stop ledc
      ledcDetachPin(this->pin);
    }else{
      // attach ledc
      ledcAttachPin(this->pin, this->channel);
      ledcWrite(this->channel, this->pwm);
    }
  }

  // int getRPM() { return (int)(((float)this->pwm / 255) * this->maxRPM); }
  int getPWM() { return this->pwm; }
  int getPin() { return this->pin; }

  String toJSON() {
    String json = "{";
    json += "\"pin\": " + String(pin) + ",";
    json += "\"pwm\": " + String(pwm) + ",";
    json += "}";
    return json;
  }

  FanSettings getSettings() {
    return FanSettings(this->channel, this->pin, this->freq, this->resolution, this->pwm, this->maxRPM);
  }
};


// TODO
// option to controll RGB led strips
// option to controll on of btn
// option to add kvm remote controll, usb, mouse, hdmi

// Adafruit_NeoPixel strip(LED_COUNT, GPIO_NUM_0, NEO_GRB + NEO_KHZ800);

// // Set all LEDs to red
// for (int i = 0; i < LED_COUNT; i++) {
//   strip.setPixelColor(i, strip.Color(255, 0, 0));  // Red color (RGB
//   values)
// }
// strip.show();  // Send the updated pixel colors to the strip

// Set all fans to 0 speed
// for (int i = 0; i < sizeof(fans); i++) {
//   fans[i].setSpeed(0);
// }

// lineary set all fan sepeeds from 0 to 255 based on voltage_inputs[0].voltage
// value for (int i = 0; i < sizeof(fans); i++) {
//   int resulution = fans[i].getResolution();
//   int fan_speed = voltage_inputs[0].getVoltage(resulution);

//   //set random fan speed
//   int fan_speed = random(0, 255);
//   fans[i].setSpeed(fan_speed);
//   Serial.print("Fan speed: ");
//   Serial.println(fan_speed);
// }

class PcController {
private:
  std::vector<FanSettings> fan_settings;
  std::vector<VoltageInputSettings> voltage_input_settings;
  std::vector<TachoInputSettings> tacho_input_settings;
  
  std::vector<Fan> fans;
  std::vector<VoltageInput> voltage_inputs;
  std::vector<TachoInput> tacho_inputs;

  unsigned long previousTimeLog = 0;
  unsigned long currentTimeLog = millis();
  unsigned long timeoutTimeLog = LOG_TIME;

  unsigned long lastWifiConnect = 0;
  bool isConnectigToWifi = false;

  AsyncWebServer server = AsyncWebServer(SERVER_PORT);

public:
  PcController() {}

  void loadVoltageInputs(std::vector<VoltageInputSettings> dfis) {
    for (VoltageInputSettings &voltage : dfis) {
      voltage_inputs.push_back(VoltageInput(voltage.pin, voltage.maxVoltage));
    }
  }

  void loadTachoInputs(std::vector<TachoInputSettings> tis) {
    for (TachoInputSettings &tacho : tis) {
      tacho_inputs.push_back(TachoInput(tacho.pin));
    }
  }

  void loadFans(std::vector<FanSettings> default_fan_settings) {
    int size = 8;   // fan settings size
    int count = 20; // fan count limit
    int addr = 0;   // eeprom begining address

    // load fans settings from EEPROM
    Serial.println("Fan settings, loading from EEPROM");
    for (int i = 0; i < count; i++) {
      uint8_t channel = EEPROM.read(addr);
      uint8_t pin = EEPROM.read(addr + 1);
      uint16_t freq = (EEPROM.read(addr + 2) << 8) | EEPROM.read(addr + 3);
      uint8_t resolution = EEPROM.read(addr + 4);
      uint8_t pwm = EEPROM.read(addr + 5);
      uint16_t maxRPM = (EEPROM.read(addr + 6) << 8) | EEPROM.read(addr + 7);

      if (pin == 255) { continue; } // value 255 means empty slot
      Serial.println("Fan settings, loading from EEPROM at address: " + String(addr));
      // save fan settings to array
      fan_settings.push_back(FanSettings(channel, pin, freq, resolution, pwm, maxRPM));

      Serial.println("channel: " + String(channel));
      Serial.println("pin: " + String(pin));
      Serial.println("freq: " + String(freq));
      Serial.println("resolution: " + String(resolution));
      Serial.println("pwm: " + String(pwm));
      Serial.println("maxRPM: " + String(maxRPM));
      addr += size;
    }

    // if no fans are saved in EEPROM, create from default fans
    if (fan_settings.size() == 0) {
      Serial.println("Fan settings, no saved settings found, creating default fans");
      for (FanSettings &fan : default_fan_settings) {
        fan_settings.push_back(fan);
        saveFanSettings(fan);
      }
    }

    // create fans
    for (FanSettings &fan : fan_settings) {
      fans.push_back(Fan(fan.channel, fan.pin, fan.freq, fan.resolution, fan.pwm, fan.maxRPM));
    }
  }

  // find fan in array and save to EEPROM
  void saveFanSettings(FanSettings fan) {
    int size = 8; // fan settings size
    int count = 20; // fan count limit
    int addr = 0; // eeprom begining address
    bool found = false;
    int empty_addr = -1;

    // find first config by pin
    for (int i = 0; i < count; i++) {
      uint8_t pin = EEPROM.read(addr + 1);
      if (pin == 255 && empty_addr == -1) {
        empty_addr = i * size;
      }

      if (pin == fan.pin) {
        addr = i * size;
        found = true;
        break;
      }
      addr += size;
    }

    int save_addr = found ? addr : empty_addr;
    if (save_addr == -1) {
      Serial.println("Fan settings, no empty slot found");
      return;
    }

    // save fan to EEPROM
    Serial.println("Fan settings, saving to EEPROM at address: " + String(save_addr));
    EEPROM.write(save_addr, fan.channel);
    EEPROM.write(save_addr + 1, fan.pin);
    EEPROM.write(save_addr + 2, (uint16_t)fan.freq >> 8);
    EEPROM.write(save_addr + 3, (uint16_t)fan.freq & 0xFF);
    EEPROM.write(save_addr + 4, fan.resolution);
    EEPROM.write(save_addr + 5, fan.pwm);
    EEPROM.write(save_addr + 6, (uint16_t)fan.maxRPM >> 8);
    EEPROM.write(save_addr + 7, (uint16_t)fan.maxRPM & 0xFF);
    EEPROM.commit();
  }

  // /api/data
  void handleApiData(AsyncWebServerRequest *request) {
    // print state
    String json = "";
    json += "{";
    

    json += "\"fans\": [ ";
    for (Fan &fan : this->fans) {
      json += fan.toJSON();
      json += ",";
    }
    json = json.substring(0, json.length() - 1); // remove last comma
    json += "], ";


    json += "\"tacho_inputs\": [ ";
    for (TachoInput &tacho : this->tacho_inputs) {
      json += tacho.toJSON();
      json += ",";
    }
    json = json.substring(0, json.length() - 1); // remove last comma
    json += "], ";


    json += "\"voltage_inputs\": [ ";
    for (VoltageInput &voltage : this->voltage_inputs) {
      json += voltage.toJSON();
      json += ",";
    }
    json = json.substring(0, json.length() - 1); // remove last comma
    json += "]";


    json += "}";
    request->send(200, "application/json", "{\"data\":" + json + "}");
  }

  // /api/fan/set/pwm?pin=3&pwm=255
  void handleApiFanSetPWM(AsyncWebServerRequest *request) {
    if (request->hasParam("pin") && request->hasParam("pwm")) {
      int pin = request->getParam("pin")->value().toInt();
      int pwm = request->getParam("pwm")->value().toInt();
      Serial.println("Setting PWM for pin " + String(pin) + " to " + String(pwm));

      for (Fan &fan : this->fans) {
        if (fan.getPin() == pin) {
          fan.setSpeed(pwm);
          saveFanSettings(fan.getSettings());
        }
      }

      // Respond to the client
      request->send(200, "application/json", "{\"data\":\"ok\"}");
    } else {
      request->send(400, "application/json", "{\"error\":\"missing parameters\"}");
    }
  }

  void handleApiFanSetPWMAll(AsyncWebServerRequest *request) {
    if (request->hasParam("pwm")) {
      int pwm = request->getParam("pwm")->value().toInt();
      for (Fan &fan : this->fans) {
        fan.setSpeed(pwm);
        saveFanSettings(fan.getSettings());
      }
      request->send(200, "application/json", "{\"data\":\"ok\"}");
    } else {
      request->send(400, "application/json", "{\"error\":\"missing parameters\"}");
    }
  }

  void setup(std::vector<FanSettings> fs, std::vector<TachoInputSettings> tis, std::vector<VoltageInputSettings> dfis) {
    Serial.println("--------STARTING---------");
    EEPROM.begin(512);
    loadFans(fs);
    loadTachoInputs(tis);
    loadVoltageInputs(dfis);
    server.on("/api/test", HTTP_GET, [this](AsyncWebServerRequest *request) {
      request->send(200, "application/json", "{\"data\":\"ok\"}");
    });
    server.on("/api/data", HTTP_GET, [this](AsyncWebServerRequest *request) {
      this->handleApiData(request);
    });
    server.on("/api/fan/set/pwm/all", HTTP_GET, [this](AsyncWebServerRequest *request) {
      this->handleApiFanSetPWMAll(request);
    });
    server.on("/api/fan/set/pwm", HTTP_GET, [this](AsyncWebServerRequest *request) {
      this->handleApiFanSetPWM(request);
    });
  };

  unsigned long onceTime = millis();
  
  void loop() {
    unsigned long currentTime = millis();
    // connect to wifi
    if (WiFi.status() != WL_CONNECTED) {
      lastWifiConnect = currentTime;
      if (!isConnectigToWifi) {
        isConnectigToWifi = true;
        WiFi.begin(WIFI_SSID, WIFI_PSW);
      }

      if (currentTime - lastWifiConnect >= 1000) {
        Serial.println("Connecting to wifi......");
        lastWifiConnect = currentTime;
      }
    } else if(isConnectigToWifi) {
      isConnectigToWifi = false;
      Serial.print("Connected to wifi: ");
      Serial.println(WiFi.localIP());
      server.begin(); // safe to start webserver
    }

    // execute once, try to set fan speed, some fan pins does not produce correct pwm signal
    for (Fan &fan : fans) {
      fan.setSpeed(fan.getPWM());
    }
    
    for (TachoInput &tacho : tacho_inputs) {
      tacho.watch();
    }
  }

  void log() {
    currentTimeLog = millis();
    if (currentTimeLog - previousTimeLog >= timeoutTimeLog) {
      previousTimeLog = currentTimeLog;
      Serial.println("-------------------------");
      Serial.println(WiFi.localIP());
      Serial.println("-------------------------");

      for (VoltageInput &voltage : voltage_inputs) {
        String voltage_pin = String(voltage.getPin());
        Serial.println((String) "P:" + voltage_pin + " Voltage: " + String(voltage.getVoltage()));
        Serial.println((String) "P:" + voltage_pin + " Voltage resolution: " + String(voltage.getVoltageResolution(4)));
      }

      for (Fan &fan : fans) {
        String fan_pin = String(fan.getPin());
        Serial.println((String) "P:" + fan_pin + " Fan PWM: " + String(fan.getPWM()));
      }

      for (TachoInput &tacho : tacho_inputs) {
        String tacho_pin = String(tacho.getPin());
        Serial.println((String) "P:" + tacho_pin + " Tacho Pulses: " + String(tacho.getPulses()));
        Serial.println((String) "P:" + tacho_pin + " Tacho RPM: " + String(tacho.getRPM()));
      }
    }
  }
};