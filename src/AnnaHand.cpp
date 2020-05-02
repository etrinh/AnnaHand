// Lolin Wemos D1 R2 flash size 0fs

#define ENABLE_ARDUINOOTA
#define ENABLE_UPDATE

#ifdef ENABLE_ARDUINOOTA
# define NO_GLOBAL_ARDUINOOTA
# include <ArduinoOTA.h>
#endif
#include <WiFiManager.h>  // Need to patch to transfer to cpp some duplicated definitions with WebServer.h, make connectWifi public and change HTTP_HEAD to _HTTP_HEAD
#include "Arduino.h"
#include <WiFiClient.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>

#define SERIAL_DEBUG false               // Enable / Disable log - activer / désactiver le journal

#define TAG "AnnaHand"
#define VERSION   "1.01"

#define MIN(a,b)  ((a)<(b)?(a):(b))
#define MAX(a,b)  ((a)>(b)?(a):(b))

// Web server port - port du serveur web
#define WEB_SERVER_PORT 80
#define URI_WIFI "/reset"
#define URI_REBOOT "/reboot"
#define URI_OTA "/ota"
#define URI_UPDATE "/update"
#define URI_STATUS "/status"
#define URI_ROOT "/"
#define URI_USAGE "/help"
#define URI_INFO "/info"
#define URI_LIGHT "/light"
#define URI_DEFAULT "/default"

ESP8266WebServer server ( WEB_SERVER_PORT );

#define OTA_TIMER (10*60)
#define FLASH_TIMER (10)
#define REBOOT_TIMER (3)
#define OTA_REBOOT_TIMER (1)

int64_t rebootRequested = 0;
int64_t otaOnTimer = 0;

#define PIN_SENSOR          D0  // GPIO16
#define PIN_LED_HAND_LIGHT  D5  // GPIO14
#define PIN_LED_HAND_RGBW_R D4  // GPIO2 (builtin led)
#define PIN_LED_HAND_RGBW_G D3  // GPIO0
#define PIN_LED_HAND_RGBW_B D2  // GPIO4
#define PIN_LED_HAND_RGBW_W D1  // GPIO5

class Light
{
  public:
    enum { EEPROM_SIZE = 64 };
    Light(String name, int pin, int baseEEPROM): m_name(name), m_pin(pin), m_baseAddr(baseEEPROM)
    {
      pinMode(pin, OUTPUT);
      // read EEPROM
      if (EEPROM.read(m_baseAddr) != 0) {
        EEPROM.put<unsigned char>(m_baseAddr, 0);
        EEPROM.put<unsigned short>(m_baseAddr+sizeof(unsigned char), m_defaultValue);
        EEPROM.put<unsigned short>(m_baseAddr+2*sizeof(unsigned char), m_defaultRampOn);
        EEPROM.put<unsigned short>(m_baseAddr+2*sizeof(unsigned char)+sizeof(unsigned short), m_defaultRampOff);
        EEPROM.put<unsigned int>(m_baseAddr+2*sizeof(unsigned char)+2*sizeof(unsigned short), m_defaultDelay);
        EEPROM.commit();
      }
      EEPROM.get<unsigned short>(m_baseAddr+sizeof(unsigned char), m_defaultValue);
      EEPROM.get<unsigned short>(m_baseAddr+2*sizeof(unsigned char), m_defaultRampOn);
      EEPROM.get<unsigned short>(m_baseAddr+2*sizeof(unsigned char)+sizeof(unsigned short), m_defaultRampOff);
      EEPROM.get<unsigned int>(m_baseAddr+2*sizeof(unsigned char)+2*sizeof(unsigned short), m_defaultDelay);
    }
    String name() const { return m_name; }
    int currentValue() const { return m_currentValue; }
    int currentProgression() const { return m_ramp>0?m_currentProgression*100/m_ramp:100; }
    int currentTarget() const { return m_targetValue; }
    void setDefault(unsigned short value) { m_defaultValue = value; EEPROM.put<unsigned short>(m_baseAddr+sizeof(unsigned char), value); }
    void setDefaultRampOn(unsigned short rampOn) { m_defaultRampOn = rampOn; EEPROM.put<unsigned short>(m_baseAddr+2*sizeof(unsigned char), rampOn); }
    void setDefaultRampOff(unsigned short rampOff) { m_defaultRampOff = rampOff; EEPROM.put<unsigned short>(m_baseAddr+2*sizeof(unsigned char)+sizeof(unsigned short), rampOff); }
    void setDefaultDelay(unsigned int value) { m_defaultDelay = value; EEPROM.put<unsigned int>(m_baseAddr+2*sizeof(unsigned char)+2*sizeof(unsigned short), value); }
    void defaultValues(unsigned short &value, unsigned short &rampOn, unsigned short &rampOff, unsigned int &delay)
    {
      value = m_defaultValue;
      rampOn = m_defaultRampOn;
      rampOff = m_defaultRampOff;
      delay = m_defaultDelay;
    }
    void setDimming(bool on, int ramp = -1) {
      if (on) setDimming(m_defaultValue, ramp);
      else setDimming((unsigned short)0, ramp);
    }
    void setDimming(unsigned short value, int ramp = -1)
    {
      m_originalValue = m_currentValue;
      m_targetValue = value;
      m_ramp = ramp>=0?ramp:(value?m_defaultRampOn:m_defaultRampOff);
      m_lastUpdate = millis();
      m_delayTimeout = m_lastUpdate + m_defaultDelay;
    }
    void update()
    {
      unsigned long currentUpdate = millis();
      if (m_defaultDelay > 0 && m_targetValue > 0 && m_delayTimeout > currentUpdate) {
        setDimming(false);
      }
      if (m_currentValue != m_targetValue) {
        if (m_ramp > 0) {
          m_currentProgression += ((int64_t)currentUpdate - m_lastUpdate);
          if (m_currentProgression >= m_ramp) {
            m_currentValue = m_targetValue;
            m_ramp = 0;
            m_lastUpdate = 0;
            m_currentProgression = 0;
          }
          else {
            m_currentValue = m_originalValue + ((int64_t)m_targetValue - m_originalValue) * MIN(m_ramp, m_currentProgression) / (m_ramp);          
            m_lastUpdate = currentUpdate;
          }
        }
        else {
          m_currentValue = m_targetValue;
        }
        analogWrite(m_pin, PWMRANGE * m_currentValue / 256);
      }
    }
  protected:
    String m_name;
    int m_pin;
    int m_baseAddr;
    unsigned short m_currentValue = 0;
    unsigned short m_originalValue = 0;
    unsigned short m_targetValue = 0;
    unsigned short m_ramp = 0;
    unsigned long m_currentProgression = 0;
    unsigned long m_lastUpdate = 0;
    unsigned long m_delayTimeout = 0;
    unsigned short m_defaultValue = 255;
    unsigned short m_defaultRampOn = 3000;
    unsigned short m_defaultRampOff = 3000;
    unsigned int m_defaultDelay = 0;
};
static std::vector<Light> Lights;

static void default_handler() {
  EEPROM.begin(Lights.size() * Light::EEPROM_SIZE);
  for (Light& l : Lights) {
    String arg;
    if ((arg = server.arg(l.name())) != "") {
      l.setDefault(arg.toInt());
    }
    if ((arg = server.arg(l.name()+"_rampOn")) != "") {
      l.setDefaultRampOn(arg.toInt());
    }
    if ((arg = server.arg(l.name()+"_rampOn")) != "") {
      l.setDefaultRampOff(arg.toInt());
    }
    if ((arg = server.arg(l.name()+"_delay")) != "") {
      l.setDefaultDelay(arg.toInt());
    }
  }
  if (server.arg("all") == "current") {
    for (Light& l : Lights) {
      l.setDefault(l.currentValue());
    }
  }

  EEPROM.commit();
  EEPROM.end();
  server.send(200);
}

static int hex2int(String hex)
{
  int result = 0;
  for (unsigned int i = 0; i < hex.length(); ++i) {
    unsigned char ch = hex.charAt(i);
    int num;
    switch(ch) {
      case 'A': case 'a': num=10; break;
      case 'B': case 'b': num=11; break;
      case 'C': case 'c': num=12; break;
      case 'D': case 'd': num=13; break;
      case 'E': case 'e': num=14; break;
      case 'F': case 'f': num=15; break;
      default: num=(ch>='0' && ch<='9')?ch-'0':0; break;
    }
    result = result * 16 + num;
  }
  return result;
}

static String int2hex(int value, unsigned int fillWidth = 2)
{

  String result(value, HEX);
  while (result.length() < fillWidth) result = "0" + result;
  return result;
}

static void light_handler() {
  int ramp = server.hasArg("ramp")?server.arg("ramp").toInt():-1;
  for (Light& l : Lights) {
    String param = server.arg(l.name());
    if (param == "") continue;
    else if (param == "on") {
      l.setDimming(true, ramp);
    }
    else if (param == "off") {
      l.setDimming(false, ramp);
    }
    else if (param == "toggle") {
      l.setDimming(l.currentValue()?false:true, ramp);
    }
    else {
      l.setDimming((unsigned short)param.toInt(), ramp);
    }
  }
  String param = server.arg("all");
  if (param != "") {
    if (param == "toggle") {
      bool isOn = Lights[0].currentValue() || Lights[1].currentValue() || Lights[2].currentValue() || Lights[3].currentValue() || Lights[4].currentValue();
      Lights[0].setDimming(isOn?false:true, ramp);
      Lights[1].setDimming(isOn?false:true, ramp);
      Lights[2].setDimming(isOn?false:true, ramp);
      Lights[3].setDimming(isOn?false:true, ramp);
      Lights[4].setDimming(isOn?false:true, ramp);
    }
    else if (param == "on") {
      Lights[0].setDimming(true, ramp);
      Lights[1].setDimming(true, ramp);
      Lights[2].setDimming(true, ramp);
      Lights[3].setDimming(true, ramp);
      Lights[4].setDimming(true, ramp);
    }
    else if (param == "off") {
      Lights[0].setDimming(false, ramp);
      Lights[1].setDimming(false, ramp);
      Lights[2].setDimming(false, ramp);
      Lights[3].setDimming(false, ramp);
      Lights[4].setDimming(false, ramp);
    }
    else if (param.startsWith("#")) {
      Lights[0].setDimming((unsigned short)(hex2int(param.substring(1,3))), ramp);
      Lights[1].setDimming((unsigned short)(hex2int(param.substring(3,5))), ramp);
      Lights[2].setDimming((unsigned short)(hex2int(param.substring(5,7))), ramp);
      Lights[3].setDimming((unsigned short)(hex2int(param.substring(7,9))), ramp);
      Lights[4].setDimming((unsigned short)(hex2int(param.substring(9,11))), ramp);
    }
  }
  param = server.arg("rgbw");
  if (param != "") {
    if (param == "toggle") {
      bool isOn = Lights[1].currentValue() || Lights[2].currentValue() || Lights[3].currentValue() || Lights[4].currentValue();
      Lights[1].setDimming(isOn?false:true, ramp);
      Lights[2].setDimming(isOn?false:true, ramp);
      Lights[3].setDimming(isOn?false:true, ramp);
      Lights[4].setDimming(isOn?false:true, ramp);
    }
    else if (param == "on") {
      Lights[1].setDimming(true, ramp);
      Lights[2].setDimming(true, ramp);
      Lights[3].setDimming(true, ramp);
      Lights[4].setDimming(true, ramp);
    }
    else if (param == "off") {
      Lights[1].setDimming(false, ramp);
      Lights[2].setDimming(false, ramp);
      Lights[3].setDimming(false, ramp);
      Lights[4].setDimming(false, ramp);
    }
    else if (param.startsWith("#")) {
      Lights[1].setDimming((unsigned short)(hex2int(param.substring(1,3))), ramp);
      Lights[2].setDimming((unsigned short)(hex2int(param.substring(3,5))), ramp);
      Lights[3].setDimming((unsigned short)(hex2int(param.substring(5,7))), ramp);
      Lights[4].setDimming((unsigned short)(hex2int(param.substring(7,9))), ramp);
    }
  }
  param = server.arg("rgb");
  if (param != "") {
    if (param == "toggle") {
      bool isOn = Lights[1].currentValue() || Lights[2].currentValue() || Lights[3].currentValue();
      Lights[1].setDimming(isOn?false:true, ramp);
      Lights[2].setDimming(isOn?false:true, ramp);
      Lights[3].setDimming(isOn?false:true, ramp);
    }
    else if (param == "on") {
      Lights[1].setDimming(true, ramp);
      Lights[2].setDimming(true, ramp);
      Lights[3].setDimming(true, ramp);
    }
    else if (param == "off") {
      Lights[1].setDimming(false, ramp);
      Lights[2].setDimming(false, ramp);
      Lights[3].setDimming(false, ramp);
    }
    else if (param.startsWith("#")) {
      Lights[1].setDimming((unsigned short)(hex2int(param.substring(1,3))), ramp);
      Lights[2].setDimming((unsigned short)(hex2int(param.substring(3,5))), ramp);
      Lights[3].setDimming((unsigned short)(hex2int(param.substring(5,7))), ramp);
    }
  }
  server.send(200);
}

static void requestReboot(int timer = REBOOT_TIMER)
{
  if (timer == 0) {
    ESP.restart();
  }
  else {
    rebootRequested = millis() + timer * 1000;    
  }
}

static void wifi_handler() {
  server.send(200);
  system_restore();
  requestReboot(0);
}

static void reboot_handler() {
  requestReboot();
  server.send(200);
}

#ifdef ENABLE_ARDUINOOTA
ArduinoOTAClass * OTA = NULL;
static void enableOTA(bool enable, bool forceCommit = false)
{
  if (enable) {
    if (OTA == NULL) {
      OTA = new ArduinoOTAClass();
      OTA->setHostname(TAG);
      OTA->setPasswordHash("913f9c49dcb544e2087cee284f4a00b7");   // MD5("device")
      OTA->begin();
      int64_t fr_start = millis();
      otaOnTimer = fr_start + OTA_TIMER * 1000; // 10 minutes
    }
  }
  else {
    if (OTA) {
      if (forceCommit) {
        delete OTA;
        OTA = NULL;
        otaOnTimer = 0;
      }
      else {
        otaOnTimer = 1; // to be disabled on next loop
      }
    }
  }
}
static void ota_handler() {
  if (server.arg("action") == "on") {
    enableOTA(true);
  }
  else if (server.arg("action") == "off") {
    enableOTA(false);
  }
  else if (server.arg("action") == "toggle") {
    enableOTA(OTA == NULL);
  }
  server.send(200);
}
#endif

#ifdef ENABLE_UPDATE
static void update_handler() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.setDebugOutput(true);
    WiFiUDP::stopAll();
    Serial.printf("Update: %s\n", upload.filename.c_str());
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace)) { //start with max available size
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { //true to set the size to the current progress
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
    Serial.setDebugOutput(false);
  }
  yield();
}
#endif

static void usage_handler() {
  String info =
    "<html>"
      "<head>"
        "<title>" TAG " - API</title>"
        "<style>"
        "</style>"
      "</head>"
      "<body>"
        "<h1>" TAG "<span id=\"version\"> (v" VERSION ") - API Usage</span></h1>"
          "<ul>"
            "<li>Reboot: " URI_REBOOT "</li>"
            "<li>Reset Wifi: " URI_WIFI "</li>"
#ifdef ENABLE_ARDUINOOTA
            "<li>OTA on/off/toggle: " URI_OTA "?action=[on|off|toggle]</li>"
#endif
            "<li>LED on/off/toggle/value (0-255): " URI_LIGHT "?([bulb|red|green|blue|white|rgbw|all]=[on|off|toggle]&ramp=[0-9]*)+</li>"
            "<li>LED set default values (value (0-255)- rampOn - rampOff: " URI_DEFAULT "?([bulb|red|green|blue][|_rampOn|_rampOff|_delay]=[0-9]*)+|all=current</li>"
            "<li>Status (JSON): " URI_STATUS "</li>"
        "</ul>"
      "</body>"
    "</html>";
  server.send(200, "text/html", info);
}

static void status_handler() {
  String lights;
  for (size_t i = 0; i < Lights.size(); ++i) {
    unsigned short defaultValue, defaultRampOn, defaultRampOff; unsigned int defaultDelay;
    Lights[i].defaultValues(defaultValue, defaultRampOn, defaultRampOff, defaultDelay);
    lights+= "\"" + Lights[i].name() + "\":{\"value\":"+String(Lights[i].currentValue()) + ",\"default\":"+String(defaultValue) + ",\"rampOn\":"+String(defaultRampOn) + ",\"rampOff\":"+String(defaultRampOff) + ",\"delay\":"+String(defaultDelay) + "},";
  }
  lights+= "\"rgb\":{\"value\":\"#"+int2hex(Lights[1].currentValue())+int2hex(Lights[2].currentValue())+int2hex(Lights[3].currentValue())+"\"},";
  lights+= "\"rgbw\":{\"value\":\"#"+int2hex(Lights[1].currentValue())+int2hex(Lights[2].currentValue())+int2hex(Lights[3].currentValue())+int2hex(Lights[4].currentValue())+"\"},";
  lights+= "\"Lrgbw\":{\"value\":\"#"+int2hex(Lights[0].currentValue())+int2hex(Lights[1].currentValue())+int2hex(Lights[2].currentValue())+int2hex(Lights[3].currentValue())+int2hex(Lights[4].currentValue())+"\"}";
  unsigned long currentTimer = millis();
  String info = "{"
                "\"version\":\"" VERSION "\","
                "\"ssid\":\"" + WiFi.SSID() + "\","
                "\"rssi\":\"" + String(WiFi.RSSI()) + "\","
                "\"ip\":\"" + WiFi.localIP().toString() + "\","
                "\"mac\":\"" + WiFi.macAddress() + "\","
                "\"chipId\":\"" + ESP.getChipId() + "\","
                "\"lights\":{" + lights + "},"
#ifdef ENABLE_ARDUINOOTA
                "\"ota\":\"" + String(OTA ? "true" : "false") + "\","
                "\"otaTimer\":" + String(MAX(0, int((otaOnTimer - currentTimer) / 1000))) + ","
#endif
                "\"reboot\":\"" + String(rebootRequested > 0 ? "true" : "false") + "\","
                "\"rebootTimer\":" + String(MAX(0, int((rebootRequested - currentTimer) / 1000))) + ""
                "}";
  server.send(200, "application/json", info);
}

static void info_handler() {
  static const __FlashStringHelper* info =
    F("<html>"
      "<head>"
        "<title>" TAG "</title>"
        "<script type=\"text/javascript\">"
          "function invoke(url)"
          "{"
            "var xhr = new XMLHttpRequest();"
            "xhr.open(\"GET\", url, true);"
            "xhr.send(null);"
          "};"
          "function update()"
          "{"
            "var xhr = new XMLHttpRequest();"
            "xhr.open(\"GET\", \"" URI_STATUS "\", true);"
            "xhr.onload = function (e) {"
              "if (xhr.readyState === 4) {"
                "if (xhr.status === 200) {"
                  "var obj = JSON.parse(xhr.responseText);"
                  "document.getElementById(\"ssid\").innerHTML = obj.ssid;"
                  "document.getElementById(\"rssi\").innerHTML = obj.rssi;"
                  "document.getElementById(\"ip\").innerHTML = obj.ip;"
                  "document.getElementById(\"mac\").innerHTML = obj.mac;"
                  "document.getElementById(\"bulb\").value = obj.lights.bulb.value;"
                  "document.getElementById(\"lights\").innerHTML = obj.lights.Lrgbw.value;"
                  "document.getElementById(\"white\").value= obj.lights.white.value;"
                  "document.getElementById(\"rgb\").value = obj.lights.rgb.value;"
                  "document.getElementById(\"reboot\").innerHTML = obj.rebootTimer>0?\" - \"+obj.rebootTimer:\"\";"
#ifdef ENABLE_ARDUINOOTA
                  "document.getElementById(\"ota\").innerHTML = obj.ota==\"true\"?\" - On (\"+Math.round(obj.otaTimer/60)+\"min)\":\" - Off\";"
#endif
                "}"
              "}"
            "};"
            "xhr.send(null);"
          "};"
          "update();"
          "setInterval(update, 1000);"
        "</script>"
      "</head>"
      "<body>"
        "<h1>" TAG "<span id=\"version\"> (v" VERSION ")</span></h1>"
          "<table style=\"height: 60px;\" width=\"100%\">"
          "<tbody>"
            "<tr>"
              "<td style=\"width: 50%;\">"
                "<span class=\"info\">SSID: </span><span id=\"ssid\"></span>"
                "<br/>"
                "<span class=\"info\">RSSI: </span><span id=\"rssi\"></span>"
                "<br/>"
                "<span class=\"info\">IP: </span><span id=\"ip\"></span>"
                "<br/>"
                "<span class=\"info\">MAC: </span><span id=\"mac\"></span>"
                "<br/>"
                "<span class=\"info\">Lights: </span><span id=\"lights\"></span><input type=\"range\" id=\"bulb\" min=\"0\" max=\"255\" onchange=\"invoke('" URI_LIGHT "?bulb='+this.value)\"/><input type=\"color\" id=\"rgb\" onchange=\"invoke('" URI_LIGHT "?rgb='+this.value.replace('#','%23'))\"/><input type=\"range\" id=\"white\" min=\"0\" max=\"255\"/ onchange=\"invoke('" URI_LIGHT "?white='+this.value)\"><a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_DEFAULT "?all=current\');return false;\">Memorize</a>"
              "</td>"
            "</tr>"
            "<tr>"
              "<td style=\"width: 50%;\">"
                "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_LIGHT "?all=toggle\');return false;\">Toggle Lights</a></span>"
                "<br/>"
                "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_REBOOT "\');return false;\">Reboot Device</a><span id=\"reboot\"></span>"
                "<br/>"
                "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_WIFI "\');return false;\">Reset Device</a>"
                "<br/>"
#ifdef ENABLE_ARDUINOOTA
                "<a class=\"link\"  href=\"\" onclick=\"invoke(\'" URI_OTA "?action=toggle\');return false;\">Toggle OTA</a><span id=\"ota\"></span>"
                "<br/>"
#endif
#ifdef ENABLE_UPDATE
                "<form id=\"upgradeForm\" method=\"post\" enctype=\"multipart/form-data\" action=\"" URI_UPDATE "\"><span class=\"action\">Upgrade Firmware: </span><input type=\"file\" name=\"fileToUpload\" id=\"upgradeFile\" /><input type=\"submit\" value=\"Upgrade\" id=\"upgradeSubmit\"/></form>"
                "<br/>"
#endif
              "</td>"
            "</tr>"
          "</tbody>"
        "</table>"
        "<a href=\"" URI_USAGE "\">API Usage</a>"
      "</body>"
    "</html>");
  server.send(200, "text/html", info);
}

static void startServer() {
  server.on ( URI_ROOT, info_handler );
  server.on ( URI_INFO, info_handler );
  server.on ( URI_USAGE, usage_handler );
  server.on ( URI_STATUS, status_handler );
  server.on ( URI_WIFI, wifi_handler );
  server.on ( URI_REBOOT, reboot_handler );
  server.on ( URI_LIGHT, light_handler );
  server.on ( URI_DEFAULT, default_handler );
#ifdef ENABLE_ARDUINOOTA
  server.on ( URI_OTA, ota_handler );
#endif
#ifdef ENABLE_UPDATE
  server.on ( URI_UPDATE, HTTP_POST, []() {
        String html = "<html>"
                        "<head>"
                        "<title>" TAG " - OTA</title>" +
                        (!Update.hasError() ? "<meta http-equiv=\"refresh\" content=\"" + String(OTA_REBOOT_TIMER + 1) + "; url=/\">" : "") +
                        "</head>"
                        "<body>Update " + (Update.hasError() ? "failed" : "succeeded") + "</body>"
                    "</html>";
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", html);
    requestReboot();
  }, update_handler );
#endif
  server.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(SERIAL_DEBUG);

  // Wi-Fi connection - Connecte le module au réseau Wi-Fi
  // attempt to connect; should it fail, fall back to AP
  WiFiManager().autoConnect(TAG + ESP.getChipId(), "");

  // Setup PWM mode and Sensor Pin
  analogWriteRange(PWMRANGE);
  analogWriteFreq(1000);
  pinMode(PIN_SENSOR, PIN_SENSOR==16?INPUT_PULLDOWN_16:INPUT);

  EEPROM.begin(5 * Light::EEPROM_SIZE);
  Lights.push_back(Light("bulb", PIN_LED_HAND_LIGHT, 0));
  Lights.push_back(Light("red", PIN_LED_HAND_RGBW_R, 64));
  Lights.push_back(Light("green", PIN_LED_HAND_RGBW_G, 128));
  Lights.push_back(Light("blue", PIN_LED_HAND_RGBW_B, 192));
  Lights.push_back(Light("white", PIN_LED_HAND_RGBW_W, 256));
  EEPROM.end();

  for (Light& l : Lights) l.setDimming(true);
  startServer();
}

void loop() {
  server.handleClient();
  unsigned long currentTime = millis();
  if (rebootRequested != 0 && currentTime > rebootRequested) {
    rebootRequested = 0;
    requestReboot(0);
  }
#ifdef ENABLE_ARDUINOOTA
  if (OTA)  OTA->handle();
  if (otaOnTimer != 0 && currentTime > otaOnTimer) {
    enableOTA(false, true);
  }
#endif
  // read sensor
  static bool lastState = false;
  bool sensorState = digitalRead(PIN_SENSOR);
  if (sensorState != lastState) {
    for (Light& l : Lights) l.setDimming(sensorState);
    lastState = sensorState;
  }
  for (Light& l : Lights) l.update();
  delay(50);
}
