#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "FS.h"
#include <LittleFS.h>

#include <AsyncJson.h>
#include <ArduinoJson.h>

#include <debounce.h>
//#include <RotaryEncoder.h>

//#define ROTARY_ENCODER_A_PIN 23
//#define ROTARY_ENCODER_B_PIN 22
//#define ROTARY_ENCODER_BUTTON_PIN 21
//#define ROTARY_ENCODER_STEPS 4

#define FLAP1_HALL 36

//pins 34,35,36,39 have no internal pullup
#define BUTTON_PIN 39
hw_timer_t *ButtonCheck_timer = NULL;
const int BUTTON_CHECK_INTERVAL = 5000;  //200hz
bool ledButtonHoldReleased = false;

#define BUTTON_LED_PIN 32
#define BUTTON_LED_CHANNEL 1
hw_timer_t *Led_timer = NULL;
const int LED_BLINK_INTERVAL = 6000;  //10000 = 100hz
int LedButtonDutyCycleMax = 255;
int LedButtonDutyCycle = 0;
bool LedButtonFadeFlip = false;

#define ONBOARD_LED 2

//#define MOTOR_DIR_PIN 12
#define MOTOR_STEP_PIN 13

hw_timer_t *Motor_timer = NULL;
const int MOTOR_SPEED_INITIAL = 1000;
int MOTOR_SPEED = MOTOR_SPEED_INITIAL;
const int MOTOR_INCREMENT = 30;

//RotaryEncoder *encoder = nullptr;

//WIFI
const char* ssid = "wifissid";
const char* password = "wifipassword";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile long motorMoveDistance = 0;
bool motorGoingHome = false;
long motorDestination = 0;

unsigned int rpmCheckRunningStep = 0;
unsigned long rpmCheckStartTime = 0;

void IRAM_ATTR runMotor() {

  if (digitalRead(MOTOR_STEP_PIN)) {
    digitalWrite(MOTOR_STEP_PIN, LOW);
    //Serial.print("mL-");
    //Serial.println(motorMoveDistance);
  } else {
    if (motorMoveDistance < 0) {
      //digitalWrite(MOTOR_DIR_PIN,HIGH);
      digitalWrite(MOTOR_STEP_PIN, HIGH);
      motorMoveDistance++;
      if (motorMoveDistance == 0) {
        digitalWrite(ONBOARD_LED, LOW);
        //encoder->setPosition(0);
      }
    } else if (motorMoveDistance > 0) {
      //digitalWrite(MOTOR_DIR_PIN,LOW);
      digitalWrite(MOTOR_STEP_PIN, HIGH);
      motorMoveDistance--;
      if (motorMoveDistance == 0) {
        digitalWrite(ONBOARD_LED, LOW);
        //encoder->setPosition(0);
      }
    }
  }
}

static void ledButtonPress(uint8_t btnId, uint8_t btnState) {
  //Serial.print("ledButtonPress btnState ");
  //Serial.println(btnState);

  // if (btnState == BTN_PRESSED) {
  //   Serial.println("ledButtonPress Pressed");
  //   digitalWrite(ONBOARD_LED,!digitalRead(ONBOARD_LED));
  //   motorMoveDistance=0;
  //   //encoder->setPosition(0);
  //   ws.textAll("ledButtonPress Pressed");
  // }
  if(btnState == BTN_OPEN){
    if (ledButtonHoldReleased) {
      ledButtonHoldReleased = false;
    }else{
      digitalWrite(ONBOARD_LED,!digitalRead(ONBOARD_LED));
      motorMoveDistance = (random(80) * 28) + 2240;
      //Serial.println("ledButtonPress Released");
      //ws.textAll("ledButtonPress Released");
    }
  }
}
static void ledButtonHold(uint8_t btnId, uint8_t btnState) {
  //Serial.print("ledButtonHold btnState ");
  //Serial.println(btnState);

  // if (btnState == BTN_PRESSED) {
  //   Serial.println("ledButtonHold Pressed");
  //   digitalWrite(ONBOARD_LED,!digitalRead(ONBOARD_LED));
  //   motorMoveDistance=0;
  //   //encoder->setPosition(0);
  //   ws.textAll("ledButtonHold Pressed");
  // }
  
  if(btnState == BTN_OPEN){
    motorhome(0);

    ledButtonHoldReleased = true;
    digitalWrite(ONBOARD_LED,!digitalRead(ONBOARD_LED));
    //Serial.println("ledButtonHold Released");
    //ws.textAll("ledButtonHold Released");
  }
}

static void hallHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    //Serial.println("Hall Sensor!");
    ws.textAll("Hall Sensor!");
    if (motorGoingHome) {
      motorMoveDistance = 0;
      motorGoingHome = false;
      ws.textAll("Motor is Home!");
      motorMoveDistance = motorDestination;
      motorDestination = 0;

      switch (rpmCheckRunningStep) {
        case 1:
          rpmCheckStartTime = millis();
          motorhome(0);
          rpmCheckRunningStep = 2;
          break;
        case 2:
          rpmCheckRunningStep = 0;
          unsigned long elapsed = millis() - rpmCheckStartTime;
          unsigned long rpm = 60 / (elapsed / 1000);

          const int capacity = JSON_OBJECT_SIZE(20);
          StaticJsonDocument<capacity> json;

          char rptText[25];
          sprintf(rptText, "Motor Rotation Time %d", elapsed);
          json["time"] = elapsed;

          char rpmText[25];
          sprintf(rpmText, "Motor Rotation RPM %d", rpm);
          json["rpm"] = rpm;

          ws.textAll(rptText);
          ws.textAll(rpmText);

          char jsonOutput[128];
          serializeJson(json, jsonOutput);
          Serial.println(jsonOutput);
          ws.textAll(jsonOutput);
          break;
      }
    }
  } else {
    ws.textAll("Released Hall Sensor!");
    Serial.println("Released Hall Sensor!");
  }
}

static Button flap1_hall(0, hallHandler);
static Button LedButtonDebounceHold(1, ledButtonHold);
static Button LedButtonDebouncePress(2, ledButtonPress);

void IRAM_ATTR runButtonCheckTimer() {

  flap1_hall.update(digitalRead(FLAP1_HALL));

  //myButton.update(digitalRead(ROTARY_ENCODER_BUTTON_PIN));

  uint8_t ledButton = digitalRead(BUTTON_PIN);
  LedButtonDebounceHold.update(ledButton);
  LedButtonDebouncePress.update(ledButton);
}

void IRAM_ATTR runLedTimer() {
  //increment led counter fade
  if (LedButtonFadeFlip) {
    if (LedButtonDutyCycle > 0) {
      ledcWrite(BUTTON_LED_CHANNEL, LedButtonDutyCycle);
      LedButtonDutyCycle--;
    } else {
      //LedButtonDutyCycle = 0;
      LedButtonFadeFlip = false;
    }
  } else {
    if (LedButtonDutyCycle <= LedButtonDutyCycleMax) {
      ledcWrite(BUTTON_LED_CHANNEL, LedButtonDutyCycle);
      LedButtonDutyCycle++;
    } else {
      //LedButtonDutyCycle = 255;
      LedButtonFadeFlip = true;
    }
  }
  //Serial.print(LedButtonDutyCycle);
}
//void IRAM_ATTR checkPosition() {
//  encoder->tick(); // just call tick() to check the state.
//}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}


void handleClientJsonData(String data) {
  StaticJsonDocument<200> decodedJson;

  DeserializationError errorDecode = deserializeJson(decodedJson, data);
  // Test if parsing succeeds.
  if (errorDecode) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(errorDecode.c_str());
    return;
  }
  //JsonObject decodedJsonObject = decodedJsonString.to<JsonObject>();
  JsonVariant jsonCmd = decodedJson["cmd"];
  if (jsonCmd.isNull()) {
    Serial.println("cmd not found");
    return;
  }
  Serial.println("json cmd received!");
  String cmdString = jsonCmd.as<String>();

  Serial.println(cmdString);

  if (cmdString == "ledtoggle") {
    digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
    return;
  }
  if (cmdString == "motorstep10") {
    motorMoveDistance += 10;
    return;
  }
  if (cmdString == "motorstep20") {
    motorMoveDistance += 20;
    return;
  }
  if (cmdString == "motorhome") {
    motorhome(0);
    return;
  }
  if (cmdString == "devil") {
    motorhome(44 * 5);
    return;
  }

  if (cmdString == "goto") {
    JsonVariant jsonValue = decodedJson["value"];
    if (jsonValue.isNull()) {
      Serial.println("goto value not found");
      return;
    }
    int destinationValue = jsonValue.as<unsigned int>();
    Serial.print("goto ");
    Serial.println(destinationValue);
    motorhome(destinationValue);
    return;
  }
  if (cmdString == "movesteps") {
    JsonVariant jsonValue = decodedJson["value"];
    if (jsonValue.isNull()) {
      Serial.println("movesteps value not found");
      return;
    }
    int steps = jsonValue.as<unsigned int>();
    Serial.print("movesteps ");
    Serial.println(steps);
    motorMoveDistance = steps;
    return;
  }
  if (cmdString == "speedinc") {
    motorspeedAdd(10);
    return;
  }
  if (cmdString == "speeddec") {
    motorspeedAdd(-10);
    return;
  }
  if (cmdString == "speedreset") {
    MOTOR_SPEED = MOTOR_SPEED_INITIAL;
    motorspeedAdd(0);
    return;
  }
  if (cmdString == "rpmcheck") {
    rpmcheck();
    return;
  }
  if (cmdString == "stop") {
    stopMotor();
    return;
  }
}

void stopMotor() {
  motorGoingHome = false;
  rpmCheckRunningStep = 0;
  motorMoveDistance = 0;
  motorDestination = 0;
}
void rpmcheck() {
  rpmCheckRunningStep = 1;
  motorhome(0);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    //client connected
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    //client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if (type == WS_EVT_DISCONNECT) {
    //client disconnected
    Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if (type == WS_EVT_ERROR) {
    //error was received from the other end
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
  } else if (type == WS_EVT_PONG) {
    //pong message was received (in response to a ping request maybe)
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
  } else if (type == WS_EVT_DATA) {
    //data packet
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len) {
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);
      String msg = "";
      if (info->opcode == WS_TEXT) {
        data[len] = 0;
        Serial.printf("%s\n", (char *)data);
        msg = (char *)data;
      } else {
        for (size_t i = 0; i < info->len; i++) {
          Serial.printf("%02x ", data[i]);
        }
        msg = String((char *)data);
        Serial.print("\n");
        Serial.print(msg);
        Serial.print("\n");
      }
      if (info->opcode == WS_TEXT) {
        //client->text("I got your text message");
        handleClientJsonData(msg);
      } else {
        client->binary("I got your binary message");
      }
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if (info->index == 0) {
        if (info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

      String msg = "";
      if (info->message_opcode == WS_TEXT) {
        data[len] = 0;
        Serial.printf("%s\n", (char *)data);
        msg.concat((char *)data);
      } else {
        for (size_t i = 0; i < len; i++) {
          Serial.printf("%02x ", data[i]);
        }
        Serial.printf("\n");
      }

      if ((info->index + len) == info->len) {
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if (info->final) {
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
          if (info->message_opcode == WS_TEXT) {
            //client->text("I got your text message");
            handleClientJsonData(msg);
          }
          //else{
          //client->binary("I got your binary message");
          //}
        }
      }
    }
  }
}
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void motorspeedAdd(int motorSpeedChange) {
  MOTOR_SPEED = MOTOR_SPEED + motorSpeedChange;
  timerAlarmWrite(Motor_timer, MOTOR_SPEED, true);

  char speedText[25];
  sprintf(speedText, "Motor Speed %d", MOTOR_SPEED);
  ws.textAll(speedText);
}

void motorhome(long destination) {
  motorDestination = destination;
  motorGoingHome = true;
}
void InitOta() {

  // Port defaults to 3232
  //ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("tarotthing");

  // No authentication by default
  //ArduinoOTA.setPassword("password");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("OTA initialized");
}

void setup() {

  pinMode(ONBOARD_LED, OUTPUT);

  pinMode(FLAP1_HALL, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  LedButtonDebounceHold.setPushDebounceInterval(3000);
  LedButtonDebouncePress.setPushDebounceInterval(25);


  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  //  encoder = new RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  //  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), checkPosition, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), checkPosition, CHANGE);

  //  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  //  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  //  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);

  pinMode(MOTOR_STEP_PIN, OUTPUT);
  //pinMode(MOTOR_DIR_PIN, OUTPUT);

  digitalWrite(MOTOR_STEP_PIN, LOW);
  //digitalWrite(MOTOR_DIR_PIN,HIGH);

  Serial.begin(115200);
  Serial.println("Booting");

  //pinMode(BUTTON_LED_PIN, OUTPUT);
  ledcSetup(BUTTON_LED_CHANNEL, 120, 8);  // 50hz
  ledcAttachPin(BUTTON_LED_PIN, BUTTON_LED_CHANNEL);


  ButtonCheck_timer = timerBegin(2, 80, true);  //240mhz / 80 = 3 mhz
  timerAttachInterrupt(ButtonCheck_timer, &runButtonCheckTimer, true);
  timerAlarmWrite(ButtonCheck_timer, BUTTON_CHECK_INTERVAL, true);  // BUTTON_CHECK_INTERVAL = 20000 = 200hz

  Led_timer = timerBegin(1, 80, true);  //240mhz / 80 = 3 mhz
  timerAttachInterrupt(Led_timer, &runLedTimer, true);
  timerAlarmWrite(Led_timer, LED_BLINK_INTERVAL, true);  // LED_BLINK_INTERVAL = 10000 = 100hz


  Motor_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(Motor_timer, &runMotor, true);
  timerAlarmWrite(Motor_timer, MOTOR_SPEED_INITIAL, true);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   Serial.println("Connection Failed! Rebooting...");
  //   delay(5000);
  //   ESP.restart();
  // }
  WiFi.waitForConnectResult();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  InitOta();
  initWebSocket();

  if (!LittleFS.begin()) {
    Serial.println("An Error has occurred while mounting LittleFS");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
  server.onNotFound(notFound);
  server.begin();

  //ONBOARD_LED on when boot finished
  digitalWrite(ONBOARD_LED, HIGH);

  timerAlarmEnable(ButtonCheck_timer);
  timerAlarmEnable(Motor_timer);
  timerAlarmEnable(Led_timer);
}

void loop() {

  ArduinoOTA.handle();

  static int pos = 0;
  //  encoder->tick();

  if (motorGoingHome) {
    motorMoveDistance = 100;
  }

  // int newPos = encoder->getPosition();
  // if (pos != newPos && newPos != 0) {
  //   RotaryEncoder::Direction dir = encoder->getDirection();
  //   // Serial.print("pos:");
  //   // Serial.print(pos);
  //   // Serial.print(" newPos:");
  //   // Serial.print(newPos);
  //   // Serial.print(" dir:");
  //   // Serial.println((int)dir);

  //   pos = newPos;
  //   if(dir == RotaryEncoder::Direction::COUNTERCLOCKWISE){
  //     //motorMoveDistance += (-1 * MOTOR_INCREMENT);
  //     //Serial.println("Turning CCW");
  //     ws.textAll("Turning CCW");
  //     motorspeedAdd(-10);
  //   }else{
  //     //motorMoveDistance += MOTOR_INCREMENT;

  //     //Serial.println("Turning CW");
  //     ws.textAll("Turning CW");
  //     motorspeedAdd(10);

  //   }
  //   // Serial.printf("mMD = %ld\r\n", motorMoveDistance );
  //   digitalWrite(ONBOARD_LED,HIGH);
  // }
}
