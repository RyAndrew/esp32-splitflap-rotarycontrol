#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <debounce.h>
#include <RotaryEncoder.h>

#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 22
#define ROTARY_ENCODER_BUTTON_PIN 21
#define ROTARY_ENCODER_STEPS 4

#define LED 2

#define MOTOR_DIR_PIN 12
#define MOTOR_STEP_PIN 13

const int MOTOR_INCREMENT = 15;
const int MOTOR_STEP_TIMER = 1500;

volatile long motorMoveDistance = 0;

//WIFI
const char* ssid = "wifissid";
const char* password = "wifipassword";

RotaryEncoder *encoder = nullptr;

hw_timer_t *Motor_timer = NULL;

void IRAM_ATTR runMotor(){

  if(digitalRead(MOTOR_STEP_PIN)){
    digitalWrite(MOTOR_STEP_PIN,LOW);
      //Serial.print("mL-");
      //Serial.println(motorMoveDistance);
  }else{
    if(motorMoveDistance<0){
      digitalWrite(MOTOR_DIR_PIN,HIGH);
      digitalWrite(MOTOR_STEP_PIN,HIGH);
      motorMoveDistance++;
      if(motorMoveDistance==0){
        digitalWrite(LED,LOW);
        encoder->setPosition(0);
      }
    }else if(motorMoveDistance>0){
      digitalWrite(MOTOR_DIR_PIN,LOW);
      digitalWrite(MOTOR_STEP_PIN,HIGH);
      motorMoveDistance--;
      if(motorMoveDistance==0){
        digitalWrite(LED,LOW);
        encoder->setPosition(0);
      }
    }
  }
}



static void buttonHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    Serial.println("button pressed");
    digitalWrite(LED,!digitalRead(LED));
    motorMoveDistance=0;
    encoder->setPosition(0);
  } else {
    Serial.println("Released button");
  }
}

static Button myButton(0, buttonHandler);

void IRAM_ATTR checkPosition() {
  encoder->tick(); // just call tick() to check the state.
}

void setup() {
  
  pinMode(LED,OUTPUT);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), checkPosition, CHANGE);

  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);

  digitalWrite(MOTOR_STEP_PIN,LOW);
  digitalWrite(MOTOR_DIR_PIN,HIGH);

  Serial.begin(115200);
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  InitOta();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //LED on when boot finished
  digitalWrite(LED,HIGH);

  Motor_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(Motor_timer, &runMotor, true);
  timerAlarmWrite(Motor_timer, MOTOR_STEP_TIMER, true);
  timerAlarmEnable(Motor_timer);
}

void InitOta( ) {

  // Port defaults to 3232
  //ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  //ArduinoOTA.setHostname("esp32");

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
      else // U_SPIFFS
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

void loop() {

  ArduinoOTA.handle();

  myButton.update(digitalRead(ROTARY_ENCODER_BUTTON_PIN));
  static int pos = 0;
  encoder->tick();

  int newPos = encoder->getPosition();
  if (pos != newPos && newPos != 0) {
    RotaryEncoder::Direction dir = encoder->getDirection();
    // Serial.print("pos:");
    // Serial.print(pos);
    // Serial.print(" newPos:");
    // Serial.print(newPos);
    // Serial.print(" dir:");
    // Serial.println((int)dir);

    pos = newPos;
    if(dir == RotaryEncoder::Direction::COUNTERCLOCKWISE){
      motorMoveDistance += (-1 * MOTOR_INCREMENT);
      // Serial.println("Turning CCW");
    }else{
      motorMoveDistance += MOTOR_INCREMENT;
      // Serial.println("Turning CW");
    }
    // Serial.printf("mMD = %ld\r\n", motorMoveDistance );
    digitalWrite(LED,HIGH);
  }
}