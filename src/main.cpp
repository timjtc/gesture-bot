#include <Arduino.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ReactESP.h>
#include <SparkFun_TB6612.h>

Preferences NVS;

// Function prototypes
void parseCommand(String command);
void setLedMode(uint8_t mode);
void sendTelemetry();
void motorDrive(int left, int right);

// Pin definitions
const uint8_t LED = 2;                                        // Onboard LED, don't connect to anything
const uint8_t DRIVER_AIN1 = 17;                               // Motor driver AI1
const uint8_t DRIVER_BIN1 = 18;                               // Motor driver BI1
const uint8_t DRIVER_AIN2 = 16;                               // Motor driver AI2
const uint8_t DRIVER_BIN2 = 19;                               // Motor driver BI2
const uint8_t DRIVER_PWMA = 4;                                // Motor driver PWMA
const uint8_t DRIVER_PWMB = 21;                               // Motor driver PWMB
const uint8_t DRIVER_STBY = 5;                                // Motor driver STBY
/*
  Available pins in ESP32:
  0, //2, //4, //5, 12, //13, 14, //15, 18, 19, 21, //22, //23, //25, //26, //27, //32, //33
*/

// BLE setup
const char* SERVICE_UUID = "a16587d4-584a-4668-b279-6ccb940cdfd0";
const char* CHARACTERISTIC_UUID_TX = "a16587d4-584a-4668-b279-6ccb940cdfd1";
const char* CHARACTERISTIC_UUID_RX = "a16587d4-584a-4668-b279-6ccb940cdfd2";
BLEServer* Server = NULL;
BLECharacteristic *CharTX, *CharRX;
bool dev_connected = false;
bool last_dev_connected = false;
String tx_idle = "";
String tx_drive = "";

// Motor setup
const int offset_motorA = 1;
const int offset_motorB = 1;
Motor MotorA_Left = Motor(DRIVER_AIN1, DRIVER_AIN2, DRIVER_PWMA, offset_motorA, DRIVER_STBY);
Motor MotorB_Right = Motor(DRIVER_BIN1, DRIVER_BIN2, DRIVER_PWMB, offset_motorB, DRIVER_STBY);

// Vehicle controller setup
const uint8_t MODE_IDLE = 1;
const uint8_t MODE_DRIVE = 2;
uint8_t mode = MODE_IDLE;

int speed_limit = 230;
int arm_motor = false;

int left_motor = 0;
int right_motor = 0;

// Blinker setup
EventLoop LEDQtrCalBlinker;
EventLoop LEDActiveBlinker;
EventLoop LEDIdleBlinker;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    dev_connected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    dev_connected = false;
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rx_value = pCharacteristic->getValue();
    if (rx_value.length() > 0) {
      parseCommand(String(rx_value.c_str()));
    }
  }
};

void setup() {

  // Initialize serial and pin modes
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // Load data
  NVS.begin("lfav", false);

  // Initialize blinkers
  LEDQtrCalBlinker.onRepeat(100, [] () {
      static bool state = false;
      digitalWrite(LED, state = !state);
  });
  LEDActiveBlinker.onRepeat(250, [] () {
      static bool state = false;
      digitalWrite(LED, state = !state);
  });
  LEDIdleBlinker.onRepeat(500, [] () {
      static bool state = false;
      digitalWrite(LED, state = !state);
  });

  BLEDevice::init("GestureBot");

  // Create the BLE Server
  Server = BLEDevice::createServer();
  Server->setCallbacks(new ServerCallbacks());
  BLEService *Service = Server->createService(SERVICE_UUID);

  // BLEChar for telemetry data
  CharTX = Service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // BLEChar for receiving commands
  CharRX = Service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE
  );

  CharRX->setCallbacks(new CommandCallbacks());
  CharTX->addDescriptor(new BLE2902());
  CharRX->addDescriptor(new BLE2902());
  Service->start();
  Server->getAdvertising()->start();
  Serial.println("Waiting for a client connection...");
  digitalWrite(LED, HIGH);

  // Load data
  speed_limit = NVS.getUShort("speed_limit", 230);

}

void loop() {

  // On connect
  if (dev_connected) {
    
    setLedMode(mode);

    if (mode == MODE_DRIVE) {
      pidLineFollow();
      sendTelemetry();
    }

    if (mode == MODE_IDLE) {
      // Safe all motors
      left_motor = 0;
      right_motor = 0;
      MotorA_Left.brake();
      MotorB_Right.brake();

      sendTelemetry();

      delay(500);
    }

  }

  // On disconnect
  if (!dev_connected && last_dev_connected) {

    // Reset mode and stop motors
    mode = MODE_IDLE;

    // Give the BLE stack the chance to get things ready and advertise again
    delay(500);
    Server->startAdvertising(); 
    Serial.println("Advertising again...");
    last_dev_connected = dev_connected;

    digitalWrite(LED, HIGH);

  }

  // While connecting
  if (dev_connected && !last_dev_connected) {

    last_dev_connected = dev_connected;
    digitalWrite(LED, HIGH);

  }

}

void parseCommand(String command) {
  command.trim();
  if (command.startsWith("mode ")) {
    String mode_value = command.substring(5);
    if (mode_value == "drv") {
      mode = MODE_DRIVE;
    } else if (mode_value == "id") {
      mode = MODE_IDLE;
    } else {
      Serial.println("Unknown mode: " + mode_value + ". Available modes: id, drv");
    }
  } else if (command.startsWith("set sl ")) {
    String sl = command.substring(7);
    NVS.putUShort("speed_limit", sl.toInt());
    speed_limit = sl.toInt();
  } else if (command.startsWith("arm md")) {
    arm_motor = 1;
  } else if (command.startsWith("safe md")) {
    arm_motor = 0;
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void sendTelemetry() {
  if (!dev_connected) return;
  if (mode == MODE_IDLE)
  {
    tx_idle = String(mode) + "," + 
              String(arm_motor) + "," + 
              String(speed_limit);
    CharTX->setValue(tx_idle.c_str());
    CharTX->notify();
  } else if (mode == MODE_DRIVE) {
    tx_drive = String(mode) + "," + 
                String(arm_motor) + "," + 
                String(speed_limit);
      CharTX->setValue(tx_drive.c_str());
      CharTX->notify();
  }
}

void setLedMode(uint8_t mode) {

  if (mode == MODE_IDLE) {
    LEDIdleBlinker.tick();
  } else if (mode == MODE_DRIVE) {
    LEDQtrCalBlinker.tick();
  }

}

void motorDrive(int left, int right) {
  
  left_motor = left;
  right_motor = right;
  if (arm_motor) {
    MotorA_Left.drive(left);
    MotorB_Right.drive(right);
  } else {
    MotorA_Left.brake();
    MotorB_Right.brake();
  }

}