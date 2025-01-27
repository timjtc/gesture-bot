#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>

// Pin definitions
const uint8_t LED = 2;                                  // Onboard LED, don't connect to anything
const uint8_t MPU_SCL = 22;                             // accelerometer SCL
const uint8_t MPU_SDA = 21;                             // accelerometer SDA

// BLE settings
#define BLEServerName "GestureBot"
static BLEUUID SERVICE_UUID("a16587d4-584a-4668-b279-6ccb940cdfd0");
static BLEUUID TELEMETRY_CHARUUID("a16587d4-584a-4668-b279-6ccb940cdfd1");
static BLEUUID CMDSTR_CHARUUID("a16587d4-584a-4668-b279-6ccb940cdfd2");
static BLEUUID CTLVAL1_CHARUUID("a16587d4-584a-4668-b279-6ccb940cdfd3");
static BLEUUID CTLVAL2_CHARUUID("a16587d4-584a-4668-b279-6ccb940cdfd4");
static BLEAddress *pServerAddress;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

// Define pointer for the BLE connection
static BLEAdvertisedDevice* BLEAdvDevice;
BLERemoteCharacteristic* TelemetryChar;
BLERemoteCharacteristic* CmdStrChar;
BLERemoteCharacteristic* CtlVal1Char;
BLERemoteCharacteristic* CtlVal2Char;

// Variables
int arm_motor = 0;
int speed_limit = 230;
int left_motor = 0;
int right_motor = 0;

// Vehicle controller setup
const uint8_t MODE_IDLE = 1;
const uint8_t MODE_DRIVE = 2;
uint8_t mode = MODE_IDLE;
double threshold = 2.6;

Adafruit_MPU6050 MPU;

// Callback function for Notify function
static void telemetryNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if(pBLERemoteCharacteristic->getUUID().toString() == TELEMETRY_CHARUUID.toString()) {

    // convert received bytes to integer
    // uint32_t counter = pData[0];
    // for(int i = 1; i<length; i++) {
    //   counter = counter | (pData[i] << i*8);
    // }

    // print to Serial
    Serial.println((char*)pData);
  }
}

// Callback function that is called whenever a client is connected or disconnected
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(BLEAdvDevice->getAddress().toString().c_str());
  
  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(BLEAdvDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(SERVICE_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  connected = true;
  TelemetryChar = pRemoteService->getCharacteristic(TELEMETRY_CHARUUID);
  CmdStrChar = pRemoteService->getCharacteristic(CMDSTR_CHARUUID);
  if(connectCharacteristic(pRemoteService, TelemetryChar) == false &&
     connectCharacteristic(pRemoteService, CmdStrChar) == false &&
     connectCharacteristic(pRemoteService, CtlVal1Char) == false &&
     connectCharacteristic(pRemoteService, CtlVal2Char) == false) {
      connected = false;
  }

  if(connected == false) {
    pClient-> disconnect();
    Serial.println("At least one characteristic UUID not found");
    return false;
  }
  return true;
}

// Function to chech Characteristic
bool connectCharacteristic(BLERemoteService* pRemoteService, BLERemoteCharacteristic* l_BLERemoteChar) {
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  if (l_BLERemoteChar == nullptr) {
    Serial.print("Failed to find one of the characteristics");
    Serial.print(l_BLERemoteChar->getUUID().toString().c_str());
    return false;
  }
  Serial.println(" - Found characteristic: " + String(l_BLERemoteChar->getUUID().toString().c_str()));

  if(l_BLERemoteChar->canNotify())
    l_BLERemoteChar->registerForNotify(telemetryNotifyCallback);

  return true;
}

// Scan for BLE servers and find the first one that advertises the service we are looking for.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  //Called for each advertising BLE server.
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
  
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
  
      BLEDevice::getScan()->stop();
      BLEAdvDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
  
    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  BLEDevice::init("");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  // Try to initialize!
  if (!MPU.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  MPU.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (MPU.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  MPU.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (MPU.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  MPU.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (MPU.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    std::string rxValue = TelemetryChar->readValue();
    Serial.println(rxValue.c_str());
    
    // String txValue = "String with random value from client: " + String(-random(1000));
    // Serial.println("Characteristic 2 (writeValue): " + txValue);
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    // CmdStrChar->writeValue(txValue.c_str(), txValue.length());
    
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }

  // In this example "delay" is used to delay with one second. This is of course a very basic 
  // implementation to keep things simple. I recommend to use millis() for any production code
  delay(1000);



  // sensors_event_t a, g, temp;
  // MPU.getEvent(&a, &g, &temp);

  // if (abs(a.acceleration.x) > threshold || abs(a.acceleration.y) > threshold) {
  //   if (a.acceleration.x < -threshold && a.acceleration.y <= threshold) {
  //     // forward
  //   }
  //   if (a.acceleration.x > threshold && a.acceleration.y <= threshold) {
  //     // backward
  //   }
  //   if (a.acceleration.x <= threshold && a.acceleration.y < -threshold) {
  //     // left
  //   }
  //   if (a.acceleration.x <= threshold && a.acceleration.y > threshold) {
  //     // right
  //   }
  // } else {
  //   // stop
  // }

  // /* Print out the values */
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);

  // Serial.println("");
  // delay(100);
}

void parseInput(const std::string& input) {
    std::vector<int> values;
    std::stringstream ss(input);
    std::string item;

    while (std::getline(ss, item, ',')) {
        values.push_back(std::stoi(item));
    }

    mode = values[0];
    arm_motor = 0;
    speed_limit = 0;
    left_motor = 0;
    right_motor = 0;

    if (mode == 1) {
        arm_motor = values[1];
        speed_limit = values[2];
    } else if (mode == 2) {
        arm_motor = values[1];
        speed_limit = values[2];
        left_motor = values[3];
        right_motor = values[4];
    }
}