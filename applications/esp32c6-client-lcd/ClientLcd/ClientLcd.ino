#include "BLEDevice.h"
#include "BLEScan.h"
#include "DFRobot_GDL.h"


/*M0*/
#if defined ARDUINO_SAM_ZERO
#define TFT_DC  7
#define TFT_CS  5
#define TFT_RST 6
/*ESP32 ESP8266*/
#elif defined(ESP32)
#define TFT_DC  D2
#define TFT_CS  D6
#define TFT_RST D3
/*ESP8266*/
#elif defined(ESP8266)
#define TFT_DC  D4
#define TFT_CS  D6
#define TFT_RST D5
/* AVR series mainboard */
#else
#define TFT_DC  2
#define TFT_CS  3
#define TFT_RST 4
#endif


#define SERVICE_UUID        "0000fe40-cc7a-482a-984a-7f2ed5b3e58f"
#define SEN55_CHAR_UUID     "0000fe42-8e22-4541-9d4c-21edae82ed19"
#define LED_CHAR_UUID       "0000fe41-8e22-4541-9d4c-21edae82ed19"
#define bleServerName "sen55CST"



DFRobot_ST7735_128x160_HW_SPI screen(/*dc=*/TFT_DC,/*cs=*/TFT_CS,/*rst=*/TFT_RST);

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

static BLEUUID bleServiceUUID(SERVICE_UUID);
static BLEUUID bleSen55CharUUID(SEN55_CHAR_UUID);
static BLEUUID bleLedCharUUID(LED_CHAR_UUID);

/* characteristic that we want to read */
static BLERemoteCharacteristic *sen55Characteristic;
static BLERemoteCharacteristic *ledCharacteristic;

/* //Address of the peripheral device. Address will be found during scanning */
static BLEAddress *pServerAddress;

/* activate notify */
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

/* variables to store SEN55 data */
struct SEN55Data {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm4_0;
  uint16_t pm10;
  int16_t temperature;
  int16_t humidity;
  int16_t vocIndex;
  int16_t noxIndex;
};

SEN55Data sensorData = {0};

float getPM1_0() { return sensorData.pm1_0 / 10.0; }
float getPM2_5() { return sensorData.pm2_5 / 10.0; }
float getPM4_0() { return sensorData.pm4_0 / 10.0; }
float getPM10() { return sensorData.pm10 / 10.0; }

float getTemperature() {
  if (sensorData.temperature == 0x7fff) return -999;
  return sensorData.temperature / 200.0;
}

float getHumidity() {
  if (sensorData.humidity == 0x7fff) return -999;
  return sensorData.humidity / 100.0;
}

float getVOCIndex() {
  if (sensorData.vocIndex == 0x7fff) return -999;
  return sensorData.vocIndex / 10.0;
}

float getNOxIndex() {
  if (sensorData.noxIndex == 0x7fff) return -999;
  return sensorData.noxIndex / 10.0;
}

/* flags to check whether new sen55 readings are available */
unsigned long lastUpdate = 0;
boolean newSen55 = false;

/* connect to the BLE server */
bool connectToServer(BLEAddress pAddress) {
  BLEClient *pClient = BLEDevice::createClient();

  // connect to the remote BLE server
  pClient->connect(pAddress);
  Serial.println("- connected to server");

  // get a reference to the service 
  BLERemoteService *pRemoteService = pClient->getService(bleServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("failed to find the SEN55 UUID: ");
    Serial.println(bleServiceUUID.toString().c_str());
    return false;
  }

  // get the characteristics in the service of the remote BLE server
  sen55Characteristic = pRemoteService->getCharacteristic(bleSen55CharUUID);
  // ledCharacteristic = pRemoteService->getCharacteristic(bleLedCharUUID);

  if (sen55Characteristic == nullptr) {
    Serial.println("failed to find the characteristic UUID");
    return false;
  }

  Serial.println("- found  the characteristics");

  //Assign callback functions for the Characteristics
  sen55Characteristic->registerForNotify(sen55NotifyCallback);
  // ledCharacteristic->registerForNotify(ledNotifyCallback);
  return true;
}

static void sen55NotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t* pData,
  size_t length,
  bool isNotify) {
  // store sen55 data
  Serial.print("Received data, length: ");
  Serial.println(length);

  if (length >= 16) {
    sensorData.pm1_0 = pData[0] | (pData[1] << 8);
    sensorData.pm2_5 = pData[2] | (pData[3] << 8);
    sensorData.pm4_0 = pData[4] | (pData[5] << 8);
    sensorData.pm10 = pData[6] | (pData[7] << 8);
    sensorData.temperature = (int16_t)(pData[8] | (pData[9] << 8));
    sensorData.humidity = (int16_t)(pData[10] | (pData[11] << 8));
    sensorData.vocIndex = (int16_t)(pData[12] | (pData[13] << 8));
    sensorData.noxIndex = (int16_t)(pData[14] | (pData[15] << 8));

    // debug output
    Serial.println("--- Parsed Data ---");
    Serial.printf("PM1.0: %.1f µg/m³\n", getPM1_0());
    Serial.printf("PM2.5: %.1f µg/m³\n", getPM2_5());
    Serial.printf("PM4.0: %.1f µg/m³\n", getPM4_0());
    Serial.printf("PM10: %.1f µg/m³\n", getPM10());
    Serial.printf("Temperature: %.1f °C\n", getTemperature());
    Serial.printf("Humidity: %.1f %%RH\n", getHumidity());
    Serial.printf("VOC Index: %.1f\n", getVOCIndex());
    Serial.printf("NOx Index: %.1f\n", getNOxIndex());

    lastUpdate = millis();
    newSen55 = true;
  }
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

void printReadings() {
  // TODO: print sen55 data to TFT LCD
}

void setup() {
  Serial.begin(115200);
  screen.begin();

  // init ble
  BLEDevice::init("");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("we are now connected to the ble server");
      // activate the notify property of each characteristic
      sen55Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("failed to connect to the server");
    }
    doConnect = false;
  }

  if (newSen55) {
    newSen55 = false;
    printReadings();
  }
  delay(1000);

  screen.setTextSize(2);
  screen.fillScreen(COLOR_RGB565_BLACK);
  screen.setFont(&FreeMono12pt7b);
  screen.setCursor(/*x=*/32,/*y=*/64);
  screen.setTextColor(COLOR_RGB565_LGRAY);
  screen.setTextWrap(true);
  screen.print("DFRobot");
  delay(500);

  screen.fillScreen(COLOR_RGB565_BLACK);
  screen.setFont(&FreeMonoBold12pt7b);
  screen.setCursor(/*x=*/32,/*y=*/64);
  screen.setTextColor(COLOR_RGB565_GREEN);
  screen.setTextWrap(true);
  screen.print("GDL");
  delay(500);

  screen.fillScreen(COLOR_RGB565_BLACK);
  screen.setFont(&FreeMonoBoldOblique12pt7b);
  screen.setCursor(/*x=*/32,/*y=*/64);
  screen.setTextColor(COLOR_RGB565_RED);
  screen.setTextWrap(true);
  screen.print("fonts test");
  delay(500);

  screen.fillScreen(COLOR_RGB565_BLACK);
  screen.setFont(&FreeMonoOblique12pt7b);
  screen.setCursor(/*x=*/32,/*y=*/64);
  screen.setTextColor(COLOR_RGB565_BLUE);
  screen.setTextWrap(true);
  screen.print("hello,world!");
  delay(500);
}

