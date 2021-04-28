/**
 * @file LogClientMlt.ino
 * 
 * @version
 *   2021/04/09 1.02    表示周りを変更　表示項目の追加
 *   2021/04/07 1.01    送信データ構造の変更
 *   2021/04/05 1.00
 * 
*/

#include <M5StickC.h>
#include <Wire.h>
#include "NimBLEDevice.h"

// The remote service we wish to connect to.
static NimBLEUUID     serviceUUID("181a");                                  // Environment Sensing UUID(0000181a-0000-1000-8000-00805f9b34fb)
static NimBLEUUID     charUUID("156f7abe-87c8-11eb-8dcd-0242ac130003");     // Generate https://www.uuidgenerator.net/


static boolean doScan = true;
static BLERemoteCharacteristic      *pRemoteCharacteristic;
static BLEAdvertisedDevice          *myDevice = NULL;

typedef struct {   // Bluetoothで送信するデータ     2byte * 7 = 14
  uint8_t         id = 0;             // uniqueID 送信側の個々IDに使用
  union {
    struct {
      char       alarmTL : 1;         // 低温度アラーム
      char       alarmTH : 1;         // 高温度アラーム
      char       unused  : 6;         // 未使用
    };
    uint8_t       flags = 0;       // statusフラグ
  };
  struct {                         // 月日
    uint8_t       month = 0;          // 月
    uint8_t       date = 0;           // 日
  } md;
  struct {                         // 時分
    uint8_t       hours = 0;          // 時
    uint8_t       minutes = 0;        // 分
  } hm;
  int16_t         pressure = 0;       // 気圧         int(pres *  10)
  int16_t         temperature = 0;    // 温度         int(temp * 100)
  int16_t         humidity = 0;       // 湿度         int(temp * 100)
  int16_t         voltage = 0;        // 電圧         int(volt * 100)
} BLE_DataPacket;
#define           SIZE_BLEDATA     14         // 上記構造体のサイズ（バイト）
BLE_DataPacket    bleDataPacket;              // 一時保管用

typedef struct {
  boolean                       doConnect = false;                // 接続要請flag
  boolean                       connected = false;                // 接続済みflag
  int                           rssi = 0;                         // RSSI
  NimBLEAddress                 serverAdrs;                       // ペリフェラル（peripheral）デバイスアドレス                        
  BLE_DataPacket                dataPacket;                       // データパケット
  NimBLERemoteCharacteristic    *pCharacteristic = NULL;          //
  NimBLEAdvertisedDevice        *pDevice = NULL;
} BLE_Device;

  // ペリフェラルデバイスリスト
#define MAX_DEVICELIST        10                                  // 接続デバイス最大数
int                           dispDevice  = 0;                    // LCD表示中デバイス
int                           kDevice     = 0;                    // カウンタ
BLE_Device                    deviceList[MAX_DEVICELIST];         // デバイスリスト

  // 動作設定
#define AUTOTIME          5000                                    // 自動表示の切換え時間（5000ミリ秒）
uint32_t                  autoMode      = 0;                      // 自動表示モード　0:オフ,>0:オン（切替までのミリ秒）


/*===========================================================================
 * 汎用変数
 */
int16_t        i,j;
int16_t        x,y;
char           tmpStr[80];

/*===========================================================================
 * Characteristicコールバック
 *    取得データを該当ペリフェラルへコピー
 */
static void notifyCallback(NimBLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  int     k;
//    Serial.print("Notify callback for characteristic ");
//    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
//    Serial.print(" of data length ");
//    Serial.print(length);
//    Serial.print(" -- data: ");
//    for (i = 0; i < length; i++) {
//      Serial.printf("%02x",pData[i]);
//      if (i % 2 == 1) Serial.printf(" ");
//    }
//    Serial.println();
  memcpy((char *)&bleDataPacket,(char *)pData,SIZE_BLEDATA);         // 受信データをパケット構造体へコピー
  Serial.printf("ID %d (%d)-->  ",bleDataPacket.id,pBLERemoteCharacteristic->getHandle());
  Serial.printf("Time %2d:%02d / ",bleDataPacket.hm.hours,bleDataPacket.hm.minutes);
  Serial.printf("Temp %.2f / Humi %.2f / ",bleDataPacket.temperature / 100.,bleDataPacket.humidity / 100.);
  Serial.printf("Pres %.1f / Volt %.2f /",bleDataPacket.pressure / 10.,bleDataPacket.voltage / 100.);
  Serial.printf("Adrs %s / RSSI %d\r\n",pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString().c_str(),pBLERemoteCharacteristic->getRemoteService()->getClient()->getRssi());
    
  for (k = 0; k < kDevice; k++) {   
    if (deviceList[k].serverAdrs.equals(pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress())) {
      deviceList[k].dataPacket = bleDataPacket;
      deviceList[k].rssi       = pBLERemoteCharacteristic->getRemoteService()->getClient()->getRssi();
      break;
    }
  }
}
/*===========================================================================
 * ペリフェラル接続・切断のコールバック
 *
 */
class ClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *pclient) {
    Serial.printf("onConnect Adrs : %s\r\n",pclient->getPeerAddress().toString().c_str());
    pclient->updateConnParams(120,120,0,60);
  }

  void onDisconnect(BLEClient* pclient) {
    int       k;
    for (k = 0; k < kDevice; k++) {
      if (deviceList[k].pDevice == NULL) continue;

      if (pclient->getPeerAddress().equals(deviceList[k].pDevice->getAddress())) {
        Serial.printf("onDisconnect Adrs : %s (ID:%d)\r\n",pclient->getPeerAddress().toString().c_str(),k);
        deviceList[k].connected = false;                // 再接続要請
        break;
      }
    }
  }
};
/*===========================================================================
 * ペリフェラルへの接続処理
 *
 */
bool connectToServer(int idx) {
   
  myDevice = deviceList[idx].pDevice;
    
  Serial.printf("Forming a connection to (%d) %s \r\n",idx,myDevice->getAddress().toString().c_str());
    
  NimBLEClient*  pClient  = BLEDevice::createClient();          //  Serial.println(" - Created client");
  pClient->setClientCallbacks(new ClientCallback());            //  Serial.println("   Set ClientCallback");

    // Connect to the remove BLE Server.
      // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  pClient->connect(myDevice);                                   // Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
  NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.printf("Failed to find our service UUID: %s\r\n",serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
  deviceList[idx].pCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (deviceList[idx].pCharacteristic == nullptr) {
    Serial.printf("Failed to find our characteristic UUID: %s\r\n",charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
//  if(deviceList[idx].pCharacteristic->canRead()) {      // canReadでも正しいデータになってない可能性あり
//    memcpy((char *)&deviceList[idx].dataPacket,(char *)deviceList[idx].pCharacteristic->readValue().c_str(),SIZE_BLEDATA);         // 受信データをパケット構造体へコピー
//  }

  if(deviceList[idx].pCharacteristic->canNotify()) {
    deviceList[idx].pCharacteristic->registerForNotify(notifyCallback);
    Serial.println(" - Set Notify");
  } else {
    Serial.println("Failed to not Notify");
  }
  
  deviceList[idx].connected = true;

  return true;
}
/*===========================================================================
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) {
    int     k;
    if (advertisedDevice->isAdvertisingService(serviceUUID)) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice->toString().c_str());

      doScan = false;
      for (k = 0; k < kDevice; k++) {      // 既知ペリフェラル？
        if (deviceList[k].serverAdrs.equals(advertisedDevice->getAddress())) {
          Serial.printf("reConnected Device %s\r\n",advertisedDevice->getName().c_str());
          deviceList[k].doConnect = true;
          doScan = true;
          break;
        }
      }
      if (!doScan) {                       // 新規ペリフェラル
        Serial.printf("New Device : %s(%s)\r\n",advertisedDevice->getName().c_str(),advertisedDevice->getAddress().toString().c_str());
        deviceList[kDevice].serverAdrs = advertisedDevice->getAddress();
        deviceList[kDevice].pDevice    = new NimBLEAdvertisedDevice(*advertisedDevice);
        deviceList[kDevice].doConnect = true;
        kDevice++;
        doScan = true;
      }
      deviceList[kDevice].rssi = advertisedDevice->getRSSI();
    } // Found our server
  } // onResult
}; // AdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);

  M5.begin();
  Wire.begin(0,26);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.printf("Search\r\nPeripheral");
  M5.Axp.ScreenBreath(9);

  for (i = 0; i < MAX_DEVICELIST; i++) {
    deviceList[i].pDevice  = NULL;
  }
  
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  NimBLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
//  pBLEScan->start(5, false);
} // End of setup.


// This is the Arduino main loop function.
void loop() {

  M5.update();
  if (M5.BtnA.wasPressed()) {           // 表示切替
    if (++dispDevice >= kDevice) dispDevice = 0;
    if (autoMode > 0) autoMode = millis() + AUTOTIME;     // 切り替でautoMode値をリセット
  }
  if (M5.BtnB.isPressed()) {            // 自動切換え On/Off
    if (autoMode == 0) autoMode = millis() + AUTOTIME;
    else               autoMode = 0;
    Serial.printf("AutoMode %d\r\n",autoMode);
  }

    // 受信データ表示の自動切替
  if (autoMode > 0) {
    if (millis() > autoMode) {
      if (++dispDevice >= kDevice) dispDevice = 0;
      autoMode = millis() + AUTOTIME;
      Serial.printf("Demo Mode %d\r\n",dispDevice);
    }
  }
  
      // 受信データの表示
  bleDataPacket = deviceList[dispDevice].dataPacket;
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,0,2);                          // 日時　時間　ペリフェラル番号
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  M5.Lcd.printf("%2d/%2d %d:%02d #%d",bleDataPacket.md.month,bleDataPacket.md.date,bleDataPacket.hm.hours,bleDataPacket.hm.minutes,bleDataPacket.id);
  M5.Lcd.setCursor(115,0);                          // 受信中表示
  M5.Lcd.setTextColor(TFT_BLACK,TFT_CYAN);
  if (deviceList[dispDevice].connected) M5.Lcd.printf(" LIVE ");

  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);         // 気圧　電圧 RSSI
  M5.Lcd.setCursor(0,13,2);
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf(" %dhPa",(int)(bleDataPacket.pressure / 10.));
  if (bleDataPacket.voltage < 360) M5.Lcd.setTextColor(TFT_BLACK,TFT_RED);
  M5.Lcd.printf(" %.2fV",bleDataPacket.voltage / 100.);
  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);         // RSSI
  M5.Lcd.printf(" %ddBm",deviceList[dispDevice].rssi);

  M5.Lcd.setCursor(0,26,4);                         // 気温
  M5.Lcd.setTextSize(1);
  if (bleDataPacket.alarmTL)      M5.Lcd.setTextColor(TFT_CYAN,TFT_BLACK);
  else if (bleDataPacket.alarmTH) M5.Lcd.setTextColor(TFT_RED,TFT_BLACK);
  else   M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  M5.Lcd.printf("Temp %.2fC",bleDataPacket.temperature / 100.);

  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);         // 湿度
  M5.Lcd.setCursor(0,45,4);
  M5.Lcd.printf("Humi %.2f%%",bleDataPacket.humidity / 100.);

   //----- 本体の情報表示
  x = 0; y = 65;
  if (M5.Axp.GetVBusVoltage() > 3.3) sprintf(tmpStr," %.2fV %dmA",M5.Axp.GetBatVoltage(),(int)M5.Axp.GetVBusCurrent());
  else                               sprintf(tmpStr," %.2fV %dmA",M5.Axp.GetBatVoltage(),(int)M5.Axp.GetBatCurrent());
  if (autoMode > 0) {
    i = strlen(tmpStr);  
    sprintf(&tmpStr[i]," auto");
  }
  M5.Lcd.fillRect(0,65,160,16,TFT_GREENYELLOW);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_ORANGE);
  M5.Lcd.drawString(tmpStr,x,y,2);
  M5.Lcd.drawString(tmpStr,x+1,y,2);
  
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.

      // 再接続
  for (i = 0; i < kDevice; i++) {
    if (deviceList[i].pDevice == NULL) continue;
    if (deviceList[i].connected) continue;
    if (!deviceList[i].doConnect) continue;

    if (connectToServer(i)) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    deviceList[i].doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
//    Serial.println("Rescan");
    NimBLEDevice::getScan()->start(1);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino

  delay(10); // Delay a second between loops.
} // End of loop
