/*
    note: need add library Adafruit_BMP280 from library manage
    Github: https://github.com/adafruit/Adafruit_BMP280_Library
*/

/**
 * @file Plus_Env2Hat_Logger.ino
 * 
 * @note
 *   2021/03/26 BLEの接続ー＞切断ー＞接続の手順が正しいかは不明、デバイス名を変える時は一度オフにし、再度違い名前にする
 *   2021/04/15 ピンソケット＋Grove＋内部接続（ＲTC/電源管理）の３系統同時は複雑になるようだ、要調査
 *   
 * @version
 *   2021/04/28 1.07    Deepsleepの時間設定を秒単位に変更（分単位だと時間が飛ぶ事がある）
 *   2021/04/15 1.06    TVOC/SGP30ユニットを追加（未完）
 *   2021/04/08 1.05    アラーム方法の変更（BLE時はブザーナシに）
 *   2021/04/07 1.04    送信データ構造の変更
 *   2021/04/06 1.03    BLEライブラリをNimBLE変更
 *   2021/04/01 1.02    BLE送信実装
 *   2021/03/17 1.01    ログをシリアルモニタへ送信機能
 *   2021/03/16 1.00
 * 
*/

#define   DEBUG_MODE

#include <M5StickCPlus.h>
#include <math.h>
#include "SHT3X.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SGP30.h>

#include <NimBLEDevice.h>

/*---------------------------------------------------------
 * 各センサー用変数
 */
SHT3X                       sht3x;              // 温湿度センサー
Adafruit_BMP280             bmp280;             // 気圧センサー
Adafruit_SGP30              sgp30;              // TVOC/eCO2センサー

#define     LED_PIN       GPIO_NUM_10           // 付属LEDのGPIOの番号
/*---------------------------------------------------------
 * 大域変数　
 *          処理速度を上げるためにローカル変数や関数呼び出しにスタックを極力使わない
 */
        // センサー情報
float       temperature   = 0.0;      // 温度
float       humidity      = 0.0;      // 湿度
float       pressure      = 0.0;      // 気圧
float       altitude      = 0.0;      // 高度
int         wbgtIndex     = 0.0;      // 暑さ指数（Wet Bulb Globe Temperature）
int         discomfortIndex = 0.0;    // 不快指数（discomfort index）
int         tvoc          = 0;        // TVOC(揮発性有機化合物)
int         eCO2          = 0;        // eCO2(二酸化炭素相当量)

        // センサーアラーム
#define     MAXSET_TEMPLOW    -20
#define     MAXSET_TEMPHIGH    40

RTC_DATA_ATTR boolean     tempAlarm       = false;
RTC_DATA_ATTR int16_t     tempAlarmLow    = -10;          // 温度下限
RTC_DATA_ATTR int16_t     tempAlarmHigh   =  20;          // 温度上限        

RTC_DateTypeDef           rtcDate;                          // 年月日
RTC_TimeTypeDef           rtcTime;                          // 日時秒

/*
 * 気圧関連
 */
#define   DEFAULT_SEALEVEL        1013.25     // 標準気圧
#define   SEALEVEL_TEMPERATURE    15          // 海面温度
RTC_DATA_ATTR float     seaLevelPressure = DEFAULT_SEALEVEL;      // 設定海面気圧


        // 動作設定用
#define WAKEUP_TIME       SLEEP_MIN(30)                     // ディープ休眠時間
#define DEMOTIME          5000                              // デモモードの画面切り替え時間（5000ミリ秒）

RTC_DATA_ATTR uint8_t     scrnMode      = 0;                // LCD表示内容　(0:温度・湿度,1:履歴）
RTC_DATA_ATTR uint8_t     lcdDirection  = 1;                // LCDの向き　 1 or 3  
RTC_DATA_ATTR uint8_t     lcdBrightness = 9;                // LCDの明るさ 7 to 15
RTC_DATA_ATTR uint32_t    defaultPowerOffTime = 20000;      // スリープ時間
RTC_DATA_ATTR boolean     resumeOn      = false;            // レジューム（スリープ時の表示に戻る）
uint16_t                  demoMode      = 0;                // デモモード　0:オフ,>0:オン（切替までのミリ秒）

uint32_t                  powerOffTime = defaultPowerOffTime;    // ディープスリープへ移行する時間

boolean       foundSGP30  = false;                          // SGP30センサー有り
long          readySGP30  = 15 * 1000;                      // 0=> SGP30センサー使用可、> 0 未初期化

        // 画面用　ダブルバッファー他
int16_t       scrnWidth,scrnHeight;                         // スクリーン（LCD)縦横
TFT_eSprite   lcdDblBuf = TFT_eSprite(&M5.Lcd);             // ダブルバッファー

uint32_t      update_time   = 0;                            // LCD書き換え間隔管理

/*
 * 気圧変動の確認用にスローメモリーへ保存する構造体、日付・気温・湿度・気圧を30分毎に保存
 */
typedef struct PressArray_ {            //  RTCメモリへ保存構造体
  int      day;                         //    日
  int      minutes;                     //    分
  int      pressure;                    //    気圧
  int      temperature;                 //    温度
  int      humidity;                    //    湿度
} PressArray;

#define MAX_PRESSARRAY   48             // 配列のサイズ
#define CLEAR_PRESSARRAY {0,0,0,0,0}    // 構造体初期化値
#ifdef DEBUG_MODE
RTC_DATA_ATTR PressArray presAry[MAX_PRESSARRAY] = {
                                                     { 15,  0, 1012, 31, 36 },  //  0:00
                                                     { 15, 30, 1013, 30, 37 },  //  0:30
                                                     { 15,  0, 1013, 30, 37 },  //  1:00
                                                     { 15, 30, 1014, 29, 37 },  //  1:30
                                                     { 15,  0, 1014, 29, 37 },  //  2:00
                                                     { 15, 30, 1014, 29, 38 },  //  2:30
                                                     { 15,  0, 1015, 28, 38 },  //  3:00
                                                     { 15, 30, 1015, 28, 38 },  //  3:30
                                                     { 15,  0, 1016, 28, 38 },  //  4:00
                                                     { 15, 30, 1016, 28, 38 },  //  4:30
                                                     { 15,  1, 1017, 28, 38 },  //  5:00
                                                     { 15, 30, 1018, 28, 38 },  //  5:30
                                                     { 15,  0, 1018, 27, 38 },  //  6:00
                                                     { 15, 30, 1019, 27, 38 },  //  6:30
                                                     { 15,  0, 1019, 27, 38 },  //  7:00
                                                     { 15, 30, 1019, 24, 47 },  //  7:30
                                                     { 15,  0, 1019, 23, 59 },  //  8:00
                                                     { 15, 30, 1020, 23, 57 },  //  8:30
                                                     { 15,  0, 1020, 23, 60 },  //  9:00
                                                     { 15, 30, 1023, 23, 49 },  //  9:30
                                                     { 15,  0, 1021, 23, 45 },  // 10:00
                                                     { 15, 30, 1021, 24, 40 },  // 10:30
                                                     { 15,  0, 1021, 25, 37 },  // 11:00
                                                     { 15, 30, 1020, 25, 35 },  // 11:30
                                                     { 15,  0, 1020, 25, 34 },  // 12:00
                                                     { 15, 30, 1020, 25, 32 },  // 12:30
                                                     { 15,  0, 1020, 25, 31 },  // 13:00
                                                     { 15, 30, 1020, 25, 30 },  // 13:30
                                                     { 15,  0, 1020, 24, 29 },  // 14:00
                                                     { 15, 30, 1020, 25, 30 },  // 14:30
                                                     { 15,  0, 1019, 25, 29 },  // 15:00
                                                     { 15, 30, 1020, 25, 28 },  // 15:30
                                                     { 15,  0, 1021, 24, 29 },  // 16:00
                                                     { 15, 30, 1021, 24, 29 },  // 16:30
                                                     { 15,  0, 1021, 24, 29 },  // 17:00
                                                     { 14, 30, 1004, 24, 58 },  // 17:30
                                                     { 14,  0, 1004, 24, 57 },  // 18:00
                                                     { 14, 30, 1006, 24, 56 },  // 18:30
                                                     { 14,  0, 1006, 24, 55 },  // 19:00
                                                     { 14, 30, 1009, 24, 57 },  // 19:30
                                                     { 14,  0, 1006, 25, 57 },  // 20:00
                                                     { 14, 30, 1008, 25, 68 },  // 20:30
                                                     { 14,  0, 1009, 27, 73 },  // 21:00
                                                     { 14, 30, 1010, 34, 42 },  // 21:30
                                                     { 14,  0, 1009, 36, 48 },  // 22:00
                                                     { 14, 30, 1010, 36, 56 },  // 22:30
                                                     { 14,  0, 1011, 32, 34 },  // 23:00
                                                     { 14, 30, 1012, 31, 35 },  // 23:30
                                                    };
#else
RTC_DATA_ATTR PressArray presAry[MAX_PRESSARRAY] = { CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                     CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                                    };
#endif

/*
 * 記録表示
 */
int       logListOffset = 0;

/*
 * BLE 関係
 */
#define SERVICE_UUID                   "181a"                                        // Environment Sensing UUID(0000181a-0000-1000-8000-00805f9b34fb)
#define CHARACTERISTIC_UUID            "156f7abe-87c8-11eb-8dcd-0242ac130003"        // Generate https://www.uuidgenerator.net/

BLEServer         *pBLEServer          = NULL;
BLECharacteristic *pBLECharacteristic  = NULL;
boolean            bleConnected        = false;

#define           MAX_SENDINTERVAL       6
#define           BLEDEVICE_NAME         "StickLogger"
RTC_DATA_ATTR uint16_t    idxBleSendInterval    = 4;
uint16_t                  bleSendInterval[MAX_SENDINTERVAL]    = { 1, 3, 5, 10, 15, 30};
RTC_DATA_ATTR uint8_t     bleDeviceNumber  = 0;             // 0:BLE off / 1～9:Number   BLEDEVICE_NAME + bleDeviceNumber

struct {   // Bluetoothで送信するデータ   2byte * 9 = 18byte
  uint8_t         id;             // uniqueID 送信側の個々IDに使用
  union {
    struct {
      char       alarmTL : 1;     // 低温度アラーム
      char       alarmTH : 1;     // 高温度アラーム
      char       unused  : 6;     // 未使用
    };
    uint8_t       flags;      // statusフラグ
  };
  struct {                    // 月日
    uint8_t       month;          // 月
    uint8_t       date;           // 日
  } md;
  struct {                    // 時分
    uint8_t       hours;          // 時
    uint8_t       minutes;        // 分
  } hm;
  int16_t         pressure;       // 気圧         int(pres *  10)
  int16_t         temperature;    // 温度         int(temp * 100)
  int16_t         humidity;       // 湿度         int(temp * 100)
  int16_t         voltage;        // 電圧         int(volt * 100)
  int16_t         tvoc;           // 揮発性有機化合物
  int16_t         eCO2;           // 二酸化炭素相当量
} bleDataPacket;  

/*
 * 電源監視用
 */
boolean     extPW = false;                    /* 電源使用 true:外部電源 / false:内部電源 */
float       pwVolt;                           // Power Voltage
float       pwCurt;                           // Power Current

/*===========================================================================
 * 汎用変数
 */
int16_t     i,j;                 // カウンター
int16_t     x,y;                 // 座標用
char        tmpStr[40];          // 汎用文字列

/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * 各種　一行計算式
 * 
 */
//  熱中症指数、不快指数の算出
#define CalcWBGTIndex(temp,humi)              (temp * (0.62 + humi * 0.005))
#define CalcdiscomfortIndex(temp,humi)        (0.81 * temp + 0.01 * humi * (0.99 * temp - 14.3) + 46.3)
//  標高ー＞気圧変換
#define   AltToPres(altitude)                 (pressure*pow((1-0.0065*altitude/(SEALEVEL_TEMPERATURE+273.2)),5.254))
// 時間からPressArrayのインデックスを求
#define CalcPressArrayIndex(hours,minutes)    (hours * 2 + int(minutes / 30))

// RGB565 24bit -> 16bit 色決定後に定数に直すこと
#define ToRGB565(r,g,b) ((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3))

/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * M5StickC-PlusのSetSleepはバグ有り　正常動作のSetSleep 2021/02/20
 * 
 */
void xSetSleep(void)          //version M5stick-C plus
{
  M5.Axp.Write1Byte(0x31, M5.Axp.Read8bit(0x31) | ( 1 << 3)); // short press to wake up
  M5.Axp.Write1Byte(0x90, M5.Axp.Read8bit(0x90) | 0x07); // GPIO1 "low noise" otherwise it goes to download mode when waking up
  M5.Axp.Write1Byte(0x12, M5.Axp.Read8bit(0x12) & ~(1<<1) & ~(1<<2) & ~(1<<3)); // Disable all outputs but DCDC1 (DCDC3 = NC, LDO2 = LCD-BL, LDO3 = LCD-Logic)
}
uint8_t           wakeUpCause;      // 起動理由 esp_sleep_get_wakeup_cause()の返り値


/*
 * BLE 関係　関数
 */
void BLE_Setup();               // BLEデバイス初期化

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      bleConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      bleConnected = false;
    }
};



/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 *＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * SETUP
 */
void setup() {

  // put your setup code here, to run once:
  M5.begin();
  Wire.begin(0,26);
//  Wire.begin(32,33);
  M5.Lcd.setRotation(lcdDirection);
  M5.Lcd.fillScreen(BLACK);
  M5.Axp.ScreenBreath(lcdBrightness);              // LCDの明るさ　９
  scrnWidth  = M5.Lcd.width();
  scrnHeight = M5.Lcd.height();
  M5.Rtc.GetData(&rtcDate);
  M5.Rtc.GetTime(&rtcTime);
  pinMode(LED_PIN,OUTPUT);                        // LED設定
  digitalWrite(LED_PIN,HIGH);


    // LCDのダブルバッファーの用意
  lcdDblBuf.createSprite(M5.Lcd.width(),M5.Lcd.height());
  lcdDblBuf.setSwapBytes(false);
  
  pinMode(M5_BUTTON_HOME, INPUT);
  if (bleDeviceNumber) {
    setCpuFrequencyMhz(80);             // CPUを20MHzで駆動 -> BLEを使用するため80Mhzに変更
  } else {
    setCpuFrequencyMhz(20);             // Bluetoothオフの時は低速で
  }
  
  if (!bmp280.begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

    /* Default settings from datasheet. */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */

    /* TVOC/eCO2 SGP30センサー */
  foundSGP30 = false;
  if (sgp30.begin()) {
    foundSGP30 = true;
    readySGP30 += millis();            // readySGP30が0>=なら読み取り可能
  } else {
    Serial.println("Sensor not found :(");
  }

    
    /* BLE SETUP */
  if (bleDeviceNumber != 0) BLE_Setup();  

  if (!resumeOn) scrnMode = 0;        // スクリーンの復帰
  if (demoMode != 0) demoMode = millis() + DEMOTIME;
  
  /*　電源ソースを調べる */
  pwVolt = M5.Axp.GetVBusVoltage();
  if (pwVolt > 3.3) extPW = true;

  /* ディープスリープ設定（ボタンA押下で起床 */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0);  

  /* 起動理由の格納 */
  wakeUpCause = esp_sleep_get_wakeup_cause();
}

/*******************************************************
 *  
 */
void BMM150_Calibrate();
enum ScrnMode { TEMPHUMI, LOGLIST };
void SetupMenu();                 // 設定
void DispTempHumi();              // 温度・湿度　デカ表示
void Dispog();             // 温度・湿度　デカ表示
void DispLogList();               // 記録表示
/*******************************************************
 *  LOOP
 */

void loop() {

  M5.update();
  if (M5.BtnA.wasPressed()) {
    scrnMode = (scrnMode + 1) & 0x01;
    lcdDblBuf.fillSprite(BLACK);
    update_time = 0;
    powerOffTime = defaultPowerOffTime + millis();
  }
  if (scrnMode == TEMPHUMI) {
    if (M5.BtnB.isPressed()) {
      SetupMenu();
      if (bleDeviceNumber != 0) {                 // DeviceNameを変えるときは一度BLEをOFFにすること
        if (pBLEServer == NULL) {
          setCpuFrequencyMhz(80);
//          delay(100);
          BLE_Setup();  
        }
      } else if (pBLEServer != NULL) {
        BLEDevice::deinit(true);
        pBLEServer = NULL;
        setCpuFrequencyMhz(20);
//        delay(100);
      }
      powerOffTime = defaultPowerOffTime + millis();
    }    
  }
  if (demoMode > 0) {
    if (millis() > demoMode) {
      scrnMode = (scrnMode + 1) & 0x01;
      demoMode = millis() + DEMOTIME;    
    }
  }

  if (scrnMode == LOGLIST) {
    if (M5.BtnB.isPressed()) {
      logListOffset++;
      if (logListOffset > 6) logListOffset = 0;
      powerOffTime = defaultPowerOffTime + millis();
    }    
  }

  /*--- SGP30センサー ---*/
  if (foundSGP30) {
    i = readySGP30 - millis();      // if文無いに書くと計算が出来ないらしい。
    if (i < 0) readySGP30 = 0;
  }
  if (readySGP30 <= 0) {
    if (!sgp30.IAQmeasure()) {
      Serial.println("Measurement failed");
      readySGP30 = 15 * 1000  + millis();
    }
  }
  
  /*--- 電源ソースを調べる ---*/
  pwVolt = M5.Axp.GetVBusVoltage();
  if (pwVolt > 3.3) extPW = true;
  else              extPW = false;

  if (millis() > update_time) {
    update_time = millis() + 500;          // 500ms間隔で再描画
    
    if(sht3x.get()==0){                     // 温度・湿度の取得
      temperature = sht3x.cTemp;
      humidity = sht3x.humidity;
      wbgtIndex = (int)CalcWBGTIndex(temperature,humidity);
      discomfortIndex = (int)CalcdiscomfortIndex(temperature,humidity);
    }

    pressure       = bmp280.readPressure() / 100;   //Pa -> hPa
    altitude       = bmp280.readAltitude(seaLevelPressure);

    M5.Rtc.GetData(&rtcDate);
    M5.Rtc.GetTime(&rtcTime);

      // 温度アラーム
    if (tempAlarm
     && (temperature < tempAlarmLow ||  temperature > tempAlarmHigh)) {
      M5.Lcd.fillScreen(BLACK);
      lcdDblBuf.pushSprite(0,0); 
      if (bleDeviceNumber == 0) M5.Beep.tone(2000);
    } else {
      M5.Beep.mute();
    }

    M5.Lcd.startWrite();
    switch (scrnMode) {
      case TEMPHUMI: 
              DispTempHumi();                // 温度・湿度表示
              break;
      case LOGLIST:
              DispLogList();
              break;
      default:
              DispLogList();
    }

    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
    M5.Lcd.endWrite();
    

    i = CalcPressArrayIndex(rtcTime.Hours,rtcTime.Minutes);
    /*--- 気圧をスローメモリーに保存 0,3,6,9,12,15,18,21時の３時間ごとに保存 ---*/
    if (presAry[i].day != rtcDate.Date) {
      presAry[i].day          = rtcDate.Date;
      presAry[i].minutes      = rtcTime.Minutes;
      presAry[i].temperature  = (int)temperature;
      presAry[i].humidity     = (int)humidity;
      presAry[i].pressure     = (int)pressure;
    }

      // BLE送信
    if (bleDeviceNumber) {
      if ((rtcTime.Minutes % bleSendInterval[idxBleSendInterval] == 0) 
       || !(wakeUpCause == ESP_SLEEP_WAKEUP_TIMER)) {
        pwVolt = M5.Axp.GetBatVoltage();
        bleDataPacket.id          = bleDeviceNumber;
        bleDataPacket.flags       = (uint8_t)0;
        if (tempAlarm) {
          if (temperature < tempAlarmLow)  bleDataPacket.alarmTL     = 1;
          if (temperature > tempAlarmHigh) bleDataPacket.alarmTH     = 1;
        }
            
        bleDataPacket.md.month    = rtcDate.Month;
        bleDataPacket.md.date     = rtcDate.Date;
        bleDataPacket.hm.hours    = rtcTime.Hours;
        bleDataPacket.hm.minutes  = rtcTime.Minutes;
        bleDataPacket.pressure    = (int)(pressure * 10.);
        bleDataPacket.temperature = (int)(temperature * 100.);
        bleDataPacket.humidity    = (int)(humidity * 100.);
        bleDataPacket.voltage     = (int)(pwVolt * 100.);
        bleDataPacket.tvoc        = sgp30.TVOC;
        bleDataPacket.eCO2        = sgp30.eCO2;
        pBLECharacteristic->setValue((uint8_t*)&bleDataPacket, sizeof(bleDataPacket));
        pBLECharacteristic->notify();
        delay(10);
      }
    }

   //-------- 充電インジケータ
    pwCurt = M5.Axp.GetBatCurrent(); //　バッテリー放電電流
    if (pwCurt > 0.0) digitalWrite(LED_PIN,LOW);
    else              digitalWrite(LED_PIN,HIGH);

  /*--- DeepSleep の設定 
          バッテリーまたはdeepsleepから起動した場合は再びdeepsleepへ
  */
    if (!extPW || (wakeUpCause == ESP_SLEEP_WAKEUP_TIMER)) {  
      if (millis() > powerOffTime) {     // 20sec
        if (bleDeviceNumber) {
          i = bleSendInterval[idxBleSendInterval] * 60 - rtcTime.Seconds;      // 指定時間間隔（Bluetooth用）
        } else {
          i = 60 * (int(rtcTime.Minutes / 30 + 1) * 30 - rtcTime.Minutes);       // 30分間隔
        }
          //---ここからは AXP192::DeepSleep(uint64_t time_in_us)の必要部分の抜き出し
         xSetSleep();
         esp_sleep_enable_timer_wakeup(SLEEP_SEC(i));       // 次の計測時刻までスリープ
         esp_deep_sleep(SLEEP_SEC(i));
          //--- ここまで
      }
    }
  } // End of   "if (millis() > update_time) {"

  delay(50);
}
/*=====================================================================
 * Functions
 */
/*
 * BLE Setup
 */
void BLE_Setup() {

  sprintf(tmpStr,"%s%d",BLEDEVICE_NAME,bleDeviceNumber);
  BLEDevice::init(tmpStr);
  Serial.printf("Device Name : %s\r\n",tmpStr);
      // Create BLE-Server
    pBLEServer = BLEDevice::createServer();

      // Create the BLE Service
  BLEService *pBLEService = pBLEServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
  pBLECharacteristic = pBLEService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::READ   |
                      NIMBLE_PROPERTY::NOTIFY
                    );
                    
  // Start the service
  pBLEService->start();

  // Start advertising
  BLEAdvertising *pBLEAdvertising = BLEDevice::getAdvertising();
  pBLEAdvertising->addServiceUUID(SERVICE_UUID);
  pBLEAdvertising->setScanResponse(false);
  pBLEAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
}
/*=====================================================================================================
 * メニュー表示
 *  設定で速度関係ないからローカル変数を使用
 *  
 *  頁1 明るさ・スリープ・レジューム・BLE・ログ
 *  頁2 方向・日時・充電電圧・デモモード
 */

enum MenuPage   { MENU_NOTHING, MENU_PREV, MENU_NEXT, MENU_EXIT };

enum MenuPage SetupMenu1();        // メニュー１
enum MenuPage SetupMenu2();        // メニュー２     result -1:prev / 0:exit(return) / 1:next
/*
 * メインメニュー
 */
void SetupMenu() {
  uint16_t      idxPage = 0;      // メニュー頁
  enum MenuPage menuRet;
  
  for ( ; true ; ) {
    switch (idxPage) {
      case 0 : menuRet = SetupMenu1(); break;
      case 1 : menuRet = SetupMenu2(); break;
      default :
              if (idxPage < 0) idxPage = 0;
              if (idxPage > 1) idxPage = 1;
    }

    if (menuRet == MENU_EXIT) break;
    switch (menuRet) {
      case MENU_NEXT : idxPage++; break;
      case MENU_PREV : idxPage--; break;
    }
  }
}
/*
 * セットアップメニュー 頁1
 */
enum MenuPage SetupMenu1() {
  enum MenuPage   menuExit = MENU_NOTHING;
  uint16_t        idxMenu = 0;      // 選択項目
  uint16_t        btnAwasPressed;   // 単純に関数を毎回呼ぶの嫌だから
  uint32_t        autoExitTime;
  uint16_t        monthDay[12] = { 31,29,31,30,31,30,31,31,30,31,30,31 };

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextFont(1);
  lcdDblBuf.fillSprite(BLACK);

  autoExitTime = defaultPowerOffTime + millis();

  do {
    lcdDblBuf.setTextColor(TFT_WHITE, TFT_BLACK);

      //-- LCDの明るさ
    lcdDblBuf.setCursor(0,0);    lcdDblBuf.printf("Scrn Brightness ");
    lcdDblBuf.setCursor(210,0);  lcdDblBuf.printf("%2d",lcdBrightness);
      //-- スリープ時間
    lcdDblBuf.setCursor(0,20);    lcdDblBuf.printf("Sleep Time ");
    lcdDblBuf.setCursor(210,20);  lcdDblBuf.printf("%2d",defaultPowerOffTime / 1000);
      //-- レジューム（スクリーンの復帰）
    lcdDblBuf.setCursor(0,40);    lcdDblBuf.printf("Resume ");
    lcdDblBuf.setCursor(200,40);  
    if (resumeOn) lcdDblBuf.printf(" ON");
    else          lcdDblBuf.printf("OFF");
      //-- Bluetooth Low Energy
    lcdDblBuf.setCursor(0,60);    lcdDblBuf.printf("BLEname");
    sprintf(tmpStr,"%s0\0",BLEDEVICE_NAME);
    i = 2 * M5.Lcd.textWidth(tmpStr,1);
    lcdDblBuf.setCursor(scrnWidth - i,60);
    if (bleDeviceNumber == 0) {
      for (i = 0; i <= strlen(BLEDEVICE_NAME) - 3; i++) lcdDblBuf.printf(" ");
      lcdDblBuf.printf("OFF");
    }
    else                      lcdDblBuf.printf("%s%1d",BLEDEVICE_NAME,bleDeviceNumber);
      //-- Bluetooth 送信間隔
    lcdDblBuf.setCursor(0,80);    lcdDblBuf.printf("BLE interval");
    lcdDblBuf.setCursor(210,80);  lcdDblBuf.printf("%2d",bleSendInterval[idxBleSendInterval]);

    lcdDblBuf.setCursor(70,120);  lcdDblBuf.printf("NEXT");
    lcdDblBuf.setCursor(160,120); lcdDblBuf.printf("RETURN");

    M5.update();
    if (btnAwasPressed = M5.BtnA.wasPressed()) autoExitTime = defaultPowerOffTime + millis();
    if (M5.BtnB.wasPressed()) {
      if (++idxMenu > 6) idxMenu = 0;
      autoExitTime = defaultPowerOffTime + millis();
    }
    
    lcdDblBuf.setTextColor(TFT_BLACK, TFT_CYAN);
    switch (idxMenu) {
      case 0:     //スクリーンの明るさ
                if (btnAwasPressed) {
                  if (++lcdBrightness > 15) lcdBrightness = 7;
                  M5.Axp.ScreenBreath(lcdBrightness);              // LCDの明るさ　９
                }
                lcdDblBuf.setCursor(210,0);  lcdDblBuf.printf("%2d",lcdBrightness);
                break;
      case 1:     //スリープ時間
                if (btnAwasPressed) {
                  defaultPowerOffTime += 10000;
                  if (defaultPowerOffTime > 60000) defaultPowerOffTime = 10000;
                }
                lcdDblBuf.setCursor(210,20);  lcdDblBuf.printf("%2d",defaultPowerOffTime / 1000);
                break;
      case 2:     //レジューム
                if (btnAwasPressed) {
                  resumeOn = !resumeOn;
                }
                lcdDblBuf.setCursor(200,40);  
                if (resumeOn) lcdDblBuf.printf(" ON");
                else          lcdDblBuf.printf("OFF");
                break;
      case 3:     //BLE
                if (btnAwasPressed) {
                  if (++bleDeviceNumber > 9) bleDeviceNumber = 0;
                }
                sprintf(tmpStr,"%s0\0",BLEDEVICE_NAME);
                i = 2 * M5.Lcd.textWidth(tmpStr,1);
                lcdDblBuf.setCursor(scrnWidth - i,60);
                if (bleDeviceNumber == 0) {
                  for (i = 0; i <= strlen(BLEDEVICE_NAME) - 3; i++) lcdDblBuf.printf(" ");
                  lcdDblBuf.printf("OFF");
                }
                else                      lcdDblBuf.printf("%s%1d",BLEDEVICE_NAME,bleDeviceNumber);
                break;
      case 4:     // BLE 送信間隔
                if (btnAwasPressed) {
                  if (++idxBleSendInterval >= MAX_SENDINTERVAL) idxBleSendInterval = 0;
                }
                lcdDblBuf.setCursor(210,80);  lcdDblBuf.printf("%2d",bleSendInterval[idxBleSendInterval]);
                break;    
      case 5: // next
              lcdDblBuf.setCursor(70,120);   lcdDblBuf.printf("NEXT");
              if (btnAwasPressed) menuExit = MENU_NEXT;
              break;
      default:
              lcdDblBuf.setCursor(160,120); lcdDblBuf.printf("RETURN");
              if (btnAwasPressed) menuExit = MENU_EXIT;
    }
    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
                
    if (millis() > autoExitTime) menuExit = MENU_EXIT;

    delay(100);
  } while (menuExit == MENU_NOTHING);

  return(menuExit);
}
/*
 * セットアップメニュー 頁2
 */
enum MenuPage  SetupMenu2() {
  enum MenuPage   menuExit = MENU_NOTHING;
  uint16_t        idxMenu = 0;      // 選択項目
  uint16_t        btnAwasPressed;   // 単純に関数を毎回呼ぶの嫌だから
  uint32_t        autoExitTime;

                               //   1  2  3  4  5  6  7  8  9 10 11 12
  uint16_t        monthDay[12] = { 31,29,31,30,31,30,31,31,30,31,30,31 };

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextFont(1);
  lcdDblBuf.fillSprite(BLACK);

  autoExitTime = defaultPowerOffTime + millis();

  do {
    lcdDblBuf.setTextColor(TFT_WHITE, TFT_BLACK);

      //-- LCDの向き
    lcdDblBuf.setCursor(0,0);     lcdDblBuf.printf("Scrn Direction ");
    lcdDblBuf.setCursor(176,0);   
    if (lcdDirection == 1)     lcdDblBuf.printf(" LEFT");
    else                       lcdDblBuf.printf("RIGHT");
      //-- 温度アラーム値
    lcdDblBuf.setCursor(0,20); lcdDblBuf.printf("TempAlerm");
    lcdDblBuf.setCursor(200,20); 
    if (tempAlarm) lcdDblBuf.printf(" ON");
    else           lcdDblBuf.printf("OFF");
    lcdDblBuf.setCursor(0,40); lcdDblBuf.printf(" Low/High");
    lcdDblBuf.setCursor(148,40); lcdDblBuf.printf("%3d/%3d",tempAlarmLow,tempAlarmHigh);

      //-- 時刻
    M5.Rtc.GetData(&rtcDate);
    M5.Rtc.GetTime(&rtcTime);
    lcdDblBuf.setCursor(0,60);    lcdDblBuf.printf("DT ");
    lcdDblBuf.setCursor(40,60);   
      lcdDblBuf.printf("%4d/%02d/%02d %02d:%02d",
                          rtcDate.Year,rtcDate.Month,rtcDate.Date,rtcTime.Hours,rtcTime.Minutes);
      //-- デモモード
    lcdDblBuf.setCursor(0,80);    lcdDblBuf.printf("DemoMode");
    lcdDblBuf.setCursor(200,80);
    if (demoMode) lcdDblBuf.printf(" ON");
    else          lcdDblBuf.printf("OFF");
      //-- ログ出力
    lcdDblBuf.setCursor(0,100); lcdDblBuf.printf("Log Output");
    lcdDblBuf.setCursor(160,100); lcdDblBuf.printf("Serial");

    lcdDblBuf.setCursor(0,120);  lcdDblBuf.printf("PREV");
    lcdDblBuf.setCursor(160,120); lcdDblBuf.printf("RETURN");

    M5.update();
    if (btnAwasPressed = M5.BtnA.wasPressed()) autoExitTime = defaultPowerOffTime + millis();
    if (M5.BtnB.wasPressed()) {
      if (++idxMenu > 12) idxMenu = 0;
      autoExitTime = defaultPowerOffTime + millis();
    }
    
    lcdDblBuf.setTextColor(TFT_BLACK, TFT_CYAN);
    switch (idxMenu) {
      case 0:     //スクリーンの向き
                lcdDblBuf.setCursor(176,0);
                if (btnAwasPressed) {
                  lcdDirection == 1 ? lcdDirection = 3 : lcdDirection = 1;
                  lcdDblBuf.fillSprite(BLACK);
                  M5.Lcd.setRotation(lcdDirection);
                }
                if (lcdDirection == 1)  lcdDblBuf.printf(" LEFT");
                else                    lcdDblBuf.printf("RIGHT");
                break;
      case 1:     //-- 温度アラーム値 On/Off
                if (btnAwasPressed) {
                  tempAlarm = !tempAlarm;
                }
                lcdDblBuf.setCursor(200,20); 
                if (tempAlarm) lcdDblBuf.printf(" ON");
                else           lcdDblBuf.printf("OFF");
                break;
      case 2:     // 温度アラーム　LOW
                if (btnAwasPressed) {
                  if (++tempAlarmLow >= MAXSET_TEMPHIGH) tempAlarmLow = MAXSET_TEMPLOW;
                }
                lcdDblBuf.setCursor(148,40); lcdDblBuf.printf("%3d",tempAlarmLow);      
                break;
      case 3:     // 温度アラーム　HIGH 
                if (btnAwasPressed) {
                  if (++tempAlarmHigh >= MAXSET_TEMPHIGH) tempAlarmHigh = MAXSET_TEMPLOW;  
                }
                lcdDblBuf.setCursor(148 + 12 * 4,40); lcdDblBuf.printf("%3d",tempAlarmHigh);      
                break;
      case 4:     //年
                if (btnAwasPressed) {
                  if (++rtcDate.Year > 2030) rtcDate.Year = 2021;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40,60);   
                lcdDblBuf.printf("%4d",rtcDate.Year);
                break;
      case 5:     //月
                if (btnAwasPressed) {
                  if (++rtcDate.Month > 12) rtcDate.Month = 1;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40 + 12 * 5,60);   
                lcdDblBuf.printf("%02d",rtcDate.Month);
                break;
      case 6:     //日
                if (btnAwasPressed) {
                  if (++rtcDate.Date > monthDay[rtcDate.Month-1]) rtcDate.Date = 1;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40 + 12 * 8,60);   
                lcdDblBuf.printf("%02d",rtcDate.Date);
                break;
      case 7:     //時
                if (btnAwasPressed) {
                  if (++rtcTime.Hours > 23) rtcTime.Hours = 0;
                  M5.Rtc.SetTime(&rtcTime);
                }
                lcdDblBuf.setCursor(40 + 12 * 11,60);   
                lcdDblBuf.printf("%02d",rtcTime.Hours);
                break;
      case 8:     //分
                if (btnAwasPressed) {
                  if (++rtcTime.Minutes > 59) rtcTime.Minutes = 0;
                  M5.Rtc.SetTime(&rtcTime);
                }
                lcdDblBuf.setCursor(40 + 12 * 14,60);   
                lcdDblBuf.printf("%02d",rtcTime.Minutes);
                break;
      case 9:    //デモモード
                if (btnAwasPressed) {
                  if (demoMode != 0) demoMode = 0;
                  else               demoMode = millis() + DEMOTIME;
                }
                lcdDblBuf.setCursor(200,80);  
                if (demoMode != 0) lcdDblBuf.printf(" ON");
                else               lcdDblBuf.printf("OFF");
                break;
      case 10:     // ログ出力
                lcdDblBuf.setCursor(160,100); lcdDblBuf.printf("Serial");
                if (btnAwasPressed) {
//                  M5.Lcd.setTextSize(2);
//                  M5.Lcd.setTextFont(1);
                  for (i = 0; i < MAX_PRESSARRAY; i++) {
                    Serial.printf("                                                     { %2d, %2d, %4d, %2d, %2d },  // %2d:%02d\r\n",
                      presAry[i].day,presAry[i].minutes,presAry[i].pressure,presAry[i].temperature,presAry[i].humidity,
                      int(i/2),i%2*30);
                    
//                    M5.Lcd.setCursor(160,100); M5.Lcd.printf("%6d",i);
                  }
                }
                lcdDblBuf.setCursor(160,100); lcdDblBuf.printf("Serial");
                break;
      case 11: // prev
              lcdDblBuf.setCursor(0,120);   lcdDblBuf.printf("PREV");
              if (btnAwasPressed) menuExit = MENU_PREV;
              break;
      default:
              lcdDblBuf.setCursor(160,120); lcdDblBuf.printf("RETURN");
              if (btnAwasPressed) menuExit = MENU_EXIT;
    }
    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
                
    if (millis() > autoExitTime) menuExit = MENU_EXIT;

    delay(100);
  } while (menuExit == MENU_NOTHING);

  return(menuExit);
}

/*==================================================================
 * 温度・湿度　デカ表示
 */
void DispTempHumi() {
  // 背景
  if (wbgtIndex >= 31)      lcdDblBuf.fillSprite(ToRGB565(192,0,0));
  else if (wbgtIndex >= 28) lcdDblBuf.fillSprite(TFT_OLIVE);
  else                      lcdDblBuf.fillSprite(TFT_DARKCYAN);

  // 気温
  x = 5; y = 1;
  sprintf(tmpStr,"%.1f ",temperature);
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.drawString(tmpStr,x+5,y+6,4);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,x,y,4);
  lcdDblBuf.setTextSize(1);  lcdDblBuf.setTextFont(4); lcdDblBuf.setCursor(160,3); lcdDblBuf.printf("C");

    // アラーム設定温度
  x = 180; y = 0;
  lcdDblBuf.setTextFont(2);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setCursor(x,y);    lcdDblBuf.printf("%3d",tempAlarmHigh);
  lcdDblBuf.setCursor(x,y+30); lcdDblBuf.printf("%3d",tempAlarmLow);
  

  
  
  // 湿度
  x = 5; y = 67;
  sprintf(tmpStr,"%.1f ",humidity);
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.drawString(tmpStr,x+5,y+6,4);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,x,y,4);
  lcdDblBuf.setTextSize(1);  lcdDblBuf.setTextFont(4); lcdDblBuf.setCursor(160,67); lcdDblBuf.printf("%%");

  x = 165; y = 25;

  // 時刻
  x = 15; y = 40;
  sprintf(tmpStr,"%2d:%02d",rtcTime.Hours,rtcTime.Minutes);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));         // 影
  lcdDblBuf.drawString(tmpStr,x+8,y+9,4);           
  lcdDblBuf.drawString(tmpStr,x+10,y+11,4);

  lcdDblBuf.setTextColor(TFT_DARKGREEN);              // 輪郭
  lcdDblBuf.drawString(tmpStr,x+2,y,4);
  lcdDblBuf.drawString(tmpStr,x,y+2,4);
 
  lcdDblBuf.setCursor(x,y,4);
  lcdDblBuf.setTextColor(TFT_GREEN);
  lcdDblBuf.drawString(tmpStr,x,y,4);

  // 電圧
#ifdef DEBUG_MODE
  pwVolt = M5.Axp.GetBatVoltage();
  lcdDblBuf.setTextSize(1);
  lcdDblBuf.setCursor(175,60,4);
  lcdDblBuf.setTextColor(TFT_GREENYELLOW);
  lcdDblBuf.printf("%.2fv",pwVolt);
#endif

  // TVOC eCO2
  if (foundSGP30) {
    x = 170; y = 90;
    lcdDblBuf.setTextSize(1);
    lcdDblBuf.setTextColor(TFT_WHITE);
    lcdDblBuf.setTextFont(2);
    lcdDblBuf.setCursor(x,y);
    if (readySGP30 > 0) {
      lcdDblBuf.printf("Not ready SGP30");
    } else {
      lcdDblBuf.printf("%dppb",sgp30.TVOC);
      lcdDblBuf.setCursor(x,y+15);
      lcdDblBuf.printf("%dppm",sgp30.eCO2);
    }
  }
  
    // BLE deviceID
  x = 210; y = 120;
  lcdDblBuf.setTextFont(2);
  lcdDblBuf.setTextSize(1);
  lcdDblBuf.setTextColor(TFT_CYAN);
  lcdDblBuf.setCursor(x,y);
  if (bleDeviceNumber == 0) lcdDblBuf.printf("OFF");
  else                      lcdDblBuf.printf(" #%d",bleDeviceNumber);

}

/*============================================================================
 * 記録表示
 */
void DispLogList() {
  lcdDblBuf.fillSprite(ToRGB565(128,128,64));

  lcdDblBuf.setTextSize(1);
  lcdDblBuf.setCursor(6,0,2);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.printf("  d      Time   %4dhPa  %2dC  %2d%%",(int)pressure,(int)temperature,(int)humidity);
    // 幅広にし見やすく
    lcdDblBuf.setCursor(7,0,2);
    lcdDblBuf.printf("  d      Time   %4dhPa  %2dC  %2d%%",(int)pressure,(int)temperature,(int)humidity);
    // 表示位置の算出　現在の時刻を最下行にする
  j = CalcPressArrayIndex(rtcTime.Hours,rtcTime.Minutes) - logListOffset * 8;
  lcdDblBuf.setTextSize(2);
  for (i = 7; i >= 0; i--,j--) {
    if (j < 0) j += MAX_PRESSARRAY;
    if (i % 2 == 0) lcdDblBuf.setTextColor(TFT_YELLOW);
    else            lcdDblBuf.setTextColor(TFT_WHITE);
    lcdDblBuf.setCursor(3,15+ 15*i,1);
    lcdDblBuf.printf("%2d %2d:%02d%5d%3d%3d",presAry[j].day,int(j / 2),presAry[j].minutes,presAry[j].pressure,presAry[j].temperature,presAry[j].humidity);
      // 幅広にし見やすく
      lcdDblBuf.setCursor(4,15+ 15*i,1);
      lcdDblBuf.printf("%2d %2d:%02d%5d%3d%3d",presAry[j].day,int(j / 2),presAry[j].minutes,presAry[j].pressure,presAry[j].temperature,presAry[j].humidity);
  }
}
