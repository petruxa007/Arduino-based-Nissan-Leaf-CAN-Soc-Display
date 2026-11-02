
#include <U8g2lib.h>           // Download code from https://github.com/olikraus/u8g2 and add to Ardunio libraries folder
#include <mcp_can.h>           // Download code from https://github.com/coryjfowler/MCP_CAN_lib add to Ardunio libraries folder
#include <EEPROM.h>            // The Arduino IDE already has this so you don't need to download it. 
#include <Wire.h>
#include <stdint.h>
#include <string.h>
#include "battery_large.h"     // large outline battery image (there must be a copy of this in with the LeafSOC code)
#include "battery_solid.h"     // small solid battery image (there must be a copy of this in with the LeafSOC code) 
#include "battery_small.h" 

#define DATE    "    NISSAN"

#define KM_PER_KWH 6.0F        // km per kWh (from my car) used to calculate Range estimate (6.4 km.kWh is my average for the coldest month of 2019)
#define MAX_GIDS 650             // From my car. LeafSpy showed 225 Gids at 94.8% SOC on 12 Oct 2019.
#define GIDS_TURTLE 8          // the number of Gids at which Turtle mode kicks in (you might achieve 5-7 sometimes).
#define WH_PER_GID 77.0F       // Wh per Gid.  74.73 comes from one of the early Leaf hackinG pioneers. (Some people prefer 80.)
#define CAN0_INT 2             // [Why is this set using a define statement? Why bother?]
#define LINE1 16
#define LINE2 38
#define LINE3 60
#define LINE4 64

/* ===== Фильтр температуры CAN 0x6A0 ===== */
#define TEMP_FILTER_SAMPLES       3
#define TEMP_JUMP_THRESHOLD_C     5
#define TEMP_CONFIRM_SAMPLES      3
#define TEMP_CANDIDATE_TOLERANCE  2

long unsigned int rxId;        // CAN message ID
unsigned char len = 0;         // data length
unsigned char rxBuf[8];        // a buffer for CAN data
uint16_t rawGids = 0;          // raw Gids
uint16_t rawGids2 = 0;         // compare with rawGids to see if value has changed
float BattVolts;               // Main battery voltage
int16_t Amp;                   
float Amps;                    // Main battery current
float kW;                      // Power used from main battery
float rawGidsLow;              // raw Gids - 8 (the number of Gids to reach Turtle mode)
float GidsPct;                 // Gids in percentage, This is the main battery SOC % number that I display
uint16_t rawSoc;               // State of charge value directly from the EV-CAN
float SocPct;                  // Leaf SOC% from EV-Can. This corresponds to the LeafSpy app SOC%, but without rounding up
float SocPctSkewed;            // Skewing the top to show "100%" when battery is actually 95% and 0% when battery is 5%.
float kWh;                     // Energy left in main batery
float km_per_kWh = KM_PER_KWH;
uint16_t km_per_dWh = KM_PER_KWH * 10; // using "deca"-Watthour because it's more efficient in EEPROM
uint16_t MaxGids = MAX_GIDS;       // Default to MAX_GIDS value before reading the EEPROM
uint16_t InitialGids = 0;          // Will be set to the first value of Gids read from the Can bus on startup
// byte GidTest = 2;              // temporary value used for comparing Gids in EEPROM to find highest value Gid
int range;                     // My esitmated range
char buffer[8];
int i;
int Page = 1;                  // Screen page; default to Page 1 before reading the EEPROM
byte BootCount = 5;

byte EEPROMaddr0 = 0;          // Address 0 in EEPROM used for the page number
byte EEPROMaddr1 = 1;          // Address 1 in EEPROM used for km_per_kWh
byte EEPROMaddr2 = 2;          // Address 2 in EEPROM used for MaxGids [I should upgrade this to a 2 byte INT at some point]
byte EEPROMaddr4 = 4;          // Boot counter
//                             // EEPROMaddr 5 to EEPROMaddr 95 used for Gids values from start-up (for 60 boots, then cyles around)

// ================== Переменные для режима выбора ==================
bool selectMode = false;       // Флаг: мы в режиме выбора
byte selectPage = 1;           // Временный выбор страницы
byte selectPage2 = 0;          // Для обнаружения изменения выбранной страницы
byte pnSequence = 0;           // Для отслеживания комбинации P-N-P-N
unsigned long lastGearChange = 0;

byte rawCCStatus1;
byte rawCCVentIntake;
byte rawCCVentTarget;
byte rawCCFanSpeed;
byte rawCCButtonPress;
byte rawCCButtonPress2;
byte rawCCSetpoint;
byte rawGearPos = 1;
byte rawGearPos2 = 1;

// Battery telemetry decoded from CAN frame 0x6A0
int8_t vcmAverageTempC = 0;
int8_t tempHistory[TEMP_FILTER_SAMPLES] = {0};
uint8_t tempHistoryIndex = 0;
bool tempFilterInitialized = false;
int16_t tempCandidateSum = 0;
int8_t tempCandidate = 0;
uint8_t tempCandidateCount = 0;

uint8_t vcmCapacityAh = 0;
uint8_t vcmCellImbalanceMv = 0;
uint16_t vcmCellMinMv = 0;
uint16_t vcmCellMaxMv = 0;
bool vcmContactorsOn = false;
bool vcmFullCharge = false;
bool vcmLowLimitLearned = false;
bool vcmHighLimitLearned = false;
bool vcmTelemetryValid = false;
bool vcmTelemetryUpdated = false;
bool displayDataUpdated = false;

MCP_CAN CAN0(10);              // Set MCP CS to pin 10

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void(* resetFunc) (void) = 0;

// --- FIFO настройки ---
#define CAN_FIFO_SIZE 16 
struct CanMessage {
  long unsigned int id;
  byte len;
  byte buf[8];
};

CanMessage canFifo[CAN_FIFO_SIZE];
volatile byte fifoHead = 0; 
volatile byte fifoTail = 0; 

// --- Функции работы с FIFO ---
bool fifoPush(long unsigned int id, byte len, byte* buf) {
  byte next = (fifoHead + 1) % CAN_FIFO_SIZE;
  if (next == fifoTail) return false;
  canFifo[fifoHead].id = id;
  canFifo[fifoHead].len = len;
  memcpy(canFifo[fifoHead].buf, buf, len);
  fifoHead = next;
  return true;
}

bool fifoPop(CanMessage &msg) {
  if (fifoTail == fifoHead) return false;
  msg = canFifo[fifoTail];
  fifoTail = (fifoTail + 1) % CAN_FIFO_SIZE;
  return true;
}

int8_t filterBatteryTemperature(int8_t rawTemp) {
  // Первое измерение используем для начальной инициализации.
  if (!tempFilterInitialized) {
    for (uint8_t i = 0; i < TEMP_FILTER_SAMPLES; i++) {
      tempHistory[i] = rawTemp;
    }

    vcmAverageTempC = rawTemp;
    tempHistoryIndex = 0;
    tempFilterInitialized = true;

    tempCandidateCount = 0;
    tempCandidateSum = 0;

    return vcmAverageTempC;
  }

  // Разница между новым значением и текущей температурой.
  int16_t difference = (int16_t)rawTemp - vcmAverageTempC;
  if (difference < 0) difference = -difference;

  // Проверяем подозрительный резкий скачок.
  if (difference > TEMP_JUMP_THRESHOLD_C) {

    // Первое подозрительное измерение становится кандидатом.
    if (tempCandidateCount == 0) {
      tempCandidate = rawTemp;
      tempCandidateSum = rawTemp;
      tempCandidateCount = 1;
      return vcmAverageTempC;
    }

    // Следующие измерения должны быть близки к кандидату.
    int16_t candidateDifference =
        (int16_t)rawTemp - tempCandidate;

    if (candidateDifference < 0) {
      candidateDifference = -candidateDifference;
    }

    if (candidateDifference <= TEMP_CANDIDATE_TOLERANCE) {
      tempCandidateSum += rawTemp;
      tempCandidateCount++;

      // Три согласующихся измерения подтверждают изменение.
      if (tempCandidateCount >= TEMP_CONFIRM_SAMPLES) {
        int8_t confirmedTemp =
            (int8_t)(tempCandidateSum / tempCandidateCount);

        // Начинаем новую историю с подтверждённой температуры.
        for (uint8_t i = 0; i < TEMP_FILTER_SAMPLES; i++) {
          tempHistory[i] = confirmedTemp;
        }

        tempHistoryIndex = 0;
        vcmAverageTempC = confirmedTemp;

        tempCandidateCount = 0;
        tempCandidateSum = 0;
      }
    } else {
      // Кандидат изменился: начинаем подтверждение заново.
      tempCandidate = rawTemp;
      tempCandidateSum = rawTemp;
      tempCandidateCount = 1;
    }

    return vcmAverageTempC;
  }

  // Значение вернулось к нормальному диапазону.
  // Отбрасываем неподтверждённый скачок.
  tempCandidateCount = 0;
  tempCandidateSum = 0;

  // Сохраняем три последних нормальных измерения.
  tempHistory[tempHistoryIndex] = rawTemp;

  tempHistoryIndex++;
  if (tempHistoryIndex >= TEMP_FILTER_SAMPLES) {
    tempHistoryIndex = 0;
  }

  // Рассчитываем среднее значение.
  int16_t tempSum = 0;

  for (uint8_t i = 0; i < TEMP_FILTER_SAMPLES; i++) {
    tempSum += tempHistory[i];
  }

  vcmAverageTempC =
      (int8_t)(tempSum / TEMP_FILTER_SAMPLES);

  return vcmAverageTempC;
}

// EEPROM stores MaxGids as a 16-bit value at addresses 2-3.
// Address 4 remains reserved for BootCount; addresses 5+ remain the boot history.
uint16_t readMaxGidsFromEEPROM() {
  uint16_t value = (uint16_t)EEPROM.read(EEPROMaddr2) |
                   ((uint16_t)EEPROM.read(3) << 8);
  return value;
}

void writeMaxGidsToEEPROM(uint16_t value) {
  EEPROM.update(EEPROMaddr2, (byte)(value & 0xFF));
  EEPROM.update(3, (byte)(value >> 8));
}

void setup() {
  u8g2.begin();
  u8g2.setContrast(20);
  u8g2.setFont(u8g2_font_logisoso16_tr);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, LINE2); u8g2.print(DATE);
    } while ( u8g2.nextPage() );
    delay(500); // was 1000 originally. If this is reduced to 0 the MCP2515 will sometimes fart out 1023 at startup)
  }else{
    u8g2.firstPage();
    do {
      u8g2.setFontMode(0); // Inverse font mode
      u8g2.setDrawColor(1); // White screen colour
      u8g2.drawBox(0, 0, 128, 64); // draw white box
      u8g2.setDrawColor(0);
      u8g2.setCursor(31, 18); u8g2.print("MCP2515");
      u8g2.setCursor(3, 40); u8g2.print("(CAN module)");
      u8g2.setCursor(15, 61); u8g2.print("start error");
    } while ( u8g2.nextPage() );
    delay(10000);
  }
      // Настройка масок и фильтров
  // -------------------------------
  // Маска 0 → фильтры 0 и 1 
  // 0x7FF = проверяем все 11 бит стандартного ID
  CAN0.init_Mask(0, 0, 0x7FF);
  CAN0.init_Filt(0, 0, 0x11a); 
  CAN0.init_Filt(1, 0, 0x1db);
  // Маска 1 → фильтры 2,3,4,5 (температуры и другие)
  CAN0.init_Mask(1, 0, 0x7FF);
  CAN0.init_Filt(2, 0, 0x5bc); 
  CAN0.init_Filt(3, 0, 0x55b); 
  CAN0.init_Filt(4, 0, 0x6A0);  
  CAN0.init_Filt(5, 0, 0x6A0); 

  CAN0.setMode(MCP_NORMAL);                 // Set to normal mode so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);

  if (EEPROM.read(EEPROMaddr0) > 6 ) {
    EEPROM.update(EEPROMaddr0, Page);
  }
  if (EEPROM.read(EEPROMaddr1) > 40  && EEPROM.read(EEPROMaddr1) < 100 ) {
    km_per_kWh = (EEPROM.read(EEPROMaddr1)) / 10.0F;
  }else{
    km_per_kWh = KM_PER_KWH;
    km_per_dWh = (uint16_t)(KM_PER_KWH * 10.0F);
    EEPROM.update(EEPROMaddr1, (byte)km_per_dWh); // first-run default
  }
  // Load MaxGids without truncating it to one byte.
  // If EEPROM is uninitialized or contains an implausible value, seed it with MAX_GIDS.
  uint16_t storedMaxGids = readMaxGidsFromEEPROM();
  if (storedMaxGids >= 100 && storedMaxGids <= 1000) {
    MaxGids = storedMaxGids;
  }else{
    MaxGids = MAX_GIDS;
    writeMaxGidsToEEPROM(MaxGids);
  }
  if (EEPROM.read(EEPROMaddr4) > 64 ) {
    EEPROM.update(EEPROMaddr4, BootCount); // only used for the first time you run this code on a new Arduino
  }else{
    BootCount = EEPROM.read(EEPROMaddr4) + 1;
    EEPROM.update(EEPROMaddr4, BootCount);
  }
}


void loop() {
  Setup_mode_check(); 
    // --- Считываем все доступные CAN-сообщения и кладем в FIFO ---
  while (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    fifoPush(rxId, len, rxBuf);  // помещаем в FIFO
  }
  // --- Обрабатываем сообщения из FIFO по очереди ---
  CanMessage msg;
  while(fifoPop(msg)) {
    processCanMessage(msg.id, msg.len, msg.buf);
  }
  // If rawGids has a value and InitialGids is still zero then write first Gids value into EEPROM MaxGids array
  if (rawGids != 0 && InitialGids == 0) { // if InitialGids has not yet been set, then set it
    InitialGids = rawGids;
    EEPROM.update(BootCount, InitialGids);
  }
  // if Gid have changed or a CC button was pressed or the Gear Position changes then print to the display
  // Refresh live CAN values at most four times per second.
  static unsigned long lastDisplayRefresh = 0;
  bool livePage = (Page >= 1 && Page <= 5);
  bool liveRefreshDue = livePage && displayDataUpdated && (unsigned long)(millis() - lastDisplayRefresh) >= 150UL;
  bool displayNeedsRedraw = (rawGids2 != rawGids || rawCCButtonPress2 != rawCCButtonPress || rawGearPos2 != rawGearPos || (Page == 6 && vcmTelemetryUpdated) || liveRefreshDue || (selectMode && selectPage2 != selectPage));
  if (displayNeedsRedraw) {
    if (selectMode) {
      u8g2.firstPage();
      do {
        // Отдельный экран меню: не рисуем поверх текущей страницы.
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawFrame(0, 0, 128, 64);
        u8g2.setCursor(8, 13); u8g2.print(F("SELECT PAGE"));
        u8g2.drawHLine(4, 17, 120);
        u8g2.setFont(u8g2_font_logisoso16_tr);
        u8g2.setCursor(8, 38);
        u8g2.print(selectPage);
        u8g2.print(F(" / 6"));
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setCursor(70, 37);
        switch (selectPage) {
          case 1: u8g2.print(F("Range/SOC")); break;
          case 2: u8g2.print(F("Power")); break;
          case 3: u8g2.print(F("Range")); break;
          case 4: u8g2.print(F("Battery")); break;
          case 5: u8g2.print(F("Telemetry")); break;
          case 6: u8g2.print(F("Diag")); break;
        }
        u8g2.setCursor(8, 51); u8g2.print(F("R: PREV   D: NEXT"));
        u8g2.setCursor(8, 61); u8g2.print(F("P: SELECT"));
      } while (u8g2.nextPage());
    }
    else if(Page == 0) { // RESET the Arduino
      EEPROM.update(EEPROMaddr0, 1);
      resetFunc();
    }
    else if (Page == 1) {
      u8g2.firstPage();
      do {
        Setup_mode_check();
        u8g2.drawXBM( 0, 40, 56, 24, battery_solid_bits);
        u8g2.setFont(u8g2_font_logisoso32_tn); // Large numerals-only font
        u8g2.setCursor(35, 32);
        dtostrf(range, 3, 0, buffer);
        u8g2.print(buffer);
        u8g2.setFont(u8g2_font_logisoso16_tr); // Medium font
        u8g2.setCursor(0, 24);   u8g2.print("");
        u8g2.setCursor(103, 24); u8g2.print("km");
        u8g2.setCursor(5, LINE3);
        u8g2.setFontMode(1); // Black text
        u8g2.setDrawColor(2); // white background
        if(rawGearPos == 1){
          dtostrf(GidsPct, 3, 0, buffer);   // Battery SOC%
          u8g2.print(buffer);
          u8g2.print("%"); // Battery SOC%
        }else{
          dtostrf(BattVolts, 3, 0, buffer);   //Battery voltage
          u8g2.print(buffer);
          u8g2.print("V"); // Battery voltage
        }
        u8g2.setFontMode(0);
        u8g2.setCursor(72, LINE3);
        if (kWh >= 10) {
          u8g2.setCursor(62, LINE3);
        }
        dtostrf(kWh, 3, 1, buffer);
        u8g2.print(buffer);
        u8g2.print("kWh");     // kWh calculated from Gids
      } while ( u8g2.nextPage() );
    }
    else if (Page == 2) {
      u8g2.firstPage();
      do {
        Setup_mode_check();

        u8g2.setFont(u8g2_font_logisoso32_tn); // Large numerals-only font
        u8g2.setCursor(10, 32);
        dtostrf(kW, 3, 0, buffer);
        u8g2.print(buffer);
        u8g2.setFont(u8g2_font_logisoso16_tr); // Medium font
        //u8g2.setCursor(0, 24);   u8g2.print("Range");
        u8g2.setCursor(90, 24); u8g2.print("KW");
        u8g2.setCursor(5, LINE3);

        u8g2.setCursor(5, LINE3);
        dtostrf(BattVolts, 3, 0, buffer);
        u8g2.print(buffer);
        u8g2.print("V"); // Battery voltage

        u8g2.setFontMode(0);
        u8g2.setCursor(82, LINE3);
        if (Amps >= 10) {
          u8g2.setCursor(72, LINE3);
        }
        dtostrf(Amps, 3, 1, buffer);
        u8g2.print(buffer);
        u8g2.print("A");     // curent
      } while ( u8g2.nextPage() );
    }
    else if (Page == 3) {
      u8g2.firstPage();
      do {
        Setup_mode_check();
        u8g2.setFont(u8g2_font_logisoso32_tn); // Large numerals-only font
        u8g2.setCursor(35, 47);
        dtostrf(range, 3, 0, buffer);
        u8g2.print(buffer);  // u8g2.print(Range);
        u8g2.setFont(u8g2_font_logisoso16_tr); // Medium font
        u8g2.setCursor(0, 39);   u8g2.print(""); // Range
        u8g2.setCursor(103, 39); u8g2.print("km"); // Range
      } while ( u8g2.nextPage() );
    }
    else if (Page == 4) {
      u8g2.firstPage();
      do {
        Setup_mode_check();

        u8g2.drawCircle(64, 32, 40);

        char valueText[16];
        int16_t totalWidth;
        int16_t x;

        // Напряжения ячеек в верхних углах — точность 0.01 В
        u8g2.setFont(u8g2_font_5x7_tr);

        dtostrf(vcmCellMinMv / 1000.0F, 1, 2, valueText);
        u8g2.setCursor(0, 8);
        u8g2.print(valueText);
        u8g2.print("V");

        dtostrf(vcmCellMaxMv / 1000.0F, 1, 2, valueText);
        totalWidth = u8g2.getStrWidth(valueText) + u8g2.getStrWidth("V");
        x = 127 - totalWidth;
        u8g2.setCursor(x, 8);
        u8g2.print(valueText);
        u8g2.print("V");

        // Температура батареи — целое число
        u8g2.setFont(u8g2_font_logisoso16_tr);

        dtostrf((float)vcmAverageTempC, 1, 0, valueText);
        totalWidth = u8g2.getStrWidth(valueText) + u8g2.getStrWidth("C");
        x = (128 - totalWidth) / 2;
        u8g2.setCursor(x, LINE1);
        u8g2.print(valueText);
        u8g2.print("C");

        // Ёмкость — один знак после запятой
        dtostrf(kWh, 1, 1, valueText);
        totalWidth = u8g2.getStrWidth(valueText) + u8g2.getStrWidth("kWh");
        x = (128 - totalWidth) / 2;
        u8g2.setCursor(x, LINE2);
        u8g2.print(valueText);
        u8g2.print("kWh");

        // Текущая мощность — целое число
        dtostrf(kW, 1, 0, valueText);
        totalWidth = u8g2.getStrWidth(valueText) + u8g2.getStrWidth("kW");
        x = (128 - totalWidth) / 2;
        u8g2.setCursor(x, LINE3);
        u8g2.print(valueText);
        u8g2.print("kW");

        // Нижние углы — напряжение батареи и ток, целые числа
        u8g2.setFont(u8g2_font_5x7_tr);

        // BattVolts — левый нижний угол
        dtostrf(BattVolts, 1, 0, valueText);
        u8g2.setCursor(0, 63);
        u8g2.print(valueText);
        u8g2.print("V");

        // Amps — правый нижний угол
        dtostrf(Amps, 1, 0, valueText);
        totalWidth = u8g2.getStrWidth(valueText) + u8g2.getStrWidth("A");
        x = 126 - totalWidth;
        u8g2.setCursor(x, 63);
        u8g2.print(valueText);
        u8g2.print("A");

      } while (u8g2.nextPage());
    }
    else if (Page == 5) {
      u8g2.firstPage();
      do {
        Setup_mode_check();
        if (!vcmTelemetryValid) {
          u8g2.setCursor(0, 19); u8g2.print(" Waiting  ");
          u8g2.setCursor(0, 40); u8g2.print("for 0x6A0");
        } else {
          u8g2.drawHLine(0,  0, 128);
          u8g2.drawHLine(0, 21, 128);
          u8g2.drawHLine(0, 42, 128);
          u8g2.drawHLine(0, 63, 128);
          u8g2.drawVLine(0,   0, 63);
          u8g2.drawVLine(63,  0, 63);
          u8g2.drawVLine(127, 0, 63);
          u8g2.setCursor(2, 19); u8g2.print("T:"); u8g2.print(vcmAverageTempC); u8g2.print("C");  u8g2.setCursor(65, 19); u8g2.print("Ah:"); u8g2.print(vcmCapacityAh);
          u8g2.setCursor(2, 40); u8g2.print("V:"); u8g2.print(vcmCellMinMv); u8g2.setCursor(65, 40); u8g2.print("V:"); u8g2.print(vcmCellMaxMv);
          u8g2.setCursor(2, 61); u8g2.print("mV:"); u8g2.print(vcmCellImbalanceMv); 
          u8g2.setCursor(65, 61); u8g2.print(""); u8g2.print(vcmContactorsOn ? "ON" : "OFF"); u8g2.print(" "); u8g2.print(vcmFullCharge ? "ON" : "OFF");
        }
      } while ( u8g2.nextPage() );
    }
    else if (Page == 6) {
      u8g2.firstPage();
      do {
        Setup_mode_check();
        u8g2.drawHLine(0,  0, 128);
        u8g2.drawHLine(0, 21, 128);
        u8g2.drawHLine(0, 42, 128);
        u8g2.drawHLine(0, 63, 128);
        u8g2.drawVLine(0,   0, 63);
        u8g2.drawVLine(63,  0, 63);
        u8g2.drawVLine(127, 0, 63);
        u8g2.setCursor(2, 19); u8g2.print("BC:"); u8g2.print(BootCount);   u8g2.setCursor(65, 19); u8g2.print("rG:"); u8g2.print(rawGids);
        u8g2.setCursor(2, 40); u8g2.print("IG:"); u8g2.print(InitialGids); u8g2.setCursor(65, 40); u8g2.print("MG:"); u8g2.print(MaxGids);
        SocPctSkewed = (SocPct - 8.0F) * 1.1F; // adjusting so that it goes from 0 to 100%
        dtostrf(GidsPct, 3, 0, buffer);
        u8g2.setCursor(2, 61); u8g2.print("mS:"); u8g2.print(buffer);    u8g2.setCursor(65, 61); u8g2.print("sS:");  u8g2.print(SocPctSkewed, 0);
      } while ( u8g2.nextPage() );
    }
  }

  vcmTelemetryUpdated = false;
  if (displayNeedsRedraw) {
    displayDataUpdated = false;
    if (Page >= 1 && Page <= 5) lastDisplayRefresh = millis();
  }
  selectPage2 = selectPage;
  rawGids2 = rawGids;
  rawCCButtonPress2 = rawCCButtonPress;
  rawGearPos2 = rawGearPos;

}

void processCanMessage(long unsigned int rxId, byte len, byte* rxBuf) {
  if ((rxId & 0x40000000) == 0x40000000) return; // remote frame, игнорируем

  if (rxId == 0x6A0 && len >= 8) {
    // Big-endian 32-bit container: min[12:0], followed by max[12:0].
    uint32_t packedVoltages = ((uint32_t)rxBuf[3] << 24) |
                              ((uint32_t)rxBuf[4] << 16) |
                              ((uint32_t)rxBuf[5] << 8)  |
                              (uint32_t)rxBuf[6];
    uint8_t flags = rxBuf[7];
    filterBatteryTemperature((int8_t)rxBuf[0]);
    vcmCapacityAh = rxBuf[1];
    vcmCellImbalanceMv = rxBuf[2];
    vcmCellMinMv = (uint16_t)((packedVoltages >> 13) & 0x1FFFU);
    vcmCellMaxMv = (uint16_t)(packedVoltages & 0x1FFFU);
    vcmContactorsOn = (flags & (1U << 0)) != 0;
    vcmFullCharge = (flags & (1U << 1)) != 0;
    vcmLowLimitLearned = (flags & (1U << 2)) != 0;
    vcmHighLimitLearned = (flags & (1U << 3)) != 0;
    vcmTelemetryValid = true;
    vcmTelemetryUpdated = true;
    return;
  }
  if ((rxId & 0x40000000) == 0x40000000) return; // remote frame, игнорируем

  if (rxId == 0x5bc && len >= 2) { 
    displayDataUpdated = true;
    rawGids = (rxBuf[0] << 2) | (rxBuf[1] >> 6);
    if (MaxGids > GIDS_TURTLE) {
      float usableGids = (rawGids > GIDS_TURTLE)
                           ? (float)(rawGids - GIDS_TURTLE) : 0.0F;
      GidsPct = (usableGids / (float)(MaxGids - GIDS_TURTLE)) * 100.0F;
      if (GidsPct > 100.0F) GidsPct = 100.0F;
      if (GidsPct < 0.0F) GidsPct = 0.0F;
      range = (int)(km_per_kWh * usableGids * WH_PER_GID / 1000.0F);
    } else {
      GidsPct = 0.0F;
      range = 0;
    }
    kWh = ((float)rawGids * WH_PER_GID) / 1000.0F;
  } 
  else if (rxId == 0x55b && len >= 2) {
    displayDataUpdated = true;
    rawSoc = (rxBuf[0] << 2) | (rxBuf[1] >> 6);
    SocPct = rawSoc / 10.0F;
    SocPctSkewed = (SocPct - 5.0F) * 1.1F;
  }
  else if (rxId == 0x11a && len >= 1) { 
    rawGearPos = (rxBuf[0] >> 4);
  }
  else if (rxId == 0x54a && len >= 5) { 
    rawCCSetpoint = (rxBuf[4]);
  }
  else if (rxId == 0x1db && len >= 4) {   
    displayDataUpdated = true;
    BattVolts = (rxBuf[2] << 2) | (rxBuf[3] >> 6);
    BattVolts = BattVolts/2.0F;
    Amp = (rxBuf[0] << 3) | (rxBuf[1] >> 5);
    if (Amp & 0x0400) Amp |= 0xf800;
    Amps = -(Amp / (2.0F));
    kW = (Amps * BattVolts)/1000.0F;
  }
}
void Setup_mode_check() {
  static byte lastGear = 0;
  static unsigned long lastSelectionRepeat = 0;
  const unsigned long SELECTION_REPEAT_MS = 5000;

  unsigned long now = millis();

  if (rawGearPos != lastGear) {
    lastGear = rawGearPos;
    lastGearChange = now;

    // --- Отслеживание комбинации P-N-P-N для входа в меню ---
    if (!selectMode) {
      if ((pnSequence == 0 || pnSequence == 2) && rawGearPos == 1) {
        pnSequence++;
      } else if ((pnSequence == 1 || pnSequence == 3) && rawGearPos == 3) {
        pnSequence++;
      } else {
        pnSequence = 0;
      }

      if (pnSequence >= 4) {
        selectMode = true;
        selectPage = Page;
        selectPage2 = 0; // принудительно обновить меню
        pnSequence = 0;
        lastSelectionRepeat = now;
      }
    } else {
      // При каждом переходе в R/D перемещаемся на одну страницу сразу.
      if (rawGearPos == 2) {
        selectPage = (selectPage <= 1) ? 6 : selectPage - 1;
        lastSelectionRepeat = now;
      } else if (rawGearPos == 4) {
        selectPage = (selectPage >= 6) ? 1 : selectPage + 1;
        lastSelectionRepeat = now;
      } else if (rawGearPos == 1) {
        // P подтверждает выбор.
        Page = selectPage;
        EEPROM.update(EEPROMaddr0, Page);
        selectMode = false;
        pnSequence = 0;
      }
    }
  }

  // Автоповтор, если R или D остаётся выбранным.
  // Первое переключение происходит сразу, последующие — каждые 450 мс.
  if (selectMode && (rawGearPos == 2 || rawGearPos == 4) &&
      (unsigned long)(now - lastSelectionRepeat) >= SELECTION_REPEAT_MS) {
    if (rawGearPos == 2) {
      selectPage = (selectPage <= 1) ? 6 : selectPage - 1;
    } else {
      selectPage = (selectPage >= 6) ? 1 : selectPage + 1;
    }
    lastSelectionRepeat = now;
  }

  // Не перезаписываем Page из EEPROM во время выбора.
  if (!selectMode) {
    byte savedPage = EEPROM.read(EEPROMaddr0);
    if (savedPage <= 6) Page = savedPage;
  }
}

void Draw_dotted_box() {
  for (byte i = 0; i < 63; i += 5) {
    u8g2.drawPixel(0, i);
    u8g2.drawPixel(127, i);
  }
  for (byte i = 0; i < 127; i += 5) {
    u8g2.drawPixel(i, 0);
    u8g2.drawPixel(i, 63);
  }
}

void Clear_EEPROM() {
  if (EEPROM.read(5) == 255) {
    for (i = 6; i <= 94; i++) {
      EEPROM.update(i, 0);
    }
  }
}
