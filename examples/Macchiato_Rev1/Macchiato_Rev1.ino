//MAcchiato E2B SSD (Rev1) Example
/*Features:
-Integrated E2B
-E2B Transciever via ATtiny85
-DS3231 RTC
-SPI SD card
-MODE button for switching between operating modes
*/
#include <Adafruit_MCP23X17.h>
/*#include "FS.h"
#include "SD.h"
#include "SPI.h"*/
#include "RTClib.h"
//#include "driver/rtc_io.h"
#include <E2B.h>
#include <SoftwareSerial.h>

#define LED_MODE 1
#define MODE_PIN 2

#define E2B_pin 4
#define UART_RX 48
#define UART_TX 45
#define E2B_ALT_DIR 38

#define FLASH0_RY_BY 21
#define FLASH1_RY_BY 47
#define FLASH0_WP 13
#define FLASH1_WP 15
#define SDRAM_CKE 14

#define E2B_pin 4

/*#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define USE_EXT0_WAKEUP          1               // 1 = EXT0 wakeup, 0 = EXT1 wakeup
#define WAKEUP_GPIO              GPIO_NUM_4     // Only RTC IO are allowed - ESP32 Pin example*/

unsigned char rom[8] = {FAMILYCODE, 0xE2, 0xCC, 0x2D, 0x01, 0x25, 0xB8, 0x30};
unsigned char scratchpad[9] = {0x5E, 0x00, 0x00, 0x04, 0x00, 0x31, 0x6A, 0xD7, 0x00};
uint8_t flash_config[2][3] = {
  //CE, OE, WE
  {8,9,12},
  {10,11,14}
};
uint8_t SDRAM_config[9] = {46,18,17,16,15,7,6,5,3}; //LDQM, WE, CAS, RAS, CS, BS0, BS1, UDQM, CLK

uint16_t dataOutgoing;    //originally: int dataOutgoing
String dataInString = "";
uint8_t dataReceived[8];

Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;
Adafruit_MCP23X17 mcp3;
RTC_DS3231 rtc;
E2B e2b(E2B_pin);  // on pin 4 (a 4.7K resistor is necessary)
SoftwareSerial mySerial(UART_RX,UART_TX); // RX, TX

void IRAM_ATTR respond(){
  e2b.MasterResetPulseDetection();
}

void setup() {
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(115200);
  while(!Serial){}
  mySerial.begin(9600);
  Serial.println("E2B SSD Test!");
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);
  pinMode(E2B_ALT_DIR,OUTPUT);

  pinMode(LED_MODE,OUTPUT);
  pinMode(MODE_PIN,INPUT_PULLUP);   //Nominally = 1, pushed = 0

  //Initializes all pin modes
  //pinMode(FLASH0_RY_BY,INPUT);                              //RY_BY output from falsh chip 0
  //pinMode(FLASH1_RY_BY,INPUT);                              //RY_BY output from falsh chip 1
  pinMode(FLASH0_WP,OUTPUT); digitalWrite(FLASH0_WP,HIGH);    //Write protect input to falsh chip 0
  pinMode(FLASH1_WP,OUTPUT); digitalWrite(FLASH1_WP,HIGH);    //Write protect input to falsh chip 1
  pinMode(SDRAM_CKE,OUTPUT); digitalWrite(SDRAM_CKE,HIGH);    //Clock enable input to sdram chip

  //Configures deep sleep mode
  /*esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO,0);  //1 = High, 0 = Low
  // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
  // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
  // No need to keep that power domain explicitly, unlike EXT1.
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);*/

  //Initializes the IO expanders
  if(!mcp1.begin_I2C(0x20)){
    Serial.println("Error initializing MCP1.");
    while(1);
  }
  if(!mcp2.begin_I2C(0x21)){
    Serial.println("Error initializing MCP2.");
    while(1);
  }
  if(!mcp3.begin_I2C(0x22)){
    Serial.println("Error initializing MCP3.");
    while(1);
  }

  for (int i=0; i < 16; i++){
    mcp1.pinMode(i, OUTPUT);
    mcp2.pinMode(i, OUTPUT);
    if(i < 3){
      pinMode(flash_config[0][i], OUTPUT);
      pinMode(flash_config[1][i], OUTPUT);
    }
    if(i < 9){
      pinMode(SDRAM_config[i], OUTPUT);
      pinMode(SDRAM_config[i], OUTPUT);
    }
  }

  //Initializes the SD card
  /*if(!SD.begin()){
    Serial.println("Card Mount Failed");
    while(1);
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  listDir(SD, "/", 0);
  createDir(SD, "/mydir");
  listDir(SD, "/", 0);
  removeDir(SD, "/mydir");
  listDir(SD, "/", 2);
  writeFile(SD, "/hello.txt", "Hello ");
  appendFile(SD, "/hello.txt", "World!\n");
  readFile(SD, "/hello.txt");
  deleteFile(SD, "/foo.txt");
  renameFile(SD, "/hello.txt", "/foo.txt");
  readFile(SD, "/foo.txt");
  testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));*/

  //Initializes the RTC
  if(!rtc.begin()){
    Serial.println("Couldn't find RTC");
    while(1);
  }

  /*Serial.println("Going to sleep now");
  esp_deep_sleep_start();*/

  //ssd_write_flash(1,receivedInt,0x4A3C);
  //dataOutgoing = ssd_read_flash(1,receivedInt);
  //dataOutgoing = 0x4A3D;
  //ssd_write_sdram(81625,0,0x4A3D);
  //ssd_read_sdram(81625,0);
}

void loop(){
  MODE_button_manager();
  receive_packet();
  transmit_packet();
}



//Handles switching through modes of operations via the MODE button
void MODE_button_manager(){
  if(!digitalRead(MODE_PIN)){
    int Tstart = 0;
    while(!digitalRead(MODE_PIN)){
      Tstart++;
    }
    Serial.print("Tstart: "); Serial.println(Tstart);
    if(Tstart >= 500){
      //Do stuff
    }
  }
}

//Receives UART packet
bool receive_packet(){
  digitalWrite(E2B_ALT_DIR,HIGH);
  if (mySerial.available() == 8){
    int receivedInt = 0;
    Serial.print("Received Data: ");
    for (int i=0; i < 8; i++) {
      dataReceived[i] = mySerial.read();
      Serial.print(dataReceived[i],HEX); Serial.print(" ");
      if((i >= 1) && (i <= 4)){
        if(i >= 3){
          receivedInt |= (int32_t)(dataReceived[i] << ((i-1) * 8)); // Combine bytes
        }else{
          receivedInt |= (dataReceived[i] << ((i-1) * 8)); // Combine bytes
        }
      }
    }
    Serial.println();

    if(dataReceived[7] != 0x0){         //Data corruption detected, should be 0x0
    Serial.println(":(");
      return 0;
    }

    if(dataReceived[0] == 0xA){         //Write
      uint16_t dataToWriteToMemory = dataReceived[5]<<8 | dataReceived[6];
      ssd_write_flash(1,receivedInt,dataToWriteToMemory);
    }else if(dataReceived[0] == 0xB){   //Read
      dataOutgoing = ssd_read_flash(1,receivedInt);
    }else{
      return 0;
    }
  /*if (mySerial.available() == 4){
    int receivedInt = 0;
    for (int i=0; i < 4; i++) {
      byte byteRead = mySerial.read();
      if(i >= 2){
        receivedInt |= (int32_t)(byteRead << (i * 8)); // Combine bytes
      }else{
        receivedInt |= (byteRead << (i * 8)); // Combine bytes
      }
    }*/
    //Serial.println(receivedInt);
  }
}

//Transmits UART packet
void transmit_packet(){
  digitalWrite(E2B_ALT_DIR,LOW);
  mySerial.write(lowByte(dataOutgoing));
  mySerial.write(highByte(dataOutgoing));
  digitalWrite(E2B_ALT_DIR,HIGH);
}

//Prints the address onto the flash chip address lines
void ssd_setAddress_flash(int addr){
  for (int i=0; i < 16; i++){
    mcp1.pinMode(i,OUTPUT);
    mcp1.digitalWrite(i,bitRead(addr,i));
    if(i < 8){
      mcp2.pinMode(i,OUTPUT);
      mcp2.digitalWrite(i,bitRead(addr,i+16));
    }
  }
}

//Prints the address onto the flash chip address lines
void ssd_setAddress_sdram(int addr){
  for (int i=0; i < 12; i++){
    mcp1.pinMode(i,OUTPUT);
    mcp1.digitalWrite(i,bitRead(addr,i));
  }
}

//Reads a two-byte value at an address
uint16_t ssd_read_flash(bool chip, int addr){
  ssd_setAddress_flash(addr);
  Serial.print("Reading Flash Address: "); Serial.println(addr);
  
  //Configures the flash chip for reading
  int CE = flash_config[chip][0];
  int OE = flash_config[chip][1];
  int WE = flash_config[chip][2];
  mcp2.digitalWrite(CE,LOW);
  mcp2.digitalWrite(OE,LOW);
  mcp2.digitalWrite(WE,HIGH);
  delay(1);

  uint16_t val = 0;
  for (int i=0; i < 16; i++){
    mcp3.pinMode(i, INPUT);
    val += (mcp3.digitalRead(i) * pow(2,i));
    //Serial.print(i); Serial.print(", "); Serial.print(bitRead(255,i)); Serial.print(", "); Serial.print(mcp3.digitalRead(i)); Serial.print(", "); Serial.println(val);
  }

  Serial.print("Data Read:  ");
  /*int dat[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
  Serial.print("Data Read:  ");
  for (int i=15; i > -1; i--){
    Serial.print(mcp3.digitalRead(dat[i]));
  }
  Serial.print(" ("); Serial.print(val); Serial.println(")");*/
  Serial.println(val,HEX);

  mcp2.digitalWrite(CE,HIGH);
  mcp2.digitalWrite(OE,HIGH);

  return val;
}

//Writes a two-byte value to an address
void ssd_write_flash(bool chip, int addr, int val){
  Serial.print("Writing: "); Serial.print(val,HEX); Serial.print(" to flash address: "); Serial.println(addr);
  ssd_setAddress_flash(addr);
  
  //Configures the flash chip for writing
  int CE = flash_config[chip][0];
  int OE = flash_config[chip][1];
  int WE = flash_config[chip][2];
  mcp2.digitalWrite(CE,LOW);
  mcp2.digitalWrite(OE,HIGH);
  mcp2.digitalWrite(WE,LOW);

  for (int i=0; i < 16; i++){
    mcp3.pinMode(i, OUTPUT);
    mcp3.digitalWrite(i,bitRead(val,i));
  }

  /*int dat[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
  Serial.print("Data Write: ");
  for (int i=15; i > -1; i--){
    Serial.print(mcp3.digitalRead(dat[i]));
  }
  Serial.print(" ("); Serial.print(val); Serial.println(")");*/

  mcp2.digitalWrite(CE,HIGH);
  mcp2.digitalWrite(WE,HIGH);
}

//Reads a byte value of the data at an address
/*uint8_t ssd_read_flash_byte(bool byteNum, int addr){
  //Configures the flash chip for reading
  int CE = flash_config[0][0];
  int OE = flash_config[0][1];
  int WE = flash_config[0][2];
  mcp2.digitalWrite(CE,LOW);
  mcp2.digitalWrite(OE,LOW);
  mcp2.digitalWrite(WE,HIGH);
  int i;

  ssd_setAddress(addr);

  bool val1;
  uint8_t val = 0;
  for (i=0; i < 8; i++){
    mcp3.pinMode(i+(byteNum * 8), INPUT);
    val1 = mcp3.digitalRead(i+(byteNum * 8));
    val += pow(val1,i+(byteNum * 8));
  }

  delay(5);
  mcp2.digitalWrite(CE,HIGH);

  return val;
}*/

//Reads a two-byte value at an address
uint16_t ssd_read_sdram(int addr, bool bank){
  ssd_setAddress_sdram(addr);
  Serial.print("Reading SDRAM Address: "); Serial.println(addr);

  //Configures the sdram chip for reading
  int LDQM = SDRAM_config[0];
  int WE = SDRAM_config[1];
  int CAS = SDRAM_config[2];
  int RAS = SDRAM_config[3];
  int CS = SDRAM_config[4];
  int BS0 = SDRAM_config[5];
  int BS1 = SDRAM_config[6];
  int UDQM = SDRAM_config[7];
  int CLK = SDRAM_config[8];
  digitalWrite(LDQM,HIGH);
  digitalWrite(WE,HIGH);
  digitalWrite(CAS,LOW);
  digitalWrite(RAS,HIGH);
  digitalWrite(CS,LOW);
  digitalWrite(UDQM,HIGH);
  digitalWrite(CLK,HIGH);

  if(bank){
    digitalWrite(BS0,HIGH);
    digitalWrite(BS1,LOW);
  }else{
    digitalWrite(BS0,LOW);
    digitalWrite(BS1,HIGH);
  }
  delay(1);

  uint16_t val = 0;
  pinMode(39, INPUT);
  val += (digitalRead(39) * pow(2,0));
  for (int i=0; i < 16; i++){
    mcp3.pinMode(i, INPUT);
    val += (mcp3.digitalRead(i) * pow(2,i));
    //Serial.print(i); Serial.print(", "); Serial.print(bitRead(255,i)); Serial.print(", "); Serial.print(mcp3.digitalRead(i)); Serial.print(", "); Serial.println(val);
  }

  Serial.print("Data Read:  ");
  /*int dat[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
  Serial.print("Data Read:  ");
  for (int i=15; i > -1; i--){
    Serial.print(mcp3.digitalRead(dat[i]));
  }
  Serial.print(" ("); Serial.print(val); Serial.println(")");*/
  Serial.println(val,HEX);

  digitalWrite(LDQM,HIGH);
  digitalWrite(WE,HIGH);
  digitalWrite(CAS,HIGH);
  digitalWrite(RAS,HIGH);
  digitalWrite(CS,HIGH);
  digitalWrite(BS0,HIGH);
  digitalWrite(BS1,HIGH);
  digitalWrite(UDQM,HIGH);
  digitalWrite(CLK,HIGH);

  return val;
}

//Writes a two-byte value to an address
void ssd_write_sdram(int addr, bool bank, uint16_t val){
  Serial.print("Writing: "); Serial.println(val,HEX);  Serial.print(" to flash address: "); Serial.println(addr);
  ssd_setAddress_sdram(addr);
  
  //Configures the sdram chip for writing
  int LDQM = SDRAM_config[0];
  int WE = SDRAM_config[1];
  int CAS = SDRAM_config[2];
  int RAS = SDRAM_config[3];
  int CS = SDRAM_config[4];
  int BS0 = SDRAM_config[5];
  int BS1 = SDRAM_config[6];
  int UDQM = SDRAM_config[7];
  int CLK = SDRAM_config[8];
  digitalWrite(LDQM,HIGH);
  digitalWrite(WE,LOW);
  digitalWrite(CAS,LOW);
  digitalWrite(RAS,HIGH);
  digitalWrite(CS,LOW);
  digitalWrite(UDQM,HIGH);
  digitalWrite(CLK,HIGH);

  if(bank){
    digitalWrite(BS0,HIGH);
    digitalWrite(BS1,LOW);
  }else{
    digitalWrite(BS0,LOW);
    digitalWrite(BS1,HIGH);
  }

  pinMode(39, OUTPUT);
  digitalWrite(39,bitRead(val,0));
  for (int i=0; i < 16; i++){
    mcp3.pinMode(i, OUTPUT);
    mcp3.digitalWrite(i,bitRead(val,i));
  }

  /*int dat[16] = {0,1,2,3,4,5,6,7,15,14,13,12,11,10,9,8};
  Serial.print("Data Write: ");
  for (int i=15; i > -1; i--){
    Serial.print(mcp3.digitalRead(dat[i]));
  }
  Serial.print(" ("); Serial.print(val); Serial.println(")");*/

  digitalWrite(LDQM,HIGH);
  digitalWrite(WE,HIGH);
  digitalWrite(CAS,HIGH);
  digitalWrite(RAS,HIGH);
  digitalWrite(CS,HIGH);
  digitalWrite(BS0,HIGH);
  digitalWrite(BS1,HIGH);
  digitalWrite(UDQM,HIGH);
  digitalWrite(CLK,HIGH);
}