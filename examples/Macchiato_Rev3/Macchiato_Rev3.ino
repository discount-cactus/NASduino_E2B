////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch manages read/write requests from the NAS controller and interfaces with the onboard flash chip
-The flash chip (S29GL01GS10TFI020) has 1GB of flash memory
    -Total Size: 1Gbit = 128MB
    -Sector Size: 64KB (0x10000 bytes)
    -Number of sectors: 2048
    -Sector 0:         Start Address: 0x00000000     End Address: 0x0000FFFF
    -Sector 2047:      Start Address: 0x0FFF0000     End Address: 0x0FFFFFFF
-MODE_PIN pin for the MODE button are reserved for future use and do not currently have purpose
-LED_MODE pin for the LED are reserved for future use and do not currently have purpose
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <E2B.h>
#include <Adafruit_MCP23X17.h>

#define E2B_pin 2
#define MODE_PIN 9
#define MODE_LED 6

//#define FLASH_WP 13
#define FLASH_CE 9
#define FLASH_OE 10
#define FLASH_WE 15
//#define FLASH_RESET 16

unsigned char rom[8] = {FAMILYCODE, 0x45, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t DQ[16] = {0,4,7,10,12,A6,A2,A0,3,5,8,11,A7,A3,A1,13};
uint8_t ssd_status[4] = {0x0,0x0,0x0,0x0};    //Status bits, average service rate (integer, decimal), capacity

uint8_t flash_mfg_id = 0;
uint8_t flash_dev_id = 0;

uint16_t dataOutgoing;    //originally: int dataOutgoing
String dataInString = "";
uint8_t dataReceived[8];

float totalServiceRate = 1.0;
float averageServiceRate = 1.0;
int serviceRateSamples = 0;
int capacity = 12;                    //A percentage out of 100 (ex: capacity = 12 --> 12/100)
uint8_t intervalNum = 0;              //Tracks the number of received packets since the last status update
const uint8_t intervalThreshold = 5; //Sets the number of packets received before updating the status bits

Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;
E2B e2b(E2B_pin);

void setup(){
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Macchiato E2B SSD");
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);
  e2b.attachUserCommand(0x9F,sendStatus);

  pinMode(MODE_PIN,INPUT_PULLUP);   //Nominally = 1, pushed = 0
  pinMode(MODE_LED, OUTPUT);
  digitalWrite(MODE_LED,LOW);

  //Initializes the IO expanders
  if(!mcp1.begin_I2C(0x20)){
    Serial.println("Error initializing MCP1.");
    while(1);
  }
  if(!mcp2.begin_I2C(0x21)){
    Serial.println("Error initializing MCP2.");
    while(1);
  }

  //mcp2.pinMode(FLASH_RESET,OUTPUT); mcp2.digitalWrite(FLASH_RESET,HIGH);    //Write protect input to falsh chip
  for (uint8_t i=0; i < 16; i++){
    mcp1.pinMode(i,OUTPUT);
    //mcp2.pinMode(i, OUTPUT);
    if((i < 8) || (i == 11) || (i == 12)){
      mcp2.pinMode(i,OUTPUT);
    }
  }
  mcp2.pinMode(FLASH_CE, OUTPUT);
  mcp2.pinMode(FLASH_OE, OUTPUT);
  mcp2.pinMode(FLASH_WE, OUTPUT);
  mcp2.digitalWrite(FLASH_CE,LOW);

  get_flash_ids(flash_mfg_id, flash_dev_id);
  ssd_setAddress(0x0000);
  flash_write_cycle(0xF0);  // Exit autoselect mode
  delay(10);
  /*uint16_t test = ssd_read_flash(0x0000);
  digitalWrite(MODE_LED, HIGH);
  if (test == 0x20) {
    delay(500); // Blink long for erased
  } else {
    delay(100); // Blink short for programmed
  }
  digitalWrite(MODE_LED, LOW);*/
  //ssd_erase_sector(0x000000);   //Erase sector 0

  //sendStatus();
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(){
  //MODE_button_manager();
  e2b.waitForRequest(false);
  if (e2b.getScratchpad(4) != 0xBE || e2b.getScratchpad(0) == 0xA || e2b.getScratchpad(0) == 0xB)
    handle_command();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////SUPPORT FUNCTIONS///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Handles switching through modes of operations via the MODE button
void MODE_button_manager(){
  if(!digitalRead(MODE_PIN)){
    int Tstart = 0;
    while(!digitalRead(MODE_PIN)){
      Tstart++;
      delay(1);
    }
    Serial.print("Tstart: "); Serial.println(Tstart);
    if(Tstart >= 500){
      //Do stuff
    }else{
      //Do other stuff
    }
  }
}

//Updates the status of the SSD to be read by the NAS controller
void update_status(){
  Serial.println("---------------------------------------------------------------------------------------------------------------");
  Serial.println("Updating status");
  uint8_t statusBits = 0x0;
  bitWrite(statusBits,0,mcp1.begin_I2C(0x20));
  bitWrite(statusBits,1,mcp2.begin_I2C(0x21));
  
  //Checks memory
  const uint8_t checkAddr = 20;
  uint16_t checkDat = ssd_read_flash(checkAddr);
  ssd_write_flash(checkAddr,0xC2);
  bitWrite(statusBits,3,(bool)ssd_read_flash(checkAddr));
  ssd_write_flash(checkAddr,checkDat);

  ssd_status[0] = statusBits;

  int IntegerPart = (int)(averageServiceRate);
  int DecimalPart = 100 * (averageServiceRate - IntegerPart);
  ssd_status[1] = IntegerPart;
  ssd_status[2] = DecimalPart;

  //capacity = find_capacity();
  capacity = 15;
  ssd_status[3] = capacity;

  Serial.print("Status bits: ");
  for(int i=0; i < 4; i++){
    Serial.print(ssd_status[i],BIN); Serial.print(" ");
  }
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------------------------------------");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////COMMUNICATIONS FUNCTIONS//////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Handles commands sent by the NAS controller
void handle_command(){
  uint8_t dataE2B[8];
  uint32_t receivedAddress = 0;    //originally int receivedAddress = 0;
  float runtime = 0.0;

  runtime = millis();
  //If we get a "write scratchpad" command
  //Serial.print("Received E2B Data: ");
  for(int i=0; i < 8; i++){
    dataE2B[i] = e2b.scratchpad[i];
    //Serial.print(dataE2B[i],HEX); Serial.print(" ");
  }
  receivedAddress = (dataE2B[1]) | (dataE2B[2] << 8) | (dataE2B[3] << 16) | (dataE2B[4] << 24);
  receivedAddress &= ~0x1UL;   // Clear bit 0 to ensure even addresses
  //Serial.print("receivedAddress: "); Serial.print(receivedAddress,HEX); //Serial.println();
  if(receivedAddress < 0){
    //Serial.println("(ERROR: Negative value!)");
    //return 0;
  }else{
    Serial.println("");
  }
  //Reads/writes to the flash chip
  if(dataE2B[0] == 0xA){         //Write
    digitalWrite(MODE_LED,HIGH);
    delay(100);
    digitalWrite(MODE_LED,LOW);
    uint16_t dataToWriteToMemory = dataE2B[5]<<8 | dataE2B[6];
    ssd_write_flash(receivedAddress,dataToWriteToMemory);
    e2b.scratchpad[7] = 0xFE;

    ssd_setAddress(0x0000);
    flash_write_cycle(0xF0);
    delay(10);

  }else if(dataE2B[0] == 0xB){   //Read
    dataOutgoing = ssd_read_flash(receivedAddress);
    //dataOutgoing = 0xAD72;
    e2b.scratchpad[0] = lowByte(dataOutgoing);
    e2b.scratchpad[1] = highByte(dataOutgoing);
    e2b.scratchpad[5] = flash_mfg_id;
    e2b.scratchpad[6] = flash_dev_id;
    e2b.scratchpad[7] = 0xFE;
    e2b.setScratchpad(e2b.scratchpad);
  }

  runtime = millis() - runtime;
  totalServiceRate += runtime;
  serviceRateSamples++;
  averageServiceRate = totalServiceRate / serviceRateSamples;

  intervalNum++;
  if(intervalNum >= intervalThreshold){
    update_status();
    intervalNum = 0;
  }
  //receive_status_update();
}

//Sends the status of the SSD to the NAS controller
void sendStatus(){
  e2b.sendData_async(rom,8);
  e2b.sendData_async(ssd_status,4);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////SSD INTERFACE FUNCTIONS////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Prints the address onto the flash chip address lines
void ssd_setAddress(uint32_t addr){
  for (uint8_t i=0; i < 16; i++){
    mcp1.digitalWrite(i,bitRead(addr,i));
    if(i < 8){
      mcp2.digitalWrite(i,bitRead(addr,i+16));
    }
  }
  mcp2.digitalWrite(11, bitRead(addr, 24));    // GPB3 → A24, Handle A24 (bit 24) → GPB3 (i=11)
  mcp2.digitalWrite(12, bitRead(addr, 25));    // GPB2 → A25, A25 (bit 25) → GPB2 (i=12)
}

//Polls the flash memory at the specified address until the DQ7 bit matches the corresponding bit in expectedValue
//Returns true if DQ7 matches expected value within timeout, false otherwise.
void wait_for_flash_ready(uint32_t addr, uint16_t val) {
  uint16_t data;
  const uint8_t maxRetries = 100;  // prevent infinite loop
  uint8_t retries = 0;

  do {
    data = ssd_read_flash(addr);
    retries++;
    delayMicroseconds(10); // prevent busy-lockup
  } while (((data & 0x80) != (val & 0x80)) && retries < maxRetries);
}

void get_flash_ids(uint8_t &mfg, uint8_t &dev) {
  // Enter Autoselect Mode
  ssd_setAddress(0x555); flash_write_cycle(0xAA);
  ssd_setAddress(0x2AA); flash_write_cycle(0x55);
  ssd_setAddress(0x555); flash_write_cycle(0x90);
  delay(5);

  // Read Manufacturer and Device ID
  mfg = lowByte(ssd_read_flash(0x0000));
  dev = lowByte(ssd_read_flash(0x0001));
  delay(5);

  // Exit Autoselect Mode TWICE for safety
  ssd_setAddress(0x0000);
  flash_write_cycle(0xF0);
  delay(10);
  flash_write_cycle(0xF0);
  delay(10);
}





//Reads a two-byte value at an address
uint16_t ssd_read_flash(uint32_t addr) {
  ssd_setAddress(addr);     // 1. Set address lines

  for (uint8_t i=0; i < 16; i++) {
    pinMode(DQ[i], INPUT);  // 2. Set DQ[0..15] to high-impedance
  }

  // 3. Activate chip
  mcp2.digitalWrite(FLASH_WE, HIGH);
  //mcp2.digitalWrite(FLASH_CE, LOW);
  delayMicroseconds(1);
  mcp2.digitalWrite(FLASH_OE, LOW);
  delayMicroseconds(3);  // Let data become valid on bus (tOE = 70ns)

  // 4. Read data
  uint16_t val = 0;
  for (uint8_t i=0; i < 16; i++) {
    val |= (digitalRead(DQ[i]) << i);
  }

  // 5. Deactivate chip
  //mcp2.digitalWrite(FLASH_CE, HIGH);
  mcp2.digitalWrite(FLASH_OE, HIGH);

  return val;
}

//Writes a two-byte value to an address
bool ssd_write_flash(uint32_t addr, uint16_t val) {
  if (val > 0xFFFF) return false;

  // Issue Unlock + Program Command Sequence
  ssd_setAddress(0x555); flash_write_cycle(0xAA);
  ssd_setAddress(0x2AA); flash_write_cycle(0x55);
  ssd_setAddress(0x555); flash_write_cycle(0xA0);
  ssd_setAddress(addr); flash_write_cycle(val);
  
  delayMicroseconds(30);
  wait_for_flash_ready(addr, val);

  // Optional: Verify write
  return ssd_read_flash(addr) == val;
}

void flash_write_cycle(uint16_t val) {
  delayMicroseconds(1);

  // 1. Set DQ to OUTPUT and load data
  for (uint8_t i = 0; i < 16; i++) {
    pinMode(DQ[i], OUTPUT);
    digitalWrite(DQ[i], bitRead(val, i));
  }

  // 2. Prepare control lines
  //mcp2.digitalWrite(FLASH_CE, LOW);   // CE LOW to enable chip
  mcp2.digitalWrite(FLASH_OE, HIGH);  // Ensure OE is off
  mcp2.digitalWrite(FLASH_WE, HIGH);  // Pre-load WE high
  delayMicroseconds(1);

  mcp2.digitalWrite(FLASH_WE, LOW);   // Begin WE pulse
  delayMicroseconds(1);               // tWP = min 50ns
  mcp2.digitalWrite(FLASH_WE, HIGH);  // End WE pulse
  delayMicroseconds(1);               // tDH = min 10ns
  //mcp2.digitalWrite(FLASH_CE, HIGH);  // Disable chip

  // 3. Release DQ lines to high-impedance (read mode)
  for (uint8_t i = 0; i < 16; i++) {
    pinMode(DQ[i], INPUT);
    digitalWrite(DQ[i], LOW);  // Ensures no internal pullups
  }

  delayMicroseconds(2);
}





// Function to find the capacity based on pin states
/*int find_capacity(){
  float runtime = millis();
  int lastDataIndex = 0;
  int zeroCounter = 0;
  const int zeroThreshold = 100;
  
  // Loop from 0 to 2^26 (this is a large loop, be cautious!)
  for(unsigned long i=0; i < (1UL << 26); i++){
    if(ssd_read_flash(i)){
      lastDataIndex = i;
      zeroCounter = 0;
    }else{
      zeroCounter++;
      if(zeroCounter >= zeroThreshold){   //If there are <zeroThreshold> consecutive zero's, exit the function
        float c = lastDataIndex / pow(2,26);
        c *= 100;
        runtime = millis() - runtime;
        
        //Output the result
        if(lastDataIndex){
          Serial.print("Capacity found. Number of times data was detected: ");
          Serial.print("lastDataIndex: "); Serial.println(lastDataIndex);
          Serial.print("capacity: "); Serial.println((int)c);
          Serial.print("runtime: "); Serial.println(runtime);
        }else{
          Serial.println("No data found.");
        }
        return (int)c;
      }
    }
  }
}*/

//Erases a whole sector of memory from the flash chip
bool ssd_erase_sector(uint32_t address){
  uint32_t sectorAddr = address & 0xFFFF0000;

  Serial.print("Erasing sector at address: 0x");
  Serial.println(sectorAddr, HEX);

  ssd_setAddress(0x555); flash_write_cycle(0xAA);
  ssd_setAddress(0x2AA); flash_write_cycle(0x55);
  ssd_setAddress(0x555); flash_write_cycle(0x80);
  ssd_setAddress(0x555); flash_write_cycle(0xAA);
  ssd_setAddress(0x2AA); flash_write_cycle(0x55);
  ssd_setAddress(sectorAddr); flash_write_cycle(0x30);

  delayMicroseconds(30);
  wait_for_flash_ready(sectorAddr, 0xFFFF);

  ssd_setAddress(0x0000);
  flash_write_cycle(0xF0);
  delayMicroseconds(5);
  return true;
}
