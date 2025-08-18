////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch manages read/write requests from the NAS controller and interfaces with the onboard flash chip
-The EEPROM chip (AT28C256-15PU) has 256kbit of non-volatile memory
    -Total Size: 256kbit = 32KB
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
#define MODE_PIN 12
#define MODE_LED 11

#define FLASH_CE A2
#define FLASH_OE A3
#define FLASH_WE A1

unsigned char rom[8] = {FAMILYCODE, 0x45, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t DQ[8] = {10,9,8,3,4,5,6,7};
const uint8_t adr[16] = {0,1,2,3,4,5,6,7,0,15,14,13,12,11,10,9};
uint8_t ssd_status[4] = {0x0,0x0,0x0,0x0};    //Status bits, average service rate (integer, decimal), capacity

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

  Wire.setClock(100000);

  pinMode(MODE_PIN,INPUT_PULLUP);   //Nominally = 1, pushed = 0
  pinMode(MODE_LED, OUTPUT); digitalWrite(MODE_LED,LOW);
  pinMode(FLASH_CE, OUTPUT); digitalWrite(FLASH_CE,HIGH);
  pinMode(FLASH_OE, OUTPUT); digitalWrite(FLASH_OE,HIGH);
  pinMode(FLASH_WE, OUTPUT); digitalWrite(FLASH_WE,HIGH);

  //Initializes the IO expanders
  if(!mcp1.begin_I2C(0x20)){
    Serial.println("Error initializing MCP1.");
    while(1);
  }
  for(uint8_t i=0; i < 16; i++){
    mcp1.pinMode(i,OUTPUT);
  }

  /*Serial.print("Before Data at 0x15: "); Serial.println(ssd_read_flash(0x15),HEX);
  delay(100);
  ssd_write_flash(0x15,0xAA);
  delay(100);
  Serial.print("After Data at 0x15: "); Serial.println(ssd_read_flash(0x15),HEX);
  delay(100);
  ssd_write_flash(0x15,0x5C);
  delay(100);
  Serial.print("After Data at 0x15: "); Serial.println(ssd_read_flash(0x15),HEX);*/
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
/*void update_status(){
  Serial.println("---------------------------------------------------------------------------------------------------------------");
  Serial.println("Updating status");
  uint8_t statusBits = 0x0;
  bitWrite(statusBits,0,mcp1.begin_I2C(0x20));
  
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
}*/

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
  for(int i=0; i < 8; i++){
    dataE2B[i] = e2b.scratchpad[i];
    Serial.print(dataE2B[i],HEX); Serial.print(" ");
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
    uint16_t dataToWriteToMemory = dataE2B[5]<<8 | dataE2B[6];
    ssd_write_flash(receivedAddress,dataToWriteToMemory);
    e2b.scratchpad[7] = 0xFE;
    delay(10);
  }else if(dataE2B[0] == 0xB){   //Read
    dataOutgoing = ssd_read_flash(receivedAddress);
    //dataOutgoing = 0xAD72;
    e2b.scratchpad[0] = lowByte(dataOutgoing);
    e2b.scratchpad[1] = highByte(dataOutgoing);
    e2b.scratchpad[5] = 0;
    e2b.scratchpad[6] = 0;
    e2b.scratchpad[7] = 0xFE;
    e2b.setScratchpad(e2b.scratchpad);
  }

  runtime = millis() - runtime;
  totalServiceRate += runtime;
  serviceRateSamples++;
  averageServiceRate = totalServiceRate / serviceRateSamples;

  intervalNum++;
  if(intervalNum >= intervalThreshold){
    //update_status();
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
/*void ssd_setAddress(uint32_t addr){
  for (uint8_t i=0; i < 16; i++){
    if(i != 8)
      mcp1.digitalWrite(adr[i],bitRead(addr,i));
  }
}*/
void ssd_setAddress(uint16_t address) {
    // --- Low byte: A0–A7 straight onto GPA0–GPA7 ---
    mcp1.writeGPIOA(address & 0xFF);

    // --- High byte: A8–A14 mapped onto GPB7–GPB1 ---
    uint8_t highBits = (address >> 8) & 0x7F;  // only 7 bits
    uint8_t portB = 0;

    if (highBits & (1 << 0)) portB |= (1 << 7); // A8 → GPB7
    if (highBits & (1 << 1)) portB |= (1 << 6); // A9 → GPB6
    if (highBits & (1 << 2)) portB |= (1 << 5); // A10 → GPB5
    if (highBits & (1 << 3)) portB |= (1 << 4); // A11 → GPB4
    if (highBits & (1 << 4)) portB |= (1 << 3); // A12 → GPB3
    if (highBits & (1 << 5)) portB |= (1 << 2); // A13 → GPB2
    if (highBits & (1 << 6)) portB |= (1 << 1); // A14 → GPB1
    // GPB0 unused

    mcp1.writeGPIOB(portB);
}


//Reads a two-byte value at an address
uint8_t ssd_read_flash(uint16_t address) {
    ssd_setAddress(address);
    delayMicroseconds(100); // allow MCP23017 to settle

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], INPUT);
    }
    delayMicroseconds(10); // bus mode change settle

    digitalWrite(FLASH_CE, LOW);
    delayMicroseconds(10); // ensure CE stable before OE
    digitalWrite(FLASH_OE, LOW);
    delayMicroseconds(10); // allow EEPROM to present data
    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
      val |= (digitalRead(DQ[i]) << i);
    }

    digitalWrite(FLASH_OE, HIGH);
    delayMicroseconds(10);
    digitalWrite(FLASH_CE, HIGH);
    delayMicroseconds(10);

    return val;
}

void ssd_write_flash(uint16_t address, uint8_t data) {
    ssd_setAddress(address);
    delayMicroseconds(100); // allow MCP23017 to settle

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], OUTPUT);
      digitalWrite(DQ[i], bitRead(data, i));
    }
    delayMicroseconds(10); // data bus settle

    digitalWrite(FLASH_CE, LOW);
    digitalWrite(FLASH_OE, HIGH);
    digitalWrite(FLASH_WE, LOW);
    delayMicroseconds(10); // > tWP min
    digitalWrite(FLASH_WE, HIGH);
    delayMicroseconds(10); // tDH data hold
    digitalWrite(FLASH_CE, HIGH);

    // 4. Data Polling (section 6.4.1 in datasheet)
    uint8_t val;
    do {
        val = ssd_read_flash(address);
    } while ((val & 0x80) != (data & 0x80)); // poll D7 until it matches
    //delay(20); // tWC write cycle

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], INPUT);
  }
  delayMicroseconds(10); // let bus float
}

//Erases a whole sector of memory from the flash chip
void ssd_erase_sector(uint16_t startAddr, uint16_t length) {
  for (uint16_t addr = startAddr; addr < startAddr + length; addr++) {
    ssd_write_flash(addr, 0xFF);
  }
}

// Function to find the capacity based on pin states
/*int find_capacity(){
}*/