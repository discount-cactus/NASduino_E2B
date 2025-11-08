//Macchiato E2B SSD (Rev4) - EEPROM variant
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch manages read/write requests from the NAS controller and interfaces with the onboard EEPROM chip
-The EEPROM chip (AT28C256-15PU) has 256kbit of non-volatile memory
    -Total Size: 256kbit = 32KB
    -Start Address: 0x0000     End Address: 0x7FFF
-MODE_PIN pin for the MODE button are reserved for future use and do not currently have purpose
-LED_MODE pin for the LED are reserved for future use and do not currently have purpose
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <E2B.h>
#include <Adafruit_MCP23X17.h>

#define memory_type 0x50
#define memory_size 0x32
#define memory_kb_or_mb 0x3

#define E2B_pin 2
#define MODE_PIN 12
#define MODE_LED 11

#define EEPROM_CE A2
#define EEPROM_OE A3
#define EEPROM_WE A1

unsigned char rom[8] = {FAMILYCODE, 0x45, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t DQ[8] = {10,9,8,3,4,5,6,7};
const uint8_t adr[16] = {0,1,2,3,4,5,6,7,0,15,14,13,12,11,10,9};

uint16_t dataOutgoing;    //originally: int dataOutgoing
String dataInString = "";
uint8_t dataReceived[8];

int capacity = 12;                    //A percentage out of 100 (ex: capacity = 12 --> 12/100)

float totalServiceRate = 1.0;
float averageServiceRate = 1.0;
int serviceRateSamples = 0;
float totalLoopTime = 1.0;
float averageLoopTime = 1.0;

uint8_t intervalNum = 0;              //Tracks the number of received packets since the last status update
const uint8_t intervalThreshold = 5;  //Sets the number of packets received before updating the status bits

Adafruit_MCP23X17 mcp;
E2B e2b(E2B_pin);

void setup(){
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Macchiato E2B SSD - EEPROM");
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);

  Wire.setClock(100000);

  pinMode(MODE_PIN,INPUT_PULLUP);   //Nominally = 1, pushed = 0
  pinMode(MODE_LED, OUTPUT); digitalWrite(MODE_LED,LOW);
  pinMode(EEPROM_CE, OUTPUT); digitalWrite(EEPROM_CE,HIGH);
  pinMode(EEPROM_OE, OUTPUT); digitalWrite(EEPROM_OE,HIGH);
  pinMode(EEPROM_WE, OUTPUT); digitalWrite(EEPROM_WE,HIGH);

  //Initializes the IO expanders
  if(!mcp.begin_I2C(0x20)){
    Serial.println("Error initializing MCP23017.");
    while(1);
  }
  for(uint8_t i=0; i < 16; i++){
    mcp.pinMode(i,OUTPUT);
  }
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
/*void MODE_button_manager(){
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
}*/

//Updates the status of the SSD to be read by the NAS controller
uint8_t update_status(){
  Serial.println("---------------------------------------------------------------------------------------------------------------");
  Serial.println("Updating status");
  uint8_t statBits = 0x0;
  bitWrite(statBits,0,mcp.begin_I2C(0x20));
  
  //Checks memory
  const uint8_t checkAddr = 20;
  uint16_t checkDat = ssd_read_eeprom(checkAddr);
  ssd_write_eeprom(checkAddr,0xC2);
  if(ssd_read_eeprom(checkAddr) == 0xC2){
    bitWrite(statBits,1,1);
  }else{
    bitWrite(statBits,1,0);
  }
  ssd_write_eeprom(checkAddr,checkDat);

  Serial.print("Status bits: "); Serial.println(statBits);
  Serial.println("---------------------------------------------------------------------------------------------------------------");
  return statBits;
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
  for(int i=0; i < 8; i++){
    dataE2B[i] = e2b.scratchpad[i];
    //Serial.print(dataE2B[i],HEX); Serial.print(" ");
  }
  receivedAddress = (dataE2B[1]) | (dataE2B[2] << 8) | (dataE2B[3] << 16) | (dataE2B[4] << 24);
  receivedAddress &= ~0x1UL;   // Clear bit 0 to ensure even addresses
  //Serial.print("receivedAddress: "); Serial.print(receivedAddress,HEX); //Serial.println();
  if(receivedAddress < 0){
    Serial.println("(ERROR: Negative value!)");
    return 0;
  }/*else{
    Serial.println("");
  }*/

  //Reads/writes to the EEPROM chip
  if(dataE2B[0] == 0xA){         //Write
    uint8_t dataToWriteToMemory = dataE2B[5]; //dataE2B[5]<<8 | dataE2B[6];
    ssd_write_eeprom(receivedAddress,dataToWriteToMemory);
    delay(10);
  }else if(dataE2B[0] == 0xB){   //Read
    dataOutgoing = ssd_read_eeprom(receivedAddress);
    e2b.scratchpad[0] = lowByte(dataOutgoing);
    e2b.scratchpad[1] = 0;
    e2b.scratchpad[2] = memory_type;
    e2b.scratchpad[3] = memory_size;
    e2b.scratchpad[5] = memory_kb_or_mb;
    e2b.scratchpad[6] = update_status();
    e2b.scratchpad[7] = capacity;
    e2b.scratchpad[8] = 0;
    e2b.setScratchpad(e2b.scratchpad);
  }

  runtime = millis() - runtime;
  totalServiceRate += runtime;
  serviceRateSamples++;
  averageServiceRate = totalServiceRate / serviceRateSamples;
  averageLoopTime = totalLoopTime / serviceRateSamples;

  intervalNum++;
  if(intervalNum >= intervalThreshold){
    capacity = find_capacity();
    intervalNum = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////SSD INTERFACE FUNCTIONS////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Prints the address onto the EEPROM chip address lines
void ssd_setAddress(uint16_t address) {
    mcp.writeGPIOA(address & 0xFF);

    uint8_t highBits = (address >> 8) & 0x7F;  // only 7 bits
    uint8_t portB = 0;

    if (highBits & (1 << 0)) portB |= (1 << 7); // A8 → GPB7
    if (highBits & (1 << 1)) portB |= (1 << 6); // A9 → GPB6
    if (highBits & (1 << 2)) portB |= (1 << 5); // A10 → GPB5
    if (highBits & (1 << 3)) portB |= (1 << 4); // A11 → GPB4
    if (highBits & (1 << 4)) portB |= (1 << 3); // A12 → GPB3
    if (highBits & (1 << 5)) portB |= (1 << 2); // A13 → GPB2
    if (highBits & (1 << 6)) portB |= (1 << 1); // A14 → GPB1
    // GPB0 is not connected

    mcp.writeGPIOB(portB);
}


//Reads a byte of data from the EEPROM chip
uint8_t ssd_read_eeprom(uint16_t address) {
    ssd_setAddress(address);
    delayMicroseconds(100); // allow MCP23017 to settle

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], INPUT);
    }
    delayMicroseconds(10); // bus mode change settle

    digitalWrite(EEPROM_CE, LOW);
    delayMicroseconds(10); // ensure CE stable before OE
    digitalWrite(EEPROM_OE, LOW);
    delayMicroseconds(10); // allow EEPROM to present data
    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
      val |= (digitalRead(DQ[i]) << i);
    }

    digitalWrite(EEPROM_OE, HIGH);
    delayMicroseconds(10);
    digitalWrite(EEPROM_CE, HIGH);
    delayMicroseconds(10);

    return val;
}

//Writes a byte of data to the EEPROM chip
void ssd_write_eeprom(uint16_t address, uint8_t data) {
    ssd_setAddress(address);
    delayMicroseconds(100);

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], OUTPUT);
      digitalWrite(DQ[i], bitRead(data, i));
    }
    delayMicroseconds(10); // data bus settle

    digitalWrite(EEPROM_CE, LOW);
    digitalWrite(EEPROM_OE, HIGH);
    digitalWrite(EEPROM_WE, LOW);
    delayMicroseconds(10); // > tWP min
    digitalWrite(EEPROM_WE, HIGH);
    delayMicroseconds(10); // tDH data hold
    digitalWrite(EEPROM_CE, HIGH);

    // 4. Data Polling (section 6.4.1 in datasheet)
    uint8_t val;
    do {
        val = ssd_read_eeprom(address);
    } while ((val & 0x80) != (data & 0x80)); // poll D7 until it matches

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], INPUT);
  }
  delayMicroseconds(10); // let the bus float
}

//Function to find the capacity based on pin states
int find_capacity(){
  int num = 0;
  for(int i=0; i < 0x7FFF; i++){
    if(ssd_read_eeprom(i))
      num++;
  }
  num = num / 0x7FFF;
  return round(num);
}