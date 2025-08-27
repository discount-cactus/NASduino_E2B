//Macchiato E2B SSD (Rev4.1) - SRAM variant
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch manages read/write requests from the NAS controller and interfaces with the onboard flash chip
-The SRAM chip (AS6C8016-55ZIN) has 8Mbit of volatile memory
    -Total Size: 8Mbit = 1MB
    -Sector 0:         Start Address: 0x00000     End Address: 0x7FFFF
-MODE_PIN pin for the MODE button are reserved for future use and do not currently have purpose
-LED_MODE pin for the LED are reserved for future use and do not currently have purpose
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <E2B.h>
#include <Adafruit_MCP23X17.h>

#define memory_type 0x51
#define memory_size 0x8
#define memory_kb_or_mb 0x6

#define E2B_pin 2
#define MODE_PIN 12
#define MODE_LED 11

#define SRAM_CE 4
#define SRAM_OE 7
#define SRAM_WE 7
#define SRAM_LB 5
#define SRAM_UB 6

unsigned char rom[8] = {FAMILYCODE, 0x45, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t DQ[16] = {10,9,8,7,6,5,4,3,0,1,MODE_LED,A3,A2,A1,A0,13};
//const uint8_t adr[16] = {0,1,2,3,4,5,6,7,0,15,14,13,12,11,10,9};

uint16_t dataOutgoing;    //originally: int dataOutgoing
String dataInString = "";
uint8_t dataReceived[8];

uint8_t statusBits = 0;
int capacity = 12;                    //A percentage out of 100 (ex: capacity = 12 --> 12/100)

float totalServiceRate = 1.0;
float averageServiceRate = 1.0;
int serviceRateSamples = 0;

uint8_t intervalNum = 0;              //Tracks the number of received packets since the last status update
const uint8_t intervalThreshold = 5;  //Sets the number of packets received before updating the status bits

Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;
E2B e2b(E2B_pin);

void setup(){
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Macchiato E2B SSD - SRAM");
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);

  Wire.setClock(100000);

  pinMode(MODE_PIN,INPUT_PULLUP);   //Nominally = 1, pushed = 0
  pinMode(MODE_LED, OUTPUT); digitalWrite(MODE_LED,LOW);

  //Initializes the IO expanders
  if(!mcp1.begin_I2C(0x20)){
    Serial.println("Error initializing MCP1.");
    while(1);
  }
  if(!mcp2.begin_I2C(0x21)){
    Serial.println("Error initializing MCP2.");
    while(1);
  }

  for(uint8_t i=0; i < 16; i++){
    mcp1.pinMode(i,OUTPUT);
    mcp2.pinMode(i, OUTPUT);
  }
  //mcp2.pinMode(FLASH_RESET,OUTPUT); mcp2.digitalWrite(FLASH_RESET,HIGH);    //Write protect input to falsh chip
  mcp2.pinMode(SRAM_CE, OUTPUT); mcp2.digitalWrite(SRAM_CE,HIGH);
  mcp2.pinMode(SRAM_OE, OUTPUT); mcp2.digitalWrite(SRAM_OE,HIGH);
  mcp1.pinMode(SRAM_WE, OUTPUT); mcp1.digitalWrite(SRAM_WE,HIGH);
  mcp2.pinMode(SRAM_LB, OUTPUT); mcp2.digitalWrite(SRAM_LB,HIGH);
  mcp2.pinMode(SRAM_UB, OUTPUT); mcp2.digitalWrite(SRAM_UB,HIGH);

  //uint8_t dataTest = 0x4C;
  uint16_t dataTest = 0x3DA8;
  Serial.print("Data written to SRAM chip: 0x"); Serial.println(dataTest,HEX);
  //ssd_write_sram_byte(15,dataTest,SRAM_LB);
  //ssd_write_sram_byte(15,dataTest,SRAM_UB);
  ssd_write_sram_word(15,dataTest);
  delay(500);
  //Serial.print("Data read from SRAM chip: 0x"); Serial.println(ssd_read_sram_byte(15,SRAM_LB),HEX);
  //Serial.print("Data read from SRAM chip: 0x"); Serial.println(ssd_read_sram_byte(15,SRAM_UB),HEX);
  Serial.print("Data read from SRAM chip: 0x"); Serial.println(ssd_read_sram_word(15),HEX);
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
  bitWrite(statBits,0,mcp1.begin_I2C(0x20));
  
  //Checks memory
  const uint8_t checkAddr = 20;
  uint16_t checkDat = ssd_read_sram_byte(checkAddr,SRAM_LB);
  ssd_write_sram_byte(checkAddr,0xC2,SRAM_LB);
  if(ssd_read_sram_byte(checkAddr,SRAM_LB) == 0xC2){
    bitWrite(statBits,1,1);
  }else{
    bitWrite(statBits,1,0);
  }
  ssd_write_sram_byte(checkAddr,checkDat,SRAM_LB);

  /*int IntegerPart = (int)(averageServiceRate);
  int DecimalPart = 100 * (averageServiceRate - IntegerPart);
  ssd_status[1] = IntegerPart;
  ssd_status[2] = DecimalPart;*/

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

  //Reads/writes to the SRAM chip
  if(dataE2B[0] == 0xA){         //Write
    if((dataE2B[5] == 0) || (dataE2B[6] == 0)){
      if(dataE2B[5] == 0){
        uint8_t dataToWriteToMemory = dataE2B[6];
        ssd_write_sram_byte(receivedAddress,dataToWriteToMemory,SRAM_UB);
      }else{
        uint8_t dataToWriteToMemory = dataE2B[5];
        ssd_write_sram_byte(receivedAddress,dataToWriteToMemory,SRAM_LB);
      }
    }else{
      uint16_t dataToWriteToMemory = dataE2B[5]<<8 | dataE2B[6];
      ssd_write_sram_word(receivedAddress,dataToWriteToMemory);
    }
    delay(10);
  }else if(dataE2B[0] == 0xB){   //Read
  if((dataE2B[5] == 0) || (dataE2B[6] == 0)){
      if(dataE2B[5] == 0){
        dataOutgoing = ssd_read_sram_byte(receivedAddress,SRAM_UB);
        e2b.scratchpad[0] = 0;
        e2b.scratchpad[1] = highByte(dataOutgoing);
      }else{
        dataOutgoing = ssd_read_sram_byte(receivedAddress,SRAM_LB);
        e2b.scratchpad[0] = lowByte(dataOutgoing);
        e2b.scratchpad[1] = 0;
      }
    }else{
      dataOutgoing = ssd_read_sram_word(receivedAddress);
      e2b.scratchpad[0] = lowByte(dataOutgoing);
      e2b.scratchpad[1] = highByte(dataOutgoing);
    }
    e2b.scratchpad[2] = memory_type;
    e2b.scratchpad[3] = memory_size;
    e2b.scratchpad[5] = memory_kb_or_mb;
    e2b.scratchpad[6] = statusBits;
    e2b.scratchpad[7] = capacity;
    e2b.scratchpad[8] = 0;
    e2b.setScratchpad(e2b.scratchpad);
  }

  runtime = millis() - runtime;
  totalServiceRate += runtime;
  serviceRateSamples++;
  averageServiceRate = totalServiceRate / serviceRateSamples;

  intervalNum++;
  if(intervalNum >= intervalThreshold){
    //update_status();
    //capacity = find_capacity();
    intervalNum = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////SSD INTERFACE FUNCTIONS////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Prints the address onto the SRAM chip address lines
void ssd_setAddress(uint16_t address){
  uint8_t mcp1_address[19] = {3,2,1,0,0,1,2,3,8,9,10,11,12,13,14,15,4,5,6};
  for (uint8_t i = 0; i < 19; i++) {
    if((i < 4) && (i > 7)){
      mcp1.digitalWrite(mcp1_address[i],bitRead(address,i));
    }else{
      mcp2.digitalWrite(mcp1_address[i],bitRead(address,i));
    }
  }

  /*mcp1.digitalWrite(3,bitRead(address,0));
  mcp1.digitalWrite(2,bitRead(address,1));
  mcp1.digitalWrite(1,bitRead(address,2));
  mcp1.digitalWrite(0,bitRead(address,3));

  mcp2.digitalWrite(0,bitRead(address,4));
  mcp2.digitalWrite(1,bitRead(address,5));
  mcp2.digitalWrite(2,bitRead(address,6));
  mcp2.digitalWrite(3,bitRead(address,7));

  mcp1.digitalWrite(8,bitRead(address,8));
  mcp1.digitalWrite(9,bitRead(address,9));
  mcp1.digitalWrite(10,bitRead(address,10));
  mcp1.digitalWrite(11,bitRead(address,11));
  mcp1.digitalWrite(12,bitRead(address,12));
  mcp1.digitalWrite(13,bitRead(address,13));
  mcp1.digitalWrite(14,bitRead(address,14));
  mcp1.digitalWrite(15,bitRead(address,15));
  mcp1.digitalWrite(4,bitRead(address,16));
  mcp1.digitalWrite(5,bitRead(address,17));
  mcp1.digitalWrite(6,bitRead(address,18));*/
}


//Reads a byte of data from the SRAM chip
uint8_t ssd_read_sram_byte(uint32_t address, uint8_t byteSelect){
    ssd_setAddress(address);
    delayMicroseconds(100); // allow MCP23017 to settle

    for (uint8_t i = 0; i < 16; i++) {
      pinMode(DQ[i], INPUT);
    }
    delayMicroseconds(10); // bus mode change settle

    mcp2.digitalWrite(SRAM_CE, LOW);
    delayMicroseconds(10); // ensure CE stable before OE
    mcp2.digitalWrite(SRAM_OE, LOW);
    delayMicroseconds(10);
    mcp1.digitalWrite(SRAM_WE, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(byteSelect, LOW);
    delayMicroseconds(10); // allow SRAM to present data

    uint8_t val = 0;
    for (uint8_t i = 0; i < 16; i++) {
      val |= (digitalRead(DQ[i]) << i);
    }

    mcp2.digitalWrite(SRAM_OE, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(SRAM_CE, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(byteSelect, HIGH);
    delayMicroseconds(10);

    return val;
}

//Reads a byte of data from the SRAM chip
uint16_t ssd_read_sram_word(uint32_t address){
    ssd_setAddress(address);
    delayMicroseconds(100); // allow MCP23017 to settle

    for (uint8_t i=0; i < 16; i++) {
      pinMode(DQ[i], INPUT);
    }
    delayMicroseconds(10); // bus mode change settle

    mcp2.digitalWrite(SRAM_CE, LOW);
    delayMicroseconds(10); // ensure CE stable before OE
    mcp2.digitalWrite(SRAM_OE, LOW);
    delayMicroseconds(10);
    mcp1.digitalWrite(SRAM_WE, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(SRAM_LB, LOW);
    delayMicroseconds(10);
    mcp2.digitalWrite(SRAM_UB, LOW);
    delayMicroseconds(10); // allow SRAM to present data

    uint16_t val = 0;
    for (uint8_t i=0; i < 16; i++) {
      val |= (digitalRead(DQ[i]) << i);
    }

    mcp2.digitalWrite(SRAM_OE, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(SRAM_CE, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(SRAM_LB, HIGH);
    delayMicroseconds(10);
    mcp2.digitalWrite(SRAM_UB, HIGH);
    delayMicroseconds(10);

    return val;
}

//Writes a byte of data to the SRAM chip
void ssd_write_sram_byte(uint32_t address, uint8_t data, uint8_t byteSelect){
    ssd_setAddress(address);
    delayMicroseconds(100);

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], OUTPUT);
      digitalWrite(DQ[i], bitRead(data, i));
    }
    delayMicroseconds(10); // data bus settle

    mcp2.digitalWrite(SRAM_CE, LOW);
    mcp2.digitalWrite(SRAM_OE, HIGH);
    mcp2.digitalWrite(byteSelect, LOW);
    mcp1.digitalWrite(SRAM_WE, LOW);

    delayMicroseconds(10); // > tWP min

    mcp1.digitalWrite(SRAM_WE, HIGH);
    mcp2.digitalWrite(SRAM_CE, HIGH);
    mcp2.digitalWrite(byteSelect, HIGH);

    // 4. Data Polling (section 6.4.1 in datasheet)
    /*uint8_t val;
    do {
        val = ssd_read_sram_byte(address);
    } while ((val & 0x80) != (data & 0x80)); // poll D7 until it matches*/

    for (uint8_t i = 0; i < 8; i++) {
      pinMode(DQ[i], INPUT);
  }
  delayMicroseconds(10); // let the bus float
}

//Writes a word of data to the SRAM chip
void ssd_write_sram_word(uint32_t address, uint16_t data){
    ssd_setAddress(address);
    delayMicroseconds(100);

    for (uint8_t i = 0; i < 16; i++) {
      pinMode(DQ[i], OUTPUT);
      digitalWrite(DQ[i], bitRead(data, i));
    }
    delayMicroseconds(10); // data bus settle

    mcp2.digitalWrite(SRAM_CE, LOW);
    mcp2.digitalWrite(SRAM_OE, HIGH);
    mcp2.digitalWrite(SRAM_LB, LOW);
    mcp2.digitalWrite(SRAM_UB, LOW);
    mcp1.digitalWrite(SRAM_WE, LOW);

    delayMicroseconds(10); // > tWP min

    mcp1.digitalWrite(SRAM_WE, HIGH);
    //delayMicroseconds(10); // tDH data hold
    mcp2.digitalWrite(SRAM_CE, HIGH);
    mcp2.digitalWrite(SRAM_LB, HIGH);
    mcp2.digitalWrite(SRAM_UB, HIGH);

    // 4. Data Polling (section 6.4.1 in datasheet)
    /*uint8_t val;
    do {
        val = ssd_read_sram_byte(address);
    } while ((val & 0x80) != (data & 0x80)); // poll D7 until it matches*/

    for (uint8_t i = 0; i < 16; i++) {
      pinMode(DQ[i], INPUT);
  }
  delayMicroseconds(10); // let the bus float
}

//Function to find the capacity based on pin states
int find_capacity(){
  int num = 0;
  for(int i=0; i < 0x7FFF; i++){
    if(ssd_read_sram_word(i))
      num++;
  }
  num = num / 0x7FFF;
  return round(num);
}