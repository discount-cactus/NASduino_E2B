//MAcchiato E2B SSD (Rev2) Example
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch manages data requests from the Macchiato Transceiver and upstream hardware and writes/reads from the onboard flash chip
-The flash chip (S29GL01GS10TFI020) has 1GB of flash memory
-MODE_PIN for the MODE button and LED_MODE pin for the LED are reserved for future use
  and do not currently have functionality

Features:
-E2B Transciever via ATtiny85
-MODE button for switching between operating modes
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_MCP23X17.h>
#include <SoftwareSerial.h>

#define MODE_PIN 14

//#define E2B_pin 4
#define UART_RX 22 //18
#define UART_TX 23 //17
#define E2B_ALT_DIR 1

//#define FLASH_WP 13
#define FLASH_CE 9
#define FLASH_OE 10
#define FLASH_WE 15
#define FLASH_RESET 16

uint8_t DQ[16] = {10,12,21,35,37,40,42,43,11,13,47,36,39,41,44,2};

uint16_t dataOutgoing;    //originally: int dataOutgoing
String dataInString = "";
uint8_t dataReceived[8];

Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;
//HardwareSerial mySerial(1);
SoftwareSerial mySerial(UART_RX,UART_TX); // RX, TX

void setup() {
  Serial.begin(115200);
  while(!Serial){}
  mySerial.begin(9600);
  //mySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  while(!mySerial){}
  Serial.println("Macchiato E2B SSD Test!");
  pinMode(E2B_ALT_DIR,OUTPUT);

  pinMode(MODE_PIN,INPUT_PULLUP);   //Nominally = 1, pushed = 0

  //Initializes all pin modes
  //pinMode(FLASH_WP,OUTPUT); digitalWrite(FLASH_WP,HIGH);    //Write protect input to falsh chip 0
  pinMode(FLASH_RESET,OUTPUT); digitalWrite(FLASH_RESET,HIGH);    //Write protect input to falsh chip 0
  pinMode(FLASH_CE, OUTPUT);
  pinMode(FLASH_OE, OUTPUT);
  pinMode(FLASH_WE, OUTPUT);

  //Initializes the IO expanders
  /*if(!mcp1.begin_I2C(0x20)){
    Serial.println("Error initializing MCP1.");
    while(1);
  }
  if(!mcp2.begin_I2C(0x21)){
    Serial.println("Error initializing MCP2.");
    while(1);
  }

  for (uint8_t i=0; i < 16; i++){
    mcp1.pinMode(i, OUTPUT);
    mcp2.pinMode(i, OUTPUT);
  }*/

}

void loop(){
  MODE_button_manager();
  receive_packet();
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
    }else{
      //Do other stuff
    }
  }
}

//Receives UART packet
bool receive_packet(){
  digitalWrite(E2B_ALT_DIR,HIGH);

  if(mySerial.available() == 8){
    delay(50);            //Tune this if data gets unstable (usually it needs an increase)
    int receivedInt = 0;
    //Serial.print("Received Data: ");
    for (int i=0; i < 8; i++) {
      dataReceived[i] = mySerial.read();
      if(i == 0){
        if((dataReceived[0] == 0xA) || (dataReceived[0] == 0xB)){
        }else{
          return 0;
        }
      }
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

    if(dataReceived[0] == 0xA){         //Write
      uint16_t dataToWriteToMemory = dataReceived[5]<<8 | dataReceived[6];
      //ssd_write_flash(receivedInt,dataToWriteToMemory);
    }else if(dataReceived[0] == 0xB){   //Read
      //dataOutgoing = ssd_read_flash(receivedInt);
      transmit_packet();
    }else{
      return 0;
    }
  }
}

//Transmits UART packet
void transmit_packet(){
  dataOutgoing = 0xADD2;
  digitalWrite(E2B_ALT_DIR,LOW);
  mySerial.write(lowByte(dataOutgoing));
  mySerial.write(highByte(dataOutgoing));
  digitalWrite(E2B_ALT_DIR,HIGH);
}

//Prints the address onto the flash chip address lines
void ssd_setAddress(int addr){
  for (uint8_t i=0; i < 16; i++){
    mcp1.pinMode(i,OUTPUT);
    mcp1.digitalWrite(i,bitRead(addr,i));
    if((i < 8) || (i == 11) || (i == 12)){
      mcp2.pinMode(i,OUTPUT);
      mcp2.digitalWrite(i,bitRead(addr,i+16));
    }
  }
}

//Reads a two-byte value at an address
uint16_t ssd_read_flash(int addr){
  ssd_setAddress(addr);
  Serial.print("Reading Flash Address: "); Serial.println(addr);
  
  //Configures the flash chip for reading
  mcp2.digitalWrite(FLASH_CE,LOW);
  mcp2.digitalWrite(FLASH_OE,LOW);
  mcp2.digitalWrite(FLASH_WE,HIGH);
  delay(1);

  uint16_t val = 0;
  for (uint8_t i=0; i < 16; i++){
    pinMode(DQ[i],INPUT);
    val += (digitalRead(DQ[i]) * pow(2,i));
  }

  Serial.print("Data Read:  ");
  /*for (int i=15; i > -1; i--){
    Serial.print(digitalRead(DQ[i]));
  }
  Serial.print(" ("); Serial.print(val); Serial.println(")");*/
  Serial.println(val,HEX);

  mcp2.digitalWrite(FLASH_CE,HIGH);
  mcp2.digitalWrite(FLASH_OE,HIGH);

  return val;
}

//Writes a two-byte value to an address
bool ssd_write_flash(int addr, int val){
  if(val <= 65536){   //Returns 0 if the value is bigger than 2^16
    Serial.print("Writing: "); Serial.print(val,HEX); Serial.print(" to flash address: "); Serial.println(addr);
    ssd_setAddress(addr);
    
    //Configures the flash chip for writing
    mcp2.digitalWrite(FLASH_CE,LOW);
    mcp2.digitalWrite(FLASH_OE,HIGH);
    mcp2.digitalWrite(FLASH_WE,LOW);

    for (uint8_t i=0; i < 16; i++){
      pinMode(DQ[i],OUTPUT);
      digitalWrite(DQ[i],bitRead(val,i));
    }

    /*Serial.print("Data Write: ");
    for (int i=15; i > -1; i--){
      Serial.print(digitalRead(DQ[i]));
    }
    Serial.print(" ("); Serial.print(val); Serial.println(")");*/

    mcp2.digitalWrite(FLASH_CE,HIGH);
    mcp2.digitalWrite(FLASH_WE,HIGH);

    return 1;
  }else{
    return 0;
  }
}