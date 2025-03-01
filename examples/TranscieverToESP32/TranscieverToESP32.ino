//ATtiny UART-to-E2B bridge example
//Simple conversion from UART-to-E2B bridge for use as a seamless transceiver between devices

/*NOTES:
-MADE FOR USE WITH THE ATTINY85 MICROCONTROLLER
-Receives a UART packet and transmits it on the E2B pin
-Receives a E2B packet and transmits it on the UART pins

-ATtiny85 does not feature a dedicated UART port, so SoftwareSerial is used

-TaskScheduler is used to complete sending and receiving of different protocols in parallel
-Link to TaskScheduler library: https://github.com/arkhipenko/TaskScheduler

Hookup :
ATtiny85 RX pin: -> Arduino RX     (pin 0 on Arduino Uno)
ATtiny85 TX pin: -> Arduino TX     (pin 1 on Arduino Uno)
*/
#include <E2B.h>
#include <SoftwareSerial.h>
 
#define E2B_pin 2
#define rxPin 3
#define txPin 4
#define dirPin 1

unsigned char rom[8] = {FAMILYCODE_TRANSCEIVER, FAMILYCODE, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte localscratchpad[9]; // buffer for data
//byte addr[8]; // 64-bit device address

E2B e2b(E2B_pin);
SoftwareSerial mySerial(rxPin, txPin);

int adr = 15;   //Temporary, for testing

void setup(){
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("E2B NAS Transiever Test.");
  e2b.setBusType(TRANSCEIVER);
  mySerial.begin(9600);
  pinMode(dirPin,INPUT);
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(){
  bool dir = digitalRead(dirPin);
  if(!dir){
    uart_to_e2b();
  }else{
    e2b_to_uart();
  }
  delay(1000);
  //uint8_t dataE2B = e2b.scratchpad[4];
  //mySerial.write(dataE2B);
}

void uart_to_e2b(){
  //Get UART, send E2B
  //Receive packet
  if (mySerial.available()){
    //Serial.write(mySerial.read());
    byte low,high;
    low = mySerial.read();
    high = mySerial.read();
    uint16_t dataE2B = high<<8 | low;
    Serial.print(dataE2B,HEX);
    Serial.println();
  }
  /*if (mySerial.available()){
    uint8_t dataUART = mySerial.read();
    //Serial.print(dataUART,HEX);
    
    boolean present;
    present = e2b.reset();                // device present var
    e2b.skip(); 
    if(present){
      e2b.write(dataUART);
    }
  }*/
}

void e2b_to_uart(){
  //Get E2B, send UART
  //uint8_t dataE2B[8] = {0xA, 0x5, 0x0, 0x0, 0x0, 0xF1, 0xA3, 0x0};
  uint8_t dataE2B[8];
  e2b.waitForRequest(false);
  Serial.print("Received E2B Data: ");
    for(int i=0; i < 8; i++){
      dataE2B[i] = e2b.scratchpad[i];
      Serial.print(dataE2B[i],HEX); Serial.print(" ");
      e2b.scratchpad[i] = 0x00;
    }
    Serial.println();

  //Transmit packet
  for (int i=0; i < 8; i++){
    mySerial.write(dataE2B[i]);
  }
  delay(1000);
  /*byte bytes[4]; 
  for (int i=0; i < 4; i++) {
    bytes[i] = (adr >> (i * 8)) & 0xFF; // Extract each byte
    mySerial.write(bytes[i]);
  }*/
  /*e2b.waitForRequest(false);
  uint8_t dataE2B = e2b.scratchpad[4];
  mySerial.write(dataE2B);
  //Serial.println(dataE2B,HEX);*/
}