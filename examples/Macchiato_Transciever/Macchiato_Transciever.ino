//ATtiny UART-to-E2B bridge example
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch is a simple conversion from UART-to-E2B bridge for use as a seamless transceiver between devices

-MADE FOR USE WITH THE ATTINY85 MICROCONTROLLER
-Receives a UART packet and transmits it on the E2B pin
-Receives a E2B packet and transmits it on the UART pins
-Direction of transmission is governed by the dirPin which is an I/O pin toggled by the device connected via softserial
-ATtiny85 does not feature a dedicated UART port, so SoftwareSerial is used

Hookup :
ATtiny85 TX pin: -> Arduino RX     (pin 0 on Arduino Uno)
ATtiny85 RX pin: -> Arduino TX     (pin 1 on Arduino Uno)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <E2B.h>
#include <SoftwareSerial.h>
 
#define E2B_pin 2
#define rxPin 3
#define txPin 4
#define dirPin 8 //1

unsigned char rom[8] = {FAMILYCODE_TRANSCEIVER, FAMILYCODE, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte localscratchpad[9]; // buffer for data
//byte addr[8]; // 64-bit device address

E2B e2b(E2B_pin);
SoftwareSerial mySerial(rxPin, txPin);

void setup(){
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  mySerial.begin(9600);
  while(!mySerial){}
  Serial.println("Macchiato E2B Transiever");
  e2b.setBusType(TRANSCEIVER);
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
  if(digitalRead(dirPin)){
    e2b_to_uart();
  }
}

//Converts E2B data to UART data to be sent to the SSD
void e2b_to_uart(){
  //Get E2B, send UART
  uint8_t dataE2B[8];
  e2b.waitForRequest(false);
    if(e2b.getScratchpad(4) != 0xBE){
    //Serial.print("Received E2B Data: ");
    for(int i=0; i < 8; i++){
      dataE2B[i] = e2b.scratchpad[i];
      Serial.print(dataE2B[i],HEX); Serial.print(" ");
      e2b.scratchpad[i] = 0x00;
      mySerial.write(dataE2B[i]);
    }
    Serial.println();

    uart_to_e2b();
  }
}

//Converts UART data to E2B data to be sent back to the NAS controller
void uart_to_e2b(){
  //Get UART, send E2B
  //Receive packet
  if(mySerial.available()){
    delay(100);            //Tune this if data gets unstable (usually it needs an increase)
    byte low,high;
    low = mySerial.read();
    high = mySerial.read();
    uint16_t dataE2B = high<<8 | low;
    //Serial.print(dataE2B,HEX);
    Serial.println();
    e2b.scratchpad[0] = low;
    e2b.scratchpad[1] = high;
  }
}