//E2B DAS Controller
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch directly interfaces to subsequent SSD's via E2B
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <E2B.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>
#include "EEPROM.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

#define E2B_pin 4
#define buttonPin 5
#define MaxConnectedDeviceNum 12

#define cmd_delay 50

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x4B, 0x46, 0x7F, 0xFF, 0x00, 0x10, 0x00};
uint8_t packetData[8];

uint8_t _connectedDevices[MaxConnectedDeviceNum][8];
uint8_t _previousDevices[MaxConnectedDeviceNum][8];

E2B e2b(E2B_pin);  // on pin 4 (a 4.7K resistor is necessary)

void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("E2B DAS Controller.");
  pinMode(buttonPin,INPUT_PULLUP);

  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialize EEPROM");
    Serial.println("Restarting...");
    delay(10);
    ESP.restart();
  }

  if(get_connected_devices_from_EEPROM()){
    initialize_connected_devices();
  }
  print_connected_devices();
}

void loop(){
  button_manager();

  query_das_write(0,17,0x3B);
  delay(1000);
  query_das_read(0,17);
  delay(1000);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////BUTTON FUNCTIONS////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Writes a byte of data to the DAS
void query_das_write(uint8_t port, int address, uint8_t dat){
  uint8_t i;
  //Prepares the data to send
  packetData[0] = 0xA;                        // Command: 0xA = write, 0xB = read
  for (i=0; i < 4; i++) {                     //Encodes the address into 4 bytes
    packetData[i + 1] = (address >> (i * 8)) & 0xFF;  //packetData[i] = (address >> (i * 8)) & 0xFF;  // Extract each byte
  }
  packetData[5] = lowByte(dat);             // Data LSB
  packetData[6] = highByte(dat);            // Data MSB
  packetData[7] = 0x00;                      // Reserved or checksum placeholder
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  //Makes sure the port has a device connected to it
  int checksum = 0;
  for (i=0; i < 8; i++) {
    checksum += _connectedDevices[port][i];
  }
  if(checksum == 0){
    Serial.println("No device logged at that the queried port.");
    return;
  }

  //Transmits data to selected SSD
  e2b.reset();
  e2b.select(_connectedDevices[port]);
  e2b.write_scratchpad();
  for(i=0; i < 8; i++){
    e2b.write(packetData[i]);
    Serial.print(packetData[i]); Serial.print(" ");
  }
  Serial.println(" ");

  delay(cmd_delay);
}

//Reads a byte of data from the DAS
uint8_t query_das_read(uint8_t port, int address){
  uint8_t i;
  //Prepares the data to send
  packetData[0] = 0xB;                        // Command: 0xA = write, 0xB = read
  for (i=0; i < 4; i++) {                     //Encodes the address into 4 bytes
    packetData[i + 1] = (address >> (i * 8)) & 0xFF;  //packetData[i] = (address >> (i * 8)) & 0xFF;  // Extract each byte
  }
  packetData[5] = 0x00;
  packetData[6] = 0x00;
  packetData[7] = 0x00;                      // Reserved or checksum placeholder
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  //Makes sure the port has a device connected to it
  int checksum = 0;
  for (i=0; i < 8; i++) {
    checksum += _connectedDevices[port][i];
  }
  if(checksum == 0){
    Serial.println("No device logged at that the queried port.");
    return 0;
  }

  //Transmits data to selected SSD
  e2b.reset();
  e2b.select(_connectedDevices[port]);
  e2b.write_scratchpad();
  for(i=0; i < 8; i++){
    e2b.write(packetData[i]);
    Serial.print(packetData[i]); Serial.print(" ");
  }
  Serial.println(" ");

  delay(cmd_delay);

  //Reads the data being sent back
  byte present = 0;
  byte data[9];
  delay(cmd_delay);

  present = e2b.reset();
  e2b.select(_connectedDevices[port]);
  e2b.read_scratchpad();

  Serial.print("Data = "); //Serial.print("\t\tData = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for (i=0; i < 9; i++) {           // we only need 2 bytes
    data[i] = e2b.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(cmd_delay);

  return data[0];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////BUTTON FUNCTIONS////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Manages operation of the button
void button_manager(){
  int buttonPressed = 0;
  if(!digitalRead(buttonPin)){
    while(!digitalRead(buttonPin)){
      buttonPressed++;
      delay(10);
    }
    if(buttonPressed < 100){
      refresh_connected_devices();
    }else{
      initialize_connected_devices();
    }
    print_connected_devices();
  }
}

//Updates the list of connected devices (_connectedDevices[][])
void refresh_connected_devices() {
  Serial.println("Starting refresh_connected_devices");
  byte i, j, k;
  byte addr[8];

  //Clears connectedDevices of data
  for (i=0; i < MaxConnectedDeviceNum; i++){
    for (j=0; j < 8; j++) {
      _connectedDevices[i][j] = 0x0;
      int address = (i * 8) + k;
      EEPROM.write(address, 0x0);
      //EEPROM.commit();
    }
  }

  //Adds addresses that are detected back into _connectedDevices based on _previousDevices
  for (i=0; i < MaxConnectedDeviceNum; i++) {
    bool match = 1;
    if (!e2b.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      e2b.reset_search();
      delay(250);
      break;
    }
    //Checks for a match in _previousDevices
    for (j=0; j < MaxConnectedDeviceNum; j++) {
      for (k=0; k < 8; k++) {
        if(_previousDevices[j][k] != addr[k]){
            match = 0;
        }
      }
      //Writes data back into _connectedDevices
      if(match){
        for (k=0; k < 8; k++) {
          _connectedDevices[j][k] = addr[k];
          int address = (i * 8) + k;
          EEPROM.write(address,addr[k]);
          //EEPROM.commit();
        }
      }
    }

  }

  //Find new addresses and add them if not already logged
  for (i=0; i < MaxConnectedDeviceNum; i++) {
    if (!e2b.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      e2b.reset_search();
      delay(250);
      return;
    }

    //Search for the first empty index or a matching address
    bool beenLogged = 0;
    int firstEmptyIndex = -1;
    for (j = 0; j < MaxConnectedDeviceNum; j++) {
      if (!beenLogged) {  // If the device has already been logged, don't keep searching
        bool mismatch = 0;
        bool isEmpty = 1;
        for (k = 0; k < 8; k++) {
          if ((_connectedDevices[j][k] != addr[k]) && (!mismatch)) {  // Scans index for a match
            mismatch = 1;
          }
          if ((_connectedDevices[j][k] != 0x0) && (isEmpty)) {  // Scans empty index
            isEmpty = 0;
          }
        }

        if (isEmpty && (firstEmptyIndex == -1)) {
          firstEmptyIndex = j;
        }
        if (!mismatch) {
          beenLogged = 1;
        }
      }
    }

    //If the address hasn't been logged yet and there is no matching address
    if ((!beenLogged) && (firstEmptyIndex != -1)) {
      for (k = 0; k < 8; k++) {
        _connectedDevices[firstEmptyIndex][k] = addr[k];
        int address = (firstEmptyIndex * 8) + k;
        EEPROM.write(address, addr[k]);
        //EEPROM.commit();
      }
    }
  }

  //Update the previousDevices array with the current _connectedDevices
  for (i = 0; i < MaxConnectedDeviceNum; i++) {
    for (j = 0; j < 8; j++) {
      _previousDevices[i][j] = _connectedDevices[i][j];
    }
  }
}

//Sets all devices in the list to 0
void initialize_connected_devices(){
  int address = 0;
  for(byte i=0; i < MaxConnectedDeviceNum; i++){
    for(byte j=0; j < 8; j++){
      _connectedDevices[i][j] = 0x0;
      EEPROM.write(address, 0x0);
      //EEPROM.commit();
      address++;
    }
  }
}

//Updates the list of connected devices (_connectedDevices[][]) from the internal nonvolatile EEPROM
//Returns TRUE if there is data, returns FALSE if empty
bool get_connected_devices_from_EEPROM(){
  bool isEmpty = 1;
  int address = 0;
  for(byte i=0; i < MaxConnectedDeviceNum; i++){
    for(byte j=0; j < 8; j++){
      _connectedDevices[i][j] = EEPROM.read(address);
      if(_connectedDevices[i][j]){
        isEmpty = 0;
      }
      address++;
    }
  }
  return isEmpty;
}

//Prints the current state of _connectedDevices
void print_connected_devices(){
  Serial.println("_connectedDevices:");
  for(byte i=0; i < MaxConnectedDeviceNum; i++){
    Serial.print("Slot "); Serial.print(i); Serial.print(": ");
    for(byte j=0; j < 8; j++){
      Serial.print(_connectedDevices[i][j],HEX); Serial.print(" ");
    }
    Serial.println();
  }
}