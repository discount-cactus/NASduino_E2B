//E2B NAS Controller
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch controls the NAS system and its subsequent SSD's via E2B and uses an ESP32 to communicate with the
device connected to the NAS via ESP-NOW which is also an ESP32 (for simplicity)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*TO-DO:
-Update _connectedDevices to make sure there's no missing devices that should be there
*/

#include <E2B.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>
#include "EEPROM.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

#define E2B_pin 4
#define ESPNOW_WIFI_CHANNEL 6
#define buttonPin 5
#define MaxConnectedDeviceNum 12

#define cmd_delay 200
#define fanEnable 1

// Define the structure to hold the data
struct DataPacket {
  uint8_t _PORTNUM;   // Byte variable for port number
  uint8_t _WR;        // Byte variable for WR
  int _ADR;           // Integer variable for address
  uint8_t _DAT;      // uint16_t variable for data
};

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x4B, 0x46, 0x7F, 0xFF, 0x00, 0x10, 0x00};
uint8_t pn;
uint8_t wr;
int adr;
uint16_t dat;
uint8_t packetData[8];

uint8_t _connectedDevices[MaxConnectedDeviceNum][8];
uint8_t _previousDevices[MaxConnectedDeviceNum][8];

bool isPerformingMarlinCommand = 0;
uint8_t connectedMasterNodes = 0;
float averageArrivalRate = 0.0;
uint8_t fanState = 0;
int fanPin = 7;

E2B e2b(E2B_pin);  // on pin 4 (a 4.7K resistor is necessary)
Adafruit_INA219 ina219;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////ESP-NOW FUNCTIONS////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes for peer management, same as original code
class NAS_Controller_ESP_NOW_Peer : public ESP_NOW_Peer {
public:
  NAS_Controller_ESP_NOW_Peer(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
  ~NAS_Controller_ESP_NOW_Peer() {}

  bool add_peer() {
    if (!add()){
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }
  
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }

  // Callback to handle receiving data
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    int i;
    if ((len == sizeof(DataPacket)) && (!isPerformingMarlinCommand)){
      DataPacket *packet = (DataPacket *)data;
      //Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
      Serial.println("Received a message from master");
      pn = packet->_PORTNUM;
      wr = packet->_WR;
      adr = packet->_ADR;
      dat = packet->_DAT;
      /*Serial.print("_PORTNUM: "); Serial.println(pn);
      Serial.print("_WR: "); Serial.println(wr,HEX);
      Serial.print("_ADR: "); Serial.println(adr);
      Serial.print("_DAT: "); Serial.println(dat,HEX);*/
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      //Determines if the instruction is to read or write
      /*if(wr == 0xA)
        ssd_write(pn,adr,dat);
      if(wr == 0xB)
        uint16_t response = ssd_read(pn,adr);
        //Serial.print("response: "); Serial.println(response,HEX);
        send_message((uint8_t *)&response, sizeof(response));*/
      if(wr != 0xA && wr != 0xB)
        return;

      packetData[0] = wr;                        // Command: 0xA = write, 0xB = read
      packetData[1] = adr & 0xFF;                // LSB
      packetData[2] = (adr >> 8) & 0xFF;
      packetData[3] = (adr >> 16) & 0xFF;
      packetData[4] = (adr >> 24) & 0xFF;        // MSB
      for (i=0; i < 4; i++) {                     //Encodes the address into 4 bytes
        packetData[i + 1] = (adr >> (i * 8)) & 0xFF;  //packetData[i] = (adr >> (i * 8)) & 0xFF;  // Extract each byte
      }
      packetData[5] = lowByte(dat);             // Data LSB
      packetData[6] = highByte(dat);            // Data MSB
      packetData[7] = 0x00;                      // Reserved or checksum placeholder
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      //Searches for device
      //This section is not needed if only one device is connected

      int checksum = 0;
      for (uint8_t i=0; i < 8; i++) {
        checksum += _connectedDevices[pn][i];
      }
      if(checksum == 0){
        Serial.println("No device logged at that the queried port.");
        return;
      }

      //Transmits data
      e2b.reset();
      e2b.select(_connectedDevices[pn]);
      e2b.write_scratchpad();
      for(i=0; i < 8; i++){
        e2b.write(packetData[i]);
        Serial.print(packetData[i]); Serial.print(" ");
      }
      Serial.println(" ");
      
      if(wr == 0xB){
        byte present = 0;
        byte data[9];
        delay(cmd_delay);
      
        present = e2b.reset();
        e2b.select(_connectedDevices[pn]);
        e2b.read_scratchpad();

        Serial.print("Data = "); //Serial.print("\t\tData = ");
        Serial.print(present, HEX);
        Serial.print(" ");
        for (i=0; i < 2; i++) {           // we only need 2 bytes
          data[i] = e2b.read();
          Serial.print(data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        //delay(50);

        // Send a response (if necessary) back to the transmitter:
        uint8_t response = data[0];
        //Serial.print("response: "); Serial.println(response,HEX);
        send_message((uint8_t *)&response, sizeof(response));
      }

      delay(cmd_delay);

    }else{
      Serial.println("Received invalid data");
    }
  }
};

// List of all the masters. It will be populated when a new master is registered
std::vector<NAS_Controller_ESP_NOW_Peer> masters;

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    NAS_Controller_ESP_NOW_Peer new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    connectedMasterNodes++;

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("E2B NAS Controller.");
  pinMode(buttonPin,INPUT_PULLUP);
  pinMode(fanPin, OUTPUT);

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

  connectedMasterNodes = 0;

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Example - Broadcast Slave");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    delay(5000);
    ESP.restart();
  }

  ESP_NOW.onNewPeer(register_new_master, NULL);
  Serial.println("Setup complete. Waiting for a master to broadcast a message...");
}

void loop(){
  button_manager();
  if (Serial.available() > 0){
    marlin_command_manager();
  }
  fan_manager();
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
  connectedMasterNodes = 0;
  
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////MARLIN UI FUNCTIONS///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Handles commands being received by Marlin UI
void marlin_command_manager(){
  isPerformingMarlinCommand = 1;
  char receivedChar = Serial.read();
  if(receivedChar == 'A'){
    run_diagnostics();
  } else if(receivedChar == 'B'){
    run_speedtest();
  } else if(receivedChar == 'C'){
    find_capacity();
  } else if(receivedChar == '0'){
    if(fanEnable){
      if(fanState == 0){
        fanState = 4;
        Serial.write("1");
      }else{
        fanState = 0;
        Serial.write("0");
      }
    }
  } else if(receivedChar == '1'){
    if(fanEnable){
      fanState = 1;
    }
  } else if(receivedChar == '2'){
    if(fanEnable){
      fanState = 2;
    }
  } else if(receivedChar == '3'){
    if(fanEnable){
      fanState = 3;
    }
  } else {
    Serial.println("Unknown command");
  }
  isPerformingMarlinCommand = 0;
}

// Empty function for diagnostics (you can add your own code here later)
void run_diagnostics() {
  String dataToSend = "";

  dataToSend += String(connectedMasterNodes);
  dataToSend += ",";
  dataToSend += String(ina219.begin()); //may need to add its I2C address (0x40)
  dataToSend += ",";
  dataToSend += String(fanEnable);
  dataToSend += ",";
  dataToSend += String(averageArrivalRate);
  /*dataToSend += ",";
  dataToSend += String();
  dataToSend += ",";
  dataToSend += String();
  dataToSend += ",";*/

  //ADD MASTER DIAGNOSTICS TO THIS

  //Adds SSD diagnostics to string
  byte data[9];
  byte addr[8];
  if(!e2b.search(addr)){
    Serial.println("No more addresses.");
    Serial.println();
    e2b.reset_search();
    delay(250);
    return;
  }
  
  /*Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }*/

  byte present = e2b.reset();
  e2b.select(addr);    
  e2b.write(0x9F);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for(byte i=0; i < 12; i++) {           // we need 12 bytes
    data[i] = e2b.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
    dataToSend += String(data[i]);
    dataToSend += ",";
  }

  Serial.println(dataToSend);
}

//Runs a test to determine how fast the transfer speeds are
void run_speedtest(){
  delay(6000);
  Serial.write("Groovy,6.74 kb/s, 5.21 kb/s,1,1,0,4");
}

//Finds the capacity of each SSD
void find_capacity(){
  delay(3000);  // Simulate computation
  Serial.write("0.2164\n");  // Adding \n to indicate end of message
}

//Manages operation of on-board fan (when connected)
void fan_manager(){
  if(fanEnable){
    switch(fanState){
      case 0:
        digitalWrite(fanPin,LOW);
        break;
      case 1:
        digitalWrite(fanPin,HIGH);
        break;
      case 2:
        digitalWrite(fanPin,LOW);
        break;
      case 3:
        digitalWrite(fanPin,HIGH);
        break;
      case 4:
        digitalWrite(fanPin,LOW);
        break;
    }
  }
}