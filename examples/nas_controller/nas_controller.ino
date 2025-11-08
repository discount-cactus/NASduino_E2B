//E2B NAS Controller
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch controls the NAS system and its subsequent SSD's via E2B and uses an ESP32 to communicate with the
device connected to the NAS via ESP-NOW which is also an ESP32 (for simplicity)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <E2B.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>
#include "EEPROM.h"

#define E2B_pin 4
#define ESPNOW_WIFI_CHANNEL 6
#define buttonPin 5
#define MaxConnectedDeviceNum 12

#define cmd_delay 50

// Define the structure to hold the data
struct DataPacket {
  uint8_t _PORTNUM;   // Byte variable for port number
  uint8_t _WR;        // Byte variable for WR
  int _ADR;           // Integer variable for address
  uint8_t _DAT;       // uint16_t variable for data
  uint8_t _DAT2;      // uint16_t variable for data 2
};

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x4B, 0x46, 0x7F, 0xFF, 0x00, 0x10, 0x00};
uint8_t pn;
uint8_t wr;
int adr;
uint8_t dat;
uint8_t dat2;
uint8_t packetData[8];

//Device manager variables
uint8_t _connectedDevices[MaxConnectedDeviceNum][8];
uint8_t _previousDevices[MaxConnectedDeviceNum][8];

//Performance variables
uint8_t connectedMasterNodes = 0;
unsigned long firstPacketTime = 0;
unsigned long packetCount = 0;
float averageArrivalRate = 0.0;   // packets per second
int ssdCount = 0;
bool isCommunicatingWithMarlin = 0;

float averageReadSpeed = 0.0;
float averageWriteSpeed = 0.0;
float averageRoundTripPacketSpeed = 0.0;
float bitErrorRate = 0.0;

E2B e2b(E2B_pin);  // on pin 4 (a 4.7K resistor is necessary)

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

    //Exits if the Marlin UI is issuing commands to the controller
    if(isCommunicatingWithMarlin)
      return;
    
    //Updates the average arrival rate
    unsigned long now = millis();
    if (packetCount == 0)
      firstPacketTime = now;  // first packet sets the baseline

    packetCount++;
    unsigned long elapsed = now - firstPacketTime;
    if (elapsed > 0)
      averageArrivalRate = (float)packetCount / ((float)elapsed / 1000.0);  // packets per second

    if (len == sizeof(DataPacket)){
      DataPacket *packet = (DataPacket *)data;
      //Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
      Serial.println("Received a message from master");
      pn = packet->_PORTNUM;
      wr = packet->_WR;
      adr = packet->_ADR;
      dat = packet->_DAT;
      dat2 = packet->_DAT2;
      /*Serial.print("_PORTNUM: "); Serial.println(pn);
      Serial.print("_WR: "); Serial.println(wr,HEX);
      Serial.print("_ADR: "); Serial.println(adr);
      Serial.print("_DAT: "); Serial.println(dat,HEX);*/
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      if(wr != 0xA && wr != 0xB)    //Exits if the command is not a read or write command
        return;

      packetData[0] = wr;                        // Command: 0xA = write, 0xB = read
      for (i=0; i < 4; i++) {                     //Encodes the address into 4 bytes
        packetData[i + 1] = (adr >> (i * 8)) & 0xFF;  //packetData[i] = (adr >> (i * 8)) & 0xFF;  // Extract each byte
      }
      packetData[5] = dat;             // Data LSB
      packetData[6] = dat2; //highByte(dat);            // Data MSB
      packetData[7] = 0x00;                      // Reserved or checksum placeholder
      ///////////////////////////////////////////////////////////////////////////////////////////////////

      //Makes sure the port has a device connected to it
      int checksum = 0;
      for (uint8_t i=0; i < 8; i++) {
        checksum += _connectedDevices[pn][i];
      }
      if(checksum == 0){
        Serial.println("No device logged at that the queried port.");
        return;
      }

      //Transmits data to selected SSD
      e2b.reset();
      e2b.select(_connectedDevices[pn]);
      e2b.write_scratchpad();
      for(i=0; i < 8; i++){
        e2b.write(packetData[i]);
        Serial.print(packetData[i],HEX); Serial.print(" ");
      }
      Serial.println(" ");
      
      if(wr == 0xB){
        byte present = 0;
        byte data[9];
        delay(cmd_delay);
      
        present = e2b.reset();
        e2b.select(_connectedDevices[pn]);
        e2b.read_scratchpad();

        Serial.print("Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
        for (i=0; i < 9; i++) {           // we only need 2 bytes, 9 shows the whole response packet
          data[i] = e2b.read();
          Serial.print(data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Send a response (if necessary) back to the transmitter:
        uint8_t response = data[0];
        //Serial.print("response: "); Serial.println(response,HEX);
        send_message((uint8_t *)&response, sizeof(response));
        response = data[1];
        if(response != 0x0){
          delay(5);                 //Probably not required
          send_message((uint8_t *)&response, sizeof(response));
        }
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
  run_diagnostics();
  delay(100);
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
/////////////////////////////////////////MARLIN UI FUNCTIONS/////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Handles commands being received by Marlin UI
void marlin_command_manager(){
  isCommunicatingWithMarlin = 1;
  char receivedChar = Serial.read();
  if(receivedChar == 'A'){
    run_diagnostics();
  } else if(receivedChar == 'B'){
    run_speed_test();
  }else{
    Serial.println("Unknown command");
  }
  isCommunicatingWithMarlin = 0;
}

// Empty function for diagnostics (you can add your own code here later)
void run_diagnostics() {
  String dataToSend = "";

  //Updates ssdCount
  ssdCount = 0;
  for (uint8_t i=0; i < MaxConnectedDeviceNum; i++) {
    if(_connectedDevices[i][0]){
      ssdCount++;
    }
  }

  dataToSend += String(ssdCount);
  dataToSend += ",";
  dataToSend += String(connectedMasterNodes);
  dataToSend += ",";
  dataToSend += String(MaxConnectedDeviceNum);
  dataToSend += ",";
  dataToSend += String(averageArrivalRate);

  if(ssdCount){
    for(uint8_t i=0; i < ssdCount; i++){
      byte present = 0;
      byte data[9];

      dataToSend += ",[";

      present = e2b.reset();
      e2b.select(_connectedDevices[i]);
      e2b.read_scratchpad();

      //Serial.print("Data = ");
      //Serial.print(present, HEX);
      //Serial.print(" ");
      for (uint8_t j=0; j < 9; j++) {           // we only need 2 bytes, 9 shows the whole response packet
        data[j] = e2b.read();
        //Serial.print(data[j], HEX);
        //Serial.print(" ");
        dataToSend += String(data[j]);
        if(j < 8)
          dataToSend += ",";
      }
      dataToSend += "]";
      //Serial.println();
      delay(10);
    }
  }
  Serial.println(dataToSend);
}

//Runs a test to determine how fast the transfer speeds are
void run_speed_test(){
  int i;
  int j;
  const int testIterationCount = 100;

  uint8_t pn;
  uint8_t wr = 0xA;
  int adr = 0;
  uint8_t dat = 0xE5;
  uint8_t dat2 = 0x00;

  float _newAverageReadSpeed = 0.0;
  float _newAverageWriteSpeed = 0.0;
  float _newAverageRoundTripPacketSpeed = 0.0;
  float _newBitErrorRate = 0.0;
  float startTime_us = 0.0;
  float endTime_us = 0.0;
  float totalTime_us = 0.0;
  float startTime2_us = 0.0;
  float endTime2_us = 0.0;
  float totalTime2_us = 0.0;
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  packetData[0] = wr;                        // Command: 0xA = write, 0xB = read
  for(i=0; i < 4; i++){                     //Encodes the address into 4 bytes
    packetData[i + 1] = (adr >> (i * 8)) & 0xFF;  //packetData[i] = (adr >> (i * 8)) & 0xFF;  // Extract each byte
  }
  packetData[5] = dat;             // Data LSB
  packetData[6] = dat2;            // Data MSB
  packetData[7] = 0x00;            // Reserved or checksum placeholder
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  //Finds the port number of the first connected device
  int checksum = 0;
  for (i=0; i < MaxConnectedDeviceNum; i++){
    for (j=0; j < 8; j++) {
      checksum += _connectedDevices[i][j];
    }
    if(checksum){
      pn = i;
      break;
    }
  }
  if(!checksum){
    //Serial.println("No device logged at that the queried port.");
    return;
  }

  //Write commands test
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  packetData[0] = 0xA;
  totalTime_us = 0.0;
  for(i=0; i < testIterationCount; i++){
    startTime_us = millis(); //micros();
    e2b.reset();
    e2b.select(_connectedDevices[pn]);
    e2b.write_scratchpad();
    for(j=0; j < 8; j++){
      e2b.write(packetData[j]);
      //Serial.print(packetData[i],HEX); Serial.print(" ");
    }
    //Serial.println(" ");
    endTime_us = millis(); //micros();
    totalTime_us += (endTime_us - startTime_us);
  }
  _newAverageWriteSpeed = totalTime_us / testIterationCount;
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  //Read commands test
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  packetData[0] = 0xB; 
  totalTime_us = 0.0;
  totalTime2_us = 0.0;
  int errors = 0;
  for(i=0; i < testIterationCount; i++){
    //Write portion of test
    startTime2_us = millis(); //micros();
    e2b.reset();
    e2b.select(_connectedDevices[pn]);
    e2b.write_scratchpad();
    for(j=0; j < 8; j++){
      e2b.write(packetData[j]);
      //Serial.print(packetData[i],HEX); Serial.print(" ");
    }
    //Serial.println(" ");

    delay(cmd_delay);

    //Read portion of test
    startTime_us = millis(); //micros();
    byte present = 0;
    byte data[9];
    present = e2b.reset();
    e2b.select(_connectedDevices[pn]);
    e2b.read_scratchpad();

    //Serial.print("Data = ");
    //Serial.print(present, HEX);
    //Serial.print(" ");
    for (j=0; j < 9; j++) {           // we only need 2 bytes, 9 shows the whole response packet
      data[j] = e2b.read();
      //Serial.print(data[j], HEX);
      //Serial.print(" ");
    }
    //Serial.println();
    if(data[0] != 0xE5)
        errors++;
    endTime_us = millis(); //micros();
    endTime2_us = millis(); //micros();
    totalTime_us += (endTime_us - startTime_us);
    totalTime2_us += (endTime2_us - startTime2_us);
  }
  _newAverageReadSpeed = totalTime_us / testIterationCount;
  _newAverageRoundTripPacketSpeed = totalTime2_us / testIterationCount;
  _newBitErrorRate = errors / testIterationCount;
  ///////////////////////////////////////////////////////////////////////////////////////////////////

  //Updates variables and sends response to Marlin GUI
  averageReadSpeed = _newAverageReadSpeed;
  averageWriteSpeed = _newAverageWriteSpeed;
  averageRoundTripPacketSpeed = _newAverageRoundTripPacketSpeed;
  bitErrorRate = _newBitErrorRate;

  String speedTestRsponse = "";
  speedTestRsponse += String(averageReadSpeed);
  speedTestRsponse += ", ";
  speedTestRsponse += String(averageWriteSpeed);
  speedTestRsponse += ", ";
  speedTestRsponse += String(averageRoundTripPacketSpeed);
  speedTestRsponse += ", ";
  speedTestRsponse += String(bitErrorRate);
  Serial.println(speedTestRsponse); // Sends response back to the Marlin UI
}