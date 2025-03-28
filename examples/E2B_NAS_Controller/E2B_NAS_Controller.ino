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

#define E2B_pin 4
#define ESPNOW_WIFI_CHANNEL 6
#define buttonPin 5
#define MaxConnectedDeviceNum 12

// Define the structure to hold the data
struct DataPacket {
  uint8_t _PORTNUM;   // Byte variable for port number
  uint8_t _WR;        // Byte variable for WR
  int _ADR;           // Integer variable for address
  uint16_t _DAT;      // uint16_t variable for data
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

E2B e2b(E2B_pin);  // on pin 4 (a 4.7K resistor is necessary)


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
    if (len == sizeof(DataPacket)){
      DataPacket *packet = (DataPacket *)data;
      Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
      pn = packet->_PORTNUM;
      wr = packet->_WR;
      adr = packet->_ADR;
      dat = packet->_DAT;
      // Print the received data
      /*Serial.print("_PORTNUM: "); Serial.println(pn);
      Serial.print("_WR: "); Serial.println(wr,HEX);
      Serial.print("_ADR: "); Serial.println(adr);
      Serial.print("_DAT: "); Serial.println(dat,HEX);*/
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      //Determines if the instruction is to read or write
      packetData[0] = wr;
      for (i=1; i < 5; i++) {                     //Encodes the address into 4 bytes
        packetData[i] = (adr >> (i * 8)) & 0xFF;  // Extract each byte
      }
      packetData[5] = lowByte(dat);
      packetData[6] = highByte(dat);
      packetData[7] = 0x00;
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      //Searches for device
      //This section is not needed if only one device is connected
      /*byte addr[8];
      if(!e2b.search(addr)){
        //Serial.println("No more addresses.");
        //Serial.println();
        e2b.reset_search();
        delay(250);
        return;
      }*/
      Serial.println();

      Serial.print("ROM =");
      for(i=0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(_connectedDevices[pn][i], HEX);
      }

      //Transmits ddata
      e2b.reset();
      e2b.select(_connectedDevices[pn]);
      e2b.write_scratchpad();
      for(i=0; i < 8; i++){
        e2b.write(packetData[i]);
      }
      
      if(wr == 0xB){
        byte present = 0;
        byte data[9];
        delay(300);   //Must be greater than 150ms
      
        present = e2b.reset();
        e2b.select(_connectedDevices[pn]);
        e2b.read_scratchpad();

        Serial.print("\t\tData = ");
        Serial.print(present, HEX);
        Serial.print(" ");
        for (i=0; i < 2; i++) {           // we only need 2 bytes
          data[i] = e2b.read();
          Serial.print(data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        delay(50);

        // Send a response (if necessary) back to the transmitter:
        uint16_t response = data[1]<<8 | data[0];
        //Serial.print("response: "); Serial.println(response,HEX);
        send_message((uint8_t *)&response, sizeof(response));
      }

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

//Updates the list of connected devices (_connectedDevices[][])
void IRAM_ATTR refresh_connected_devices() {
  Serial.println("Starting refresh_connected_devices");
  byte i, j, k;
  byte addr[8];

  //Clears connectedDevices of data
  for (i=0; i < MaxConnectedDeviceNum; i++){
    for (j=0; j < 8; j++) {
      _connectedDevices[i][j] = 0x0;
      int address = (i * 8) + k;
      EEPROM.write(address, 0x0);
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
        }
      }
    }

  }

  //Find new addresses and add them if not already logged
  for (i = 0; i < MaxConnectedDeviceNum; i++) {
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
}


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