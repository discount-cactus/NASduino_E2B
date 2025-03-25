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

E2B e2b(E2B_pin);  // on pin 4 (a 4.7K resistor is necessary)


// Classes for peer management, same as original code
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
  ~ESP_NOW_Peer_Class() {}

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
    if (len == sizeof(DataPacket)) {
      DataPacket *packet = (DataPacket *)data;
      Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");

      pn = packet->_PORTNUM;
      wr = packet->_WR;
      adr = packet->_ADR;
      dat = packet->_DAT;

      // Print the received data
      Serial.print("_PORTNUM: "); Serial.println(pn);
      Serial.print("_WR: "); Serial.println(wr,HEX);
      Serial.print("_ADR: "); Serial.println(adr);
      Serial.print("_DAT: "); Serial.println(dat,HEX);

      int i;
      ///////////////////////////////////////////////////////////////////////////////////////////////////
      //Determines if the instruction is to read or write
      packetData[0] = wr;
    
      //Encodes the address into 4 bytes
      for (i=1; i < 5; i++) {
        packetData[i] = (adr >> (i * 8)) & 0xFF; // Extract each byte
      }

      //Encodes the data
      packetData[5] = lowByte(dat);
      packetData[6] = highByte(dat);
      packetData[7] = 0x00;
      ///////////////////////////////////////////////////////////////////////////////////////////////////

      byte present = 0;
      byte data[9];
      byte addr[8];
      
      //Searches for device
      //This section is not neededd if only one device is connected
      if(!e2b.search(addr)){
        Serial.println("No more addresses.");
        Serial.println();
        e2b.reset_search();
        delay(250);
        return;
      }
      
      Serial.print("ROM =");
      for(i=0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
      }

      if (E2B::crc8(addr, 7) != addr[7]) {
          Serial.println("CRC is not valid!");
          return;
      }
      /*Serial.println();
      for(i=0; i < 8; i++){
        addr[i] = _connectedDevices[pn][i];
      }*/

      //Transmits ddata
      e2b.reset();
      e2b.select(addr);
      e2b.write_scratchpad();
      for(i=0; i < 8; i++){
        e2b.write(packetData[i]);
      }
      
      delay(300);
      
      present = e2b.reset();
      e2b.select(addr);
      e2b.read_scratchpad(); //e2b.write(0xBE);         // Read Scratchpad

      Serial.print("\t\tData = ");
      Serial.print(present, HEX);
      Serial.print(" ");
      for (i=0; i < 2; i++) {           // we need 9 bytes
        data[i] = e2b.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      delay(50);

      // Send a response (if necessary) back to the transmitter:
      uint16_t response = 0x1234;  // Example response
      //uint16_t response = data[1]<<8 | data[0];
      send_message((uint8_t *)&response, sizeof(response));
    } else {
      Serial.println("Received invalid data");
    }
  }
};

// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

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

void IRAM_ATTR refresh_connected_devices(){
  Serial.println("Starting refresh_connected_devices");
  byte i,j,k;
  byte addr[8];
  
  //Finds a new address
  for(i=0; i < MaxConnectedDeviceNum; i++){
    if(!e2b.search(addr)){
      Serial.println("No more addresses.");
      Serial.println();
      e2b.reset_search();
      delay(250);
      return;
    }
    
    //Prints the address
    /*Serial.print("ROM =");
    for(j=0; j < 8; j++) {
      Serial.write(' ');
      Serial.print(addr[j], HEX);
    }
    if (E2B::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();*/

    //Searches for first empty index and for matches of address
    bool beenLogged = 0;
    int firstEmptyIndex = -1;
    for(j=0; j < MaxConnectedDeviceNum; j++){
      if(!beenLogged){        //If the devices haas already been logged, don't keep searching
        bool mismatch = 0;
        bool isEmpty = 1;
        for(k=0; k < 8; k++){
          if((_connectedDevices[j][k] != addr[k]) && (!mismatch)){      //Scans index for a match
            mismatch = 1;
          }
          if((_connectedDevices[j][k] != 0x0) && (isEmpty)){            //Scans empty index
            isEmpty = 0;
          }
        }
        /*if(isEmpty){
          //Update _connectedDevices to make sure there's no missing devices that should be there
        }*/
        if((isEmpty) && (firstEmptyIndex == -1)){
          firstEmptyIndex = j;
          //Serial.print("First empty index found at: "); Serial.println(j);
        }
        if(!mismatch){
          beenLogged = 1;
          //Serial.print("Device already logged at index: "); Serial.println(j);
        }
      }
    }

    //If the address hasn't been logged yet and there is no matching address
    if((!beenLogged) && (firstEmptyIndex != -1)){
      //Serial.print("Logging new device into index: "); Serial.println(firstEmptyIndex);
      for(k=0; k < 8; k++){
        _connectedDevices[firstEmptyIndex][k] = addr[k];
      }
    }


  }
}

void initialize_connected_devices(){
  for(byte i=0; i < MaxConnectedDeviceNum; i++){
    for(byte j=0; j < 8; j++){
      _connectedDevices[i][j] = 0x0;
    }
  }
}

void print_connected_devices(){
  //Prints the current state of _connectedDevices
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
  while(!Serial);
  Serial.println("E2B NAS Controller.");
  pinMode(buttonPin,INPUT_PULLUP);
  //attachInterrupt(buttonPin,refresh_connected_devices,FALLING);

  initialize_connected_devices();

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
  //Serial.println(digitalRead(buttonPin));
  /*prepareData("WRITE", 26281432, 0xA3F1);

  byte i;
  byte present = 0;
  byte data[9];
  byte addr[8];
  
  //Searches for device
  //This section is not neededd if only one device is connected
  if(!e2b.search(addr)){
    Serial.println("No more addresses.");
    Serial.println();
    e2b.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for(i=0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (E2B::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();

  //Transmits ddata
  e2b.reset();
  e2b.select(addr);
  e2b.write_scratchpad(); //e2b.write(0x4E);        //Writes the following data to the transciever's scratchpad
  for(i=0; i < 8; i++){
    e2b.write(packetData[i]);
  }
  
  delay(1000);*/
  
  /*present = e2b.reset();
  e2b.select(addr);
  e2b.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for (i=0; i < 9; i++) {           // we need 9 bytes
    data[i] = e2b.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(E2B::crc8(data, 8), HEX);
  Serial.println();*/
}







//Prepares data and instructions to be sent to transciever and SSD
/*void prepareData(String wr, int address, uint16_t data){
  int i;

  //Determines if the instruction is to read or write
  if(wr == "WRITE"){
    packetData[0] = 0xA;
  }else if(wr == "READ"){
    packetData[0] = 0xB;
  }
 
  //Encodes the address into 4 bytes
  for (i=1; i < 5; i++) {
    packetData[i] = (address >> (i * 8)) & 0xFF; // Extract each byte
  }

  //Encodes the data
  packetData[5] = lowByte(data);
  packetData[6] = highByte(data);
  packetData[7] = 0x00;
}*/