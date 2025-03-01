//E2B NAS Controller
//The NAS controller is any uC that manages connected SSD's and communicates wirelessly with devices requiring additional storage
//Manages connecting to master device via ESP-NOW (refer to ESP_NOW_Broadcast_Slave sketch)

/*TO-DO:
-Function that re-initializes the list of connecteed SSD's (with the push of a button)
-Connectivity to master via ESP-NOW
*/
#include <E2B.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>

#define E2B_pin 2
#define ESPNOW_WIFI_CHANNEL 6
#define buttonPin 5

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

E2B e2b(E2B_pin);  // on pin 2 (a 4.7K resistor is necessary)

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
  
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == sizeof(DataPacket)) {
      DataPacket *packet = (DataPacket *)data;
      Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
      /*Serial.printf("  Port: %d, WR: %d, ADR: %d, DAT: %u\n", 
                    packet->_PORTNUM, packet->_WR, packet->_ADR, packet->_DAT);*/
      
      pn = packet->_PORTNUM;
      wr = packet->_WR;
      adr = packet->_ADR;
      dat = packet->_DAT;

      Serial.print("_PORTNUM: "); Serial.println(pn);
      Serial.print("_WR: "); Serial.println(wr);
      Serial.print("_ADR: "); Serial.println(adr);
      Serial.print("_DAT: "); Serial.println(dat);
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

void setup(){
  Serial.begin(9600);
  while(!Serial);
  Serial.println("E2B NAS Controller.");
  pinMode(buttonPin,INPUT_PULLUP);

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
void prepareData(String wr, int address, uint16_t data){
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
}




