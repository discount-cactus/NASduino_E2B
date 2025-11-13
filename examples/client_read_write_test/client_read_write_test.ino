//Client ESP32 - Simple read adn write test
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*NOTES:
This sketch uses an ESP32 to communicate to the NAS Controller which is also an ESP32
via ESP-NOW to do a simple read and write to one of the SSD's in the NAS
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#define ESPNOW_WIFI_CHANNEL 6

// Define the structure to hold the data
struct DataPacket {
  uint8_t _PORTNUM;   // Byte variable for port number
  uint8_t _WR;        // Byte variable for WR
  int _ADR;           // Integer variable for address
  uint8_t _DAT;       // uint8_t variable for data
  uint8_t _DAT2;      // uint8_t variable for data 2
};

struct InfoPacket {
  uint8_t _CMD;
  uint8_t _clientType;
  uint32_t _ip;
  char _name[16];
  char _password[16];
  uint8_t _powerDraw_mA;
};


uint8_t queriedData = 0x00;
uint8_t queriedData2 = 0x00;

class ESP_NOW_Client_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Client_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Client_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

// Create a broadcast peer object
ESP_NOW_Client_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

uint16_t receivedData = 0; // Variable to store the received uint16_t data

// Callback function to handle incoming messages
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len){
  uint8_t received = 0;
  if (len == sizeof(uint8_t)) {
    received = data[0];
    //Serial.printf("Received response: 0x%04X\n", received);

    if(queriedData == 0x0){
      queriedData = received;
      Serial.println(queriedData,HEX);
    }else{
      queriedData2 = received;
      Serial.println(queriedData2,HEX);
    }
  }else{
    // If the received data is not the expected size, print an error
    Serial.println("Error: Received data of unexpected size.");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Example - Broadcast Master");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    delay(5000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onReceive);    // Register the onReceive callback to listen for incoming messages

  nas_send_properties_to_controller();

  Serial.println("Setup complete. Broadcasting messages every 2 seconds.");
}

void loop(){
  nas_send_properties_to_controller();
  delay(1000);
  query_nas_write(0,17,0xE0D4);
  delay(1000);
  clear_buffer();
  query_nas_read(0,17);
  delay(1000);
}



//Sends essential data about the client to the NAS controller
void nas_send_properties_to_controller() {
  InfoPacket info;
  info._CMD = 0xC; // "Info" command
  info._clientType = 0xA8;
  info._ip = WiFi.localIP();
  strncpy(info._name, "ClientNode01", sizeof(info._name));
  strncpy(info._password, "pass1234", sizeof(info._password));
  info._powerDraw_mA = 108;

  if (!broadcast_peer.send_message((uint8_t *)&info, sizeof(info))) {
    Serial.println("Failed to broadcast client info message");
  } else {
    Serial.println("Client info sent to NAS controller");
  }
}

//Writes to the NAS
void query_nas_write(uint8_t port, int address, uint16_t dat){
  // Prepare the data to send
  DataPacket packet;
  packet._PORTNUM = port;    // Example port number
  packet._WR = 0xA;         // Example WR value (write command)
  packet._ADR = address; // Example address (integer)

  if(lowByte(dat) == 0x0 || highByte(dat) == 0x0){
    if(lowByte(dat) == 0x0){
      packet._DAT = dat = highByte(dat);
      packet._DAT2 = 0x00;
    }else{
      packet._DAT = dat = lowByte(dat);
      packet._DAT2 = 0x00;
    }
  }else{
    packet._DAT = highByte(dat);   // Example data (uint8_t)
    packet._DAT2 = lowByte(dat);   // Example data (uint8_t)
  }

  // Print the message being sent for debugging
  Serial.printf("Broadcasting message: Port: %d, WR: %d, ADR: %d, DAT: %u, DAT2: %u\n", 
                packet._PORTNUM, packet._WR, packet._ADR, packet._DAT, packet._DAT2);

  // Send the message
  if (!broadcast_peer.send_message((uint8_t *)&packet, sizeof(packet))) {
    Serial.println("Failed to broadcast message");
  }
}

//Reads from the NAS
void query_nas_read(uint8_t port, int address){
  // Prepare the data to send
  DataPacket packet;
  packet._PORTNUM = port;   // Example port number
  packet._WR = 0xB;         // Example WR value (write command)
  packet._ADR = address;    // Example address (integer)
  packet._DAT = 0x00;       // Example data (uint8_t)
  packet._DAT2 = 0x00;      // Example data (uint8_t)

  // Print the message being sent for debugging
  Serial.printf("Broadcasting message: Port: %d, WR: %d, ADR: %d, DAT: %u, DAT2: %u\n", 
                packet._PORTNUM, packet._WR, packet._ADR, packet._DAT, packet._DAT2);

  // Send the message
  if (!broadcast_peer.send_message((uint8_t *)&packet, sizeof(packet))) {
    Serial.println("Failed to broadcast message");
  }
}

void clear_buffer(){
  queriedData = 0x00;
  queriedData2 = 0x00;
}