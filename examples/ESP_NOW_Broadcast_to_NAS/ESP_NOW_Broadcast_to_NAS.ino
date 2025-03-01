#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>

#define ESPNOW_WIFI_CHANNEL 6

// Define the structure to hold the data
struct DataPacket {
  uint8_t _PORTNUM;   // Byte variable for port number
  uint8_t _WR;        // Byte variable for WR
  int _ADR;           // Integer variable for address
  uint16_t _DAT;      // uint16_t variable for data
};

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
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
ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

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

  Serial.println("Setup complete. Broadcasting messages every 5 seconds.");
}

void loop() {
  // Prepare the data to send
  DataPacket packet;
  packet._PORTNUM = 5;    // Example port number
  packet._WR = 1;         // Example WR value (write command)
  packet._ADR = 26281432; // Example address (integer)
  packet._DAT = 0xA3F1;   // Example data (uint16_t)

  // Print the message being sent for debugging
  Serial.printf("Broadcasting message: Port: %d, WR: %d, ADR: %d, DAT: %u\n", 
                packet._PORTNUM, packet._WR, packet._ADR, packet._DAT);

  // Send the message
  if (!broadcast_peer.send_message((uint8_t *)&packet, sizeof(packet))) {
    Serial.println("Failed to broadcast message");
  }

  delay(5000);  // Broadcast every 5 seconds
}
