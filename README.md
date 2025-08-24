# NASduino (E2B)
NASduino is a simple Arduino-based NAS for enabling small-scale projects to access large amounts of data in a simple and scalable manner. This is made capable through my single-wire communications protocol E2B.

## Required materials
- 2 x ESP32's
- 1-12 x Macchiato SSD's
- Pushbutton
- Wires

## Wiring
# NAS
The default NAS setup uses two ESP32's (along with the SSD board I developed) used as a host and another as the NAS controller and communicate through ESP-NOW. This is the default setup but other ways are certainly possible with minor modification to this circuit setup.

![bd_nas_wiring1 drawio](https://github.com/user-attachments/assets/74ab05f2-4f83-47d9-a0aa-296c534ff14b)

- Host device: No additional hardware
- NAS Controller:
    - Button: Connected between pin 5 and GND
    - E2B line: Connected to pin 4 and Macchiato SSD's E2B pin
- Macchiato SSD:
    - E2B pin: Connected to E2B pin and pin 4 on NAS Controller device

# DAS
The DAS uses almost the same setup as the NAS wiring, with the absence of one ESP32 while the other one now assumes the role of the host and NAS controller.

![bd_das_wiring drawio](https://github.com/user-attachments/assets/84d9dd21-dea7-4410-9cba-858ff7782e50)

- Host Device/NAS Controller:
    - Button: Connected between pin 5 and GND
    - E2B line: Connected to pin 4 and Macchiato SSD's E2B pin
- Macchiato SSD:
    - E2B pin: Connected to E2B pin and pin 4 on NAS Controller device

# Software Setup
