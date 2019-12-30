#ifndef LOLIND32
#define LOLIND32
// Add TFT Display

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735

#define TFT_CS  13 //
#define TFT_RST 15 // Or set to -1 and connect to RESET pin (There is no pin)
#define TFT_DC  2 // A0
#define TFT_SDA 0 // Data out
#define TFT_SCK 12 // Clock out

#define OLED_SCL 22
#define OLED_SDA 4   // LOLIN32 dev board does not have 21 available

void myLogging(char * text);
void tftSetup(int ver, char * ssid);
void tftFossasatLogo();
void tftWifiLogo(bool wifi_connected, String ipaddress, bool mqtt_connected, bool statusReached);
void tftDrawString(uint8_t row, uint8_t col, String text, int textColor);
void tftDrawSatOnMap(int x, int y);
void tftStatus(float batteryChargingVoltage,
               float batteryChargingCurrent,
               float batteryVoltage,
               float solarCellAVoltage,
               float solarCellBVoltage,
               float solarCellCVoltage,
               float batteryTemperature,
               float boardTemperature,
               int mcuTemperature,
               int resetCounter,
               float last_packet_received_rssi,
               float last_packet_received_snr,
               float last_packet_received_frequencyerror,
               String last_packet_received_time,
               bool wifi_connected,
               String ipaddress,
               bool mqtt_connected,
               String station,
               int sat_pos_x,
               int sat_pos_y,
               bool statusReached);
#endif
