#include "lolind32.h"
#include "graphics.h"

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCK, TFT_RST);

void drawXbm(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *xbm, int textColor) {
  int16_t widthInXbm = (width + 7) / 8;
  uint8_t data = 0;

  for(int16_t y = 0; y < height; y++) {
    for(int16_t x = 0; x < width; x++ ) {
      if (x & 7) {
        data >>= 1; // Move a bit
      } else {  // Read new data every 8 bit
        data = pgm_read_byte(xbm + (x / 8) + y * widthInXbm);
      }
      // if there is a bit draw it
      if (data & 0x01) {
        tft.drawPixel(xMove + x, yMove + y, textColor);
      }
    }
  }
}


void tftDrawString(uint8_t row, uint8_t col, String text, int textColor){
  tft.setTextColor(textColor);
  tft.setCursor(row, col);
  tft.println(text);
}

char text1[60];
char text2[60];
char text3[60];
char text4[60];
char text5[60];
char text6[60];
char text7[60];
char text8[60];

void myLogging(char * text){
  strcpy(text1, text2);
  strcpy(text2, text3);
  strcpy(text3, text4);
  strcpy(text4, text5);
  strcpy(text5, text6);
  strcpy(text6, text7);
  strcpy(text7, text8);
  strcpy(text8, text);
  tftDrawString( 0, 0, text1, ST7735_WHITE);
  tftDrawString( 8, 0, text2, ST7735_WHITE);
  tftDrawString(16, 0, text3, ST7735_WHITE);
  tftDrawString(24, 0, text4, ST7735_WHITE);
  tftDrawString(32, 0, text5, ST7735_WHITE);
  tftDrawString(40, 0, text6, ST7735_WHITE);
  tftDrawString(48, 0, text7, ST7735_WHITE);
  tftDrawString(56, 0, text8, ST7735_WHITE);
}
//
// WiFI logo
//
void tftWifiLogo(bool wifi_connected, String ipaddress, bool mqtt_connected, bool statusReached) {
  tft.fillRect(0, 0, 96, 8, ST7735_BLACK);
  tft.fillRect(0, 8, 160, 128, ST7735_BLACK);
  drawXbm(30 , 15 , WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits, ST7735_GREEN);
  if ( wifi_connected ){
    tftDrawString(40,55,"WiFi Connected ",ST7735_GREEN);
    tftDrawString(40,65, ipaddress, ST7735_GREEN);
  } else {
    tftDrawString(40,55,"WiFi Connecting",ST7735_GREEN);
  }

  if ( mqtt_connected ){ tftDrawString(40,75,"MQTT On ",ST7735_GREEN); } else { tftDrawString(40,75,"MQTT Off",ST7735_GREEN); }
  if ( statusReached ){ tftDrawString(40,85,"Status reached    ",ST7735_GREEN); } else { tftDrawString(40,85,"Status not reached",ST7735_RED); }
}

//
// Fossasat Logo
//
void tftFossasatLogo(String station){
  tft.fillRect(0, 0, 96, 8, ST7735_BLACK);
  tft.fillRect(0, 8, 160, 128, ST7735_BLACK);
  drawXbm(20 , 30 , Fossa_Logo_width, Fossa_Logo_height, Fossa_Logo_bits, ST7735_GREEN);
  tftDrawString(30,70,"sta: "+station,ST7735_GREEN);
}

struct tm timeinfo;
char timbuf[9];
uint8_t lastSec = 61;
uint8_t lastSecondDone = 0;

void tftShowTime(uint8_t x, uint8_t y){
  if(!getLocalTime(&timeinfo)){
    timeinfo.tm_sec = 0;
    timeinfo.tm_min = 0;
    timeinfo.tm_hour = 0;
  }
  if ( timeinfo.tm_sec != lastSec ) {
    // blank the time field ( once a second )
    tft.fillRect(x, y, 60, 8, ST7735_BLACK);
    lastSec = timeinfo.tm_sec;
  }
  sprintf(timbuf, "  %d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  tftDrawString( x, y, timbuf, ST7735_YELLOW);
}

// Fix this //////
unsigned long Xtick_interval;
int Xtick_timing = 100;
int XgraphVal = 1;
int Xdelta = 1;

void tftDrawSatOnMap(int sat_pos_x, int sat_pos_y) {
  tft.fillRect(0, 0, 96, 8, ST7735_BLACK);
  tft.fillRect(0, 8, 160, 128, ST7735_BLACK);

  tft.fillRect(83,0,128,11,ST7735_BLACK);

  if (sat_pos_x == 0 && sat_pos_y == 0) {
    tftDrawString( 0, 45, "Waiting for FossaSat Pos", ST7735_RED );
  }
  else {
    int x = 0;
    int y = 30;
    drawXbm(x, y, earth_width, earth_height, earth_bits, ST7735_GREEN);

    if ((millis()-Xtick_interval)>Xtick_timing) {
    // Change the value to plot
        XgraphVal+=Xdelta;
        Xtick_interval=millis();
    // If the value reaches a limit, then change delta of value
          if (XgraphVal >= 6)      {Xdelta = -1; Xtick_timing=50; }// ramp down value
          else if (XgraphVal <= 1) {Xdelta = +1; Xtick_timing=100;} // ramp up value
    }
    tft.fillCircle(sat_pos_x+x, sat_pos_y+y, XgraphVal+1,    ST7735_BLACK);
    tft.drawCircle(sat_pos_x+x, sat_pos_y+y, XgraphVal,      ST7735_WHITE);
    tft.drawCircle(sat_pos_x+x, sat_pos_y+y, (XgraphVal/3)+1,ST7735_BLACK);
    tft.drawCircle(sat_pos_x+x, sat_pos_y+y, XgraphVal/3,    ST7735_WHITE);
  }

}

void tftShowValues(float batteryChargingVoltage,
                   float batteryChargingCurrent,
                   float batteryVoltage,
                   float solarCellAVoltage,
                   float solarCellBVoltage,
                   float solarCellCVoltage,
                   float batteryTemperature,
                   float boardTemperature,
                   int   mcuTemperature,
                   int   resetCounter,
                   float last_packet_received_rssi,
                   float last_packet_received_snr,
                   float last_packet_received_frequencyerror,
                   String last_packet_received_time) {
  tft.fillRect(0, 0, 96, 8, ST7735_BLACK);
  tft.fillRect(0, 8, 160, 128, ST7735_BLACK);
  tftDrawString( 0,  10, "Bat Voltage:  " + String(batteryVoltage) + "V",ST7735_WHITE);
  tftDrawString( 0,  20, "Charge Volt:  " + String(batteryChargingVoltage) + "V",ST7735_WHITE);
  tftDrawString( 0,  30, "Charge Cur:   " + String(batteryChargingCurrent) +"A",ST7735_WHITE);
  tftDrawString( 0,  40, "Batt Temp:    " + String(batteryTemperature) + "C" ,ST7735_WHITE);
  tftDrawString( 0,  50, "Solarpanel:", ST7735_WHITE);
  tftDrawString( 0,  60, "A: " + String(solarCellAVoltage) + "V B: " + String(solarCellBVoltage) + "V C: " + String(solarCellCVoltage) + "V",ST7735_WHITE);
  tftDrawString( 0,  70, "Board Temp:   " + String(boardTemperature) + "C" ,ST7735_WHITE);
  tftDrawString( 0,  80, "Reset count:  " + String(resetCounter)  ,ST7735_WHITE);
  tftDrawString( 0,  90, "Last Packet: "+last_packet_received_time ,ST7735_WHITE);
  tftDrawString( 0, 100, "RSSI:" + String(last_packet_received_rssi) + "dBm SNR:"+ String(last_packet_received_snr) +"dB",ST7735_WHITE );
  tftDrawString( 0, 110, "Freq error: " + String(last_packet_received_frequencyerror) +" Hz",ST7735_WHITE);
}

void tftStatus(float batteryChargingVoltage,
               float batteryChargingCurrent,
               float batteryVoltage,
               float solarCellAVoltage,
               float solarCellBVoltage,
               float solarCellCVoltage,
               float batteryTemperature,
               float boardTemperature,
               int   mcuTemperature,
               int   resetCounter,
               float last_packet_received_rssi,
               float last_packet_received_snr,
               float last_packet_received_frequencyerror,
               String last_packet_received_time,
               bool   wifi_connected,
               String ipaddress,
               bool   mqtt_connected,
               String station,
               int sat_pos_x,
               int sat_pos_y,
               bool statusReached) {

  // select frame every 5 seconds (only once)
  if ( lastSecondDone != lastSec ){
    if (lastSec%20 == 0)  { tftFossasatLogo(station); lastSecondDone = lastSec; }
    if (lastSec%20 == 5)  { tftWifiLogo(wifi_connected, ipaddress, mqtt_connected, statusReached);  lastSecondDone = lastSec;}
    if (lastSec%20 == 10) { tftDrawSatOnMap(sat_pos_x, sat_pos_y); lastSecondDone = lastSec;}
    if (lastSec%20 == 15) { tftShowValues(batteryChargingVoltage,
                                              batteryChargingCurrent,
                                              batteryVoltage,
                                              solarCellAVoltage,
                                              solarCellBVoltage,
                                              solarCellCVoltage,
                                              batteryTemperature,
                                              boardTemperature,
                                              mcuTemperature,
                                              resetCounter,
                                              last_packet_received_rssi,
                                              last_packet_received_snr,
                                              last_packet_received_frequencyerror,
                                              last_packet_received_time);  lastSecondDone = lastSec;}
  } // lastSecodnDone != lastSec
  tftShowTime(100,0);
}

void tftSetup(int ver, char * ssid){
  tft.initR(INITR_BLACKTAB); // Init ST7735S chip
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setRotation(1);
  tftDrawString(25, 0, "FossaSAT-1 Station",ST7735_WHITE);
  tftDrawString(25, 30, "version: ",ST7735_WHITE);
  char tmpbuf[9];
  sprintf(tmpbuf, "%d", ver);
  tftDrawString(75, 30, tmpbuf, ST7735_WHITE);
  tftDrawString(35, 50,"by @gmag12," ,ST7735_WHITE);
  tftDrawString(35, 60,"   @4m1g0," ,ST7735_WHITE);
  tftDrawString(35, 70,"   @g4lile0" ,ST7735_WHITE);
  /*
  for (int i=0; i<160; i+=20) {
    for (int j=0; j< 128; j++) {
      tft.drawPixel(i,j,ST7735_RED);
    }
  }
*/
  delay (4000);

  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);

  tftWifiLogo(false, "", false, false);
}
