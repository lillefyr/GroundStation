/*
The aim of this project is to create an open network of ground stations for the Fossa Satellites distributed all over the world and connected through Internet.
This project is based on ESP32 boards and is compatible with sx126x and sx127x you can build you own board using one of these modules but most of us use a development
board like the ones listed in the Supported boards section.
The developers of this project have no relation with the Fossa team in charge of the mission, we are passionate about space and created this project to be able to track
and use the satellites as well as supporting the mission.

Supported boards
    Heltec WiFi LoRa 32 V1 (433MHz SX1278)
    Heltec WiFi LoRa 32 V2 (433MHz SX1278)
    TTGO LoRa32 V1 (433MHz SX1278)
    TTGO LoRa32 V2 (433MHz SX1278)

Supported modules
    sx126x
    sx127x

    World Map with active Ground Stations and satellite stimated possition
    Main community chat: https://t.me/joinchat/DmYSElZahiJGwHX6jCzB3Q
      In order to onfigure your Ground Station please open a private chat to get your credentials https://t.me/fossa_updates_bot
    Data channel (station status and received packets): https://t.me/FOSSASAT_DATA
    Test channel (simulator packets received by test groundstations): https://t.me/FOSSASAT_TEST

    Developers:
              @gmag12       https://twitter.com/gmag12
              @dev_4m1g0    https://twitter.com/dev_4m1g0
              @g4lile0      https://twitter.com/G4lile0

    LICENSE     GNU General Public License v3.0


  */

// include the library
#include <RadioLib.h>
#include <SPI.h>
#include "Comms.h"
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "graphics.h"
#include <ArduinoJson.h>                                    //    https://github.com/bblanchon/ArduinoJson
#include <WiFi.h>
#include "time.h"
#include <sys/time.h>
#include <ESPmDNS.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "esp32_mqtt_client.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include <AsyncTCP.h>										// https://github.com/me-no-dev/AsyncTCP
#include <FS.h>
//#include <SPIFFS.h>
#include <DNSServer.h>

#include "lolind32.h"
#include "config.h"

void  welcome_message (void);

constexpr auto LOG_TAG = "FOSSAGS";

Esp32_mqtt_clientClass mqtt;

const int fs_version = 1912291;      // version year month day release
const char*  message[32];
bool mqtt_connected = false;

void manageMQTTEvent (esp_mqtt_event_id_t event) {
  if (event == MQTT_EVENT_CONNECTED) {
	  mqtt_connected = true;
    String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/data/#";
    mqtt.subscribe (topic.c_str());
    mqtt.subscribe ("fossa/global/sat_pos_oled");
	  welcome_message ();
    
  } else   if (event == MQTT_EVENT_DISCONNECTED) {
    mqtt_connected = false;
  }
}

uint8_t sat_pos_oled[2] = {0,0};
void manageSatPosOled(char* payload, size_t payload_len) {
  DynamicJsonDocument doc(60);
  char payloadStr[payload_len+1];
  memcpy(payloadStr, payload, payload_len);
  payloadStr[payload_len] = '\0';
  deserializeJson(doc, payload);
  sat_pos_oled[0] = doc[0];
  sat_pos_oled[1] = doc[1];
}

void manageMQTTData (char* topic, size_t topic_len, char* payload, size_t payload_len) {
  // Don't use Serial.print here. It will not work. Use ESP_LOG or printf instead.
  ESP_LOGI (LOG_TAG,"Received MQTT message: %.*s : %.*s\n", topic_len, topic, payload_len, payload);
  char topicStr[topic_len+1];
  memcpy(topicStr, topic, topic_len);
  topicStr[topic_len] = '\0';
  if (!strcmp(topicStr, "fossa/global/sat_pos_oled")) {
    manageSatPosOled(payload, payload_len);
  }
}

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0; // 3600;         // 3600 for Spain
const int   daylightOffset_sec = 0; // 3600;

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// SX1262 has the following connections:
// NSS pin:   5
// DIO1 pin:  27
// NRST pin:  14
// BUSY pin:  26
SX1262 lora = new Module(5, 27, 14, 26);  // nss, DIO1, rst, busy
int state;

// satellite callsign
char callsign[] = "FOSSASAT-1";

// Last packet received         "22:23:23"
String last_packet_received_time = " Waiting      ";
float last_packet_received_rssi;
float last_packet_received_snr;
float last_packet_received_frequencyerror;

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

//Initial dummy System info:
float batteryChargingVoltage = 0.0f;
float batteryChargingCurrent = 0.0f;
float batteryVoltage = 0.0f;
float solarCellAVoltage = 0.0f;
float solarCellBVoltage = 0.0f;
float solarCellCVoltage = 0.0f;
float batteryTemperature = 0.0f;
float boardTemperature = 0.0f;
int mcuTemperature = 0;
int resetCounter = 0;
byte powerConfig = 0b11111111;

void setup() {

  Serial.begin (115200);
  Serial.printf("Fossa Ground station Version to %d\n", fs_version);

  //connect to WiFi
  Serial.println("connect to WiFi");
  WiFi.begin(wifi_ssid, wifi_pass);
  delay(500);

  Serial.println();
  Serial.print("Waiting for WiFi... ");

  while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

#ifdef LOLIND32
  tftSetup(fs_version, wifi_ssid);
#endif

#define MQTTREQUIRED
#ifdef MQTTREQUIRED
  ESP_LOGI (LOG_TAG, "mqtt.init");
  mqtt.init (mqtt_server_name, mqtt_port, mqtt_user, mqtt_pass);

  String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/status";
 
  mqtt.setLastWill(topic.c_str());
  mqtt.onEvent (manageMQTTEvent);
  mqtt.onReceive (manageMQTTData);
  mqtt.begin();

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if (strcmp (tz, "")) {
	  setenv ("TZ", tz, 1);
	  ESP_LOGD (LOG_TAG, "Set timezone value as %s", tz);
	  tzset ();
  }

  printLocalTime();
#endif

  // initialize SX1278 with default settings
  Serial.print(F("[SX126x] Initializing ... "));
  ESP_LOGI (LOG_TAG, "lora.begin");
  

  int state = lora.begin(LORA_CARRIER_FREQUENCY,
                          LORA_BANDWIDTH,
                          LORA_SPREADING_FACTOR,
                          LORA_CODINGRATE,
                          SYNC_WORD_6X,
                          17,
                          (uint8_t)LORA_CURRENT_LIMIT,
                          LORA_PREAMBLE_LENGTH,
                          TCXO_VOLTAGE);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  // attach the ISR to radio interrupt/
  lora.setDio1Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX126x] Starting to listen ... "));

  state = lora.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // lora.standby()
  // lora.sleep()
  // lora.transmit();
  // lora.receive();
  // lora.readData();
  // lora.scanChannel();

  Serial.println (" Connected !!!");
  
  printControls();
}

void loop() {

/// fossa station code
  // check serial data
  if(Serial.available()) {
    // disable reception interrupt
    enableInterrupt = false;
   // detachInterrupt(digitalPinToInterrupt(DIO1));

    // get the first character
    char serialCmd = Serial.read();

    // wait for a bit to receive any trailing characters
    delay(50);

    // dump the serial buffer
    while(Serial.available()) {
      Serial.read();
    }

    // process serial command
    switch(serialCmd) {
      case 'p':
        sendPing();
        break;
      case 'i':
        requestInfo();
        break;
      case 'l':
        requestPacketInfo();
        break;
      case 'r':
        requestRetransmit();
        break;
/*      
      case 'e':
        config_manager.eraseConfig();
        ESP.restart();
        break;
      case 't':
        switchTestmode();
        ESP.restart();
        break;
*/
      case 'b':
        ESP.restart();
        break;
       
      default:
        Serial.print(F("Unknown command: "));
        Serial.println(serialCmd);
        break;
    }

    // set radio mode to reception
    lora.setDio1Action(setFlag);

    state = lora.startReceive();
    enableInterrupt = true;
  }
  
  //fossa station code

  // check if the flag is set (received interruption)
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // read received data
    size_t respLen = lora.getPacketLength();
    uint8_t* respFrame = new uint8_t[respLen];
    state = lora.readData(respFrame, respLen);

    // get function ID
    uint8_t functionId = FCP_Get_FunctionID(callsign, respFrame, respLen);
    
    uint8_t* respOptData;
    uint8_t respOptDataLen;

    if (( functionId >= 0x10 ) && ( functionId < 0x15 )){
      Serial.print(F("Function ID: 0x"));
      Serial.println(functionId, HEX);

      // check optional data
      respOptData = nullptr;
      respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen);
      Serial.print(F("Optional data ("));
      Serial.print(respOptDataLen);
      Serial.println(F(" bytes):"));
      if(respOptDataLen > 0) {
        // read optional data
        respOptData = new uint8_t[respOptDataLen];
        FCP_Get_OptData(callsign, respFrame, respLen, respOptData);
        PRINT_BUFF(respFrame, respLen);
        
      }
  
      struct tm timeinfo;
      if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return;
      }
  
      // store time of the last packet received:
      String thisTime="";
      if (timeinfo.tm_hour < 10){ thisTime=thisTime + " ";} // add leading space if required
      thisTime=String(timeinfo.tm_hour) + ":";
      if (timeinfo.tm_min < 10){ thisTime=thisTime + "0";} // add leading zero if required
      thisTime=thisTime + String(timeinfo.tm_min) + ":";
      if (timeinfo.tm_sec < 10){ thisTime=thisTime + "0";} // add leading zero if required
      thisTime=thisTime + String(timeinfo.tm_sec);
      // const char* newTime = (const char*) thisTime.c_str();
      
      last_packet_received_time= thisTime;
      last_packet_received_rssi= lora.getRSSI();
      last_packet_received_snr= lora.getSNR();
      last_packet_received_frequencyerror= 0.0; // lora.getFrequencyError();
  
      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX126x] RSSI:\t\t"));
      Serial.print(lora.getRSSI());
      Serial.println(F(" dBm"));
  
      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX126x] SNR:\t\t"));
      Serial.print(lora.getSNR());
      Serial.println(F(" dB"));

  
  /*
      // print frequency error
      Serial.print(F("[SX126x] Frequency error:\t"));
      Serial.print(lora.getFrequencyError());
      Serial.println(F(" Hz"));
  */
    }
      // process received frame
    switch(functionId) {
      case RESP_PONG:
        Serial.println(F("Pong!"));
        json_pong();
        break;

      case RESP_SYSTEM_INFO:
        
        Serial.println(F("System info:"));

        Serial.print(F("batteryChargingVoltage = "));
        batteryChargingVoltage = FCP_Get_Battery_Charging_Voltage(respOptData);
        Serial.println(FCP_Get_Battery_Charging_Voltage(respOptData));
        
        Serial.print(F("batteryChargingCurrent = "));
        batteryChargingCurrent = (FCP_Get_Battery_Charging_Current(respOptData), 4);
        Serial.println(FCP_Get_Battery_Charging_Current(respOptData), 4);

        Serial.print(F("batteryVoltage = "));
        batteryVoltage=FCP_Get_Battery_Voltage(respOptData);
        Serial.println(FCP_Get_Battery_Voltage(respOptData));          

        Serial.print(F("solarCellAVoltage = "));
        solarCellAVoltage= FCP_Get_Solar_Cell_Voltage(0, respOptData);
        Serial.println(FCP_Get_Solar_Cell_Voltage(0, respOptData));

        Serial.print(F("solarCellBVoltage = "));
        solarCellBVoltage= FCP_Get_Solar_Cell_Voltage(1, respOptData);
        Serial.println(FCP_Get_Solar_Cell_Voltage(1, respOptData));

        Serial.print(F("solarCellCVoltage = "));
        solarCellCVoltage= FCP_Get_Solar_Cell_Voltage(2, respOptData);
        Serial.println(FCP_Get_Solar_Cell_Voltage(2, respOptData));

        Serial.print(F("batteryTemperature = "));
        batteryTemperature=FCP_Get_Battery_Temperature(respOptData);
        Serial.println(FCP_Get_Battery_Temperature(respOptData));

        Serial.print(F("boardTemperature = "));
        boardTemperature=FCP_Get_Board_Temperature(respOptData);
        Serial.println(FCP_Get_Board_Temperature(respOptData));

        Serial.print(F("mcuTemperature = "));
        mcuTemperature =FCP_Get_MCU_Temperature(respOptData);
        Serial.println(FCP_Get_MCU_Temperature(respOptData));

        Serial.print(F("resetCounter = "));
        resetCounter=FCP_Get_Reset_Counter(respOptData);
        Serial.println(FCP_Get_Reset_Counter(respOptData));

        Serial.print(F("powerConfig = 0b"));
        powerConfig=FCP_Get_Power_Configuration(respOptData);
        Serial.println(FCP_Get_Power_Configuration(respOptData), BIN);
        json_system_info();
        break;

      case RESP_LAST_PACKET_INFO:
        Serial.println(F("Last packet info:"));

        Serial.print(F("SNR = "));
        Serial.print(respOptData[0] / 4.0);
        Serial.println(F(" dB"));

        Serial.print(F("RSSI = "));
        Serial.print(respOptData[1] / -2.0);
        Serial.println(F(" dBm"));
        break;

      case RESP_REPEATED_MESSAGE:
        Serial.println(F("Got repeated message:"));
        Serial.println((char*)respOptData);
        json_message((char*)respOptData,respLen);
        break;

      default:
//        Serial.println(F("Unknown function ID!"));
        break;
    }

    if (state == ERR_NONE) {
      // packet was successfully received
  //     Serial.println(F("[SX126x] Received packet!"));

      // print data of the packet
 //     Serial.print(F("[SX126x] Data:\t\t"));
 //      Serial.println(str);
    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX126x] CRC error!"));
    } else {
      // some other error occurred
      Serial.print(F("[SX126x] Failed, code "));
      Serial.println(state);
    }

    // put module back to listen mode
    state = lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }
  
  unsigned long last_connection_fail = millis();
  if (!mqtt_connected){
    if (millis() - last_connection_fail > 300000){ // 5m
      Serial.println("MQTT Disconnected, restarting...");
      ESP.restart();
    }
  } else {
    last_connection_fail = millis();
  }

#ifdef LOLIND32
  // pass everything as parameters and handle in  tftStatus
  tftStatus(batteryChargingVoltage,
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
            last_packet_received_time,
            true,
            WiFi.localIP().toString(),
            mqtt_connected,
            station,
            sat_pos_oled[0], // extraced in manageSatPosOled
            sat_pos_oled[1],
            true);
#endif
}


void  welcome_message (void) {
        const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(16);
          DynamicJsonDocument doc(capacity);
          doc["station"] = station;
          JsonArray station_location = doc.createNestedArray("station_location");
          station_location.add(latitude);
          station_location.add(longitude);
          doc["version"] = fs_version;

//          doc["time"] = ;
          serializeJson(doc, Serial);
          String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/welcome";
          char buffer[512];
          size_t n = serializeJson(doc, buffer);
          mqtt.publish(topic.c_str(), buffer,n );
          ESP_LOGI (LOG_TAG, "Wellcome sent");
}

void  json_system_info(void) {
          //// JSON
          
          time_t now;
          time(&now);
          const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(18);
          DynamicJsonDocument doc(capacity);
          doc["station"] = station;  // G4lile0
          JsonArray station_location = doc.createNestedArray("station_location");
          station_location.add(latitude);
          station_location.add(longitude);
          doc["rssi"] = last_packet_received_rssi;
          doc["snr"] = last_packet_received_snr;
          doc["frequency_error"] = last_packet_received_frequencyerror;
          doc["unix_GS_time"] = now;
          doc["batteryChargingVoltage"] = batteryChargingVoltage;
          doc["batteryChargingCurrent"] = batteryChargingCurrent;
          doc["batteryVoltage"] = batteryVoltage;
          doc["solarCellAVoltage"] = solarCellAVoltage;
          doc["solarCellBVoltage"] = solarCellBVoltage;
          doc["solarCellCVoltage"] = solarCellCVoltage;
          doc["batteryTemperature"] = batteryTemperature;
          doc["boardTemperature"] = boardTemperature;
          doc["mcuTemperature"] = mcuTemperature;
          doc["resetCounter"] = resetCounter;
          doc["powerConfig"] = powerConfig;
          serializeJson(doc, Serial);
          String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/sys_info";
          char buffer[512];
          serializeJson(doc, buffer);
          size_t n = serializeJson(doc, buffer);
          mqtt.publish(topic.c_str(), buffer,n );
}


void  json_message(char* frame, size_t respLen) {
          time_t now;
          time(&now);
          Serial.println(String(respLen));
          char tmp[respLen+1];
          memcpy(tmp, frame, respLen);
          tmp[respLen-12] = '\0';


              // if special miniTTN message   
          Serial.println(String(frame[0]));
          Serial.println(String(frame[1]));
          Serial.println(String(frame[2]));
//          if ((frame[0]=='0x54') &&  (frame[1]=='0x30') && (frame[2]=='0x40'))
          if ((frame[0]=='T') &&  (frame[1]=='0') && (frame[2]=='@'))
          {
          Serial.println("mensaje miniTTN");
          const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(11) +JSON_ARRAY_SIZE(respLen-12);
          DynamicJsonDocument doc(capacity);
          doc["station"] = station;  // G4lile0
          JsonArray station_location = doc.createNestedArray("station_location");
          station_location.add(latitude);
          station_location.add(longitude);
          doc["rssi"] = last_packet_received_rssi;
          doc["snr"] = last_packet_received_snr;
          doc["frequency_error"] = last_packet_received_frequencyerror;
          doc["unix_GS_time"] = now;
          JsonArray msgTTN = doc.createNestedArray("msgTTN");

          
          for (byte i=0 ; i<  (respLen-12);i++) {

                msgTTN.add(String(tmp[i], HEX));

            }
          

          
//          doc["len"] = respLen;
//          doc["msg"] = String(tmp);
//          doc["msg"] = String(frame);
 
          
          serializeJson(doc, Serial);
          String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/miniTTN";
        
          char buffer[256];
          serializeJson(doc, buffer);
          size_t n = serializeJson(doc, buffer);
          mqtt.publish(topic.c_str(), buffer,n );

            
            }

            else

            {
              
          
          const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(11);
          DynamicJsonDocument doc(capacity);
          doc["station"] = station;  // G4lile0
          JsonArray station_location = doc.createNestedArray("station_location");
          station_location.add(latitude);
          station_location.add(longitude);
          doc["rssi"] = last_packet_received_rssi;
          doc["snr"] = last_packet_received_snr;
          doc["frequency_error"] = last_packet_received_frequencyerror;
          doc["unix_GS_time"] = now;
//          doc["len"] = respLen;
          doc["msg"] = String(tmp);
//          doc["msg"] = String(frame);
 
          
          serializeJson(doc, Serial);
          String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/msg";
          
          char buffer[256];
          serializeJson(doc, buffer);
          size_t n = serializeJson(doc, buffer);
          mqtt.publish(topic.c_str(), buffer,n );
              
              }
            
          


          
          
          
         
}



void  json_pong(void) {
          //// JSON
          time_t now;
          time(&now);
          const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(6);
          DynamicJsonDocument doc(capacity);
          doc["station"] = station;  // G4lile0
          JsonArray station_location = doc.createNestedArray("station_location");
          station_location.add(latitude);
          station_location.add(longitude);
          doc["rssi"] = last_packet_received_rssi;
          doc["snr"] = last_packet_received_snr;
          doc["frequency_error"] = last_packet_received_frequencyerror;
          doc["unix_GS_time"] = now;
          doc["pong"] = 1;
          serializeJson(doc, Serial);
          String topic = "fossa/" + String(mqtt_user) + "/" + String(station) + "/pong";
          char buffer[256];
          serializeJson(doc, buffer);
          size_t n = serializeJson(doc, buffer);
          mqtt.publish(topic.c_str(), buffer,n );
}


// function to print controls
void printControls() {
  Serial.println(F("------------- Controls -------------"));
  Serial.println(F("p - send ping frame"));
  Serial.println(F("i - request satellite info"));
  Serial.println(F("l - request last packet info"));
  Serial.println(F("r - send message to be retransmitted"));
  Serial.println(F("t - change the test mode and restart"));
  Serial.println(F("e - erase board config and reset"));
  Serial.println(F("b - reboot the board"));
  Serial.println(F("------------------------------------"));
}



void sendPing() {
  Serial.print(F("Sending ping frame ... "));

  // data to transmit
  uint8_t functionId = CMD_PING;

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);

  // send data
  int state = lora.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}


void  switchTestmode() {
  char temp_station[32];
  if ((station[0]=='t') &&  (station[1]=='e') && (station[2]=='s') && (station[4]=='_')) {
    Serial.println(F("Changed from test mode to normal mode"));
    for (byte a=5; a<=strlen(station); a++ ) {
      station[a-5]=station[a];
    }
  }
  else
  {
    strcpy(temp_station,"test_");
    strcat(temp_station,station);
    strcpy(station,temp_station);
    Serial.println(F("Changed from normal mode to test mode"));
  }

//  config_manager.saveFlashData();
}



void requestInfo() {
  Serial.print(F("Requesting system info ... "));

  // data to transmit
  uint8_t functionId = CMD_TRANSMIT_SYSTEM_INFO;

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);

  // send data
  int state = lora.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void requestPacketInfo() {
  Serial.print(F("Requesting last packet info ... "));

  // data to transmit
  uint8_t functionId = CMD_GET_LAST_PACKET_INFO;

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId);

  // send data
  int state = lora.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

void requestRetransmit() {
  Serial.println(F("Enter message to be sent:"));
  Serial.println(F("(max 32 characters, end with LF or CR+LF)"));

  // get data to be retransmited
  char optData[32];
  uint8_t bufferPos = 0;
  while(bufferPos < 32) {
    while(!Serial.available());
    char c = Serial.read();
    Serial.print(c);
    if((c != '\r') && (c != '\n')) {
      optData[bufferPos] = c;
      bufferPos++;
    } else {
      break;
    }
  }

  // wait for a bit to receive any trailing characters
  delay(100);

  // dump the serial buffer
  while(Serial.available()) {
    Serial.read();
  }

  Serial.println();
  Serial.print(F("Requesting retransmission ... "));

  // data to transmit
  uint8_t functionId = CMD_RETRANSMIT;
  optData[bufferPos] = '\0';
  uint8_t optDataLen = strlen(optData);

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, (uint8_t*)optData);

  // send data
  int state = lora.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}
