#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include <SPI.h>
#include "AD7190.h"
#include "ESPAsyncWebServer.h"
#include "FS.h"
#include "SPIFFS.h"
#include <SPIFFS.h>
#include "Utils.h"
#include "FileSys.h"

#define DEBUG

bool start = false;
uint32_t timestamp = 0;
uint32_t prevTimestamp = 0;

const char *ssid = "Hamownia";
//const char *password = "KPPTR";

AsyncWebServer server(80);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, 48, NEO_GRB + NEO_KHZ800);

int sampleCnt = 4000;

char str[200000];

int cnt = 0;

const int freq = 200;

void setup() {
  //Server stuff
  FS_init();
  Serial.begin(115200);
 
  pixels.begin();
  
  if(!WiFi.softAP(ssid)){
    log_e("Failed to initialize wifi");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html", false);
  });
  
  server.serveStatic("/download", SPIFFS, "/log.csv");
 
  server.on("/parse", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/log.csv", "text/plain", false);
  });

  server.on("/help", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "KPPTR telemetry receiver:");
  });

  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request){
    deleteFile(SPIFFS, "/log.csv");

    for(int i=0;i<5;i++){
      pixels.setPixelColor(0, pixels.Color(10,00,0)); //Set LED to RED
      delay(50);
      pixels.setPixelColor(0, pixels.Color(0,0,0)); //Set LED to OFF
      delay(50);
    }
    request->send(200, "text/plain", "Succesfully deleted file!");
  });
  
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    start = true;
    pixels.setPixelColor(0, pixels.Color(0,0, 10)); //Set LED to BLUE
    request->send(SPIFFS, "/index.html", "text/html", false);
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    start = false;
    closeFile(SPIFFS, "/log.csv");
    pixels.setPixelColor(0, pixels.Color(0,10, 0)); //Set LED to GREEN
    request->send(SPIFFS, "/index.html", "text/html", false);
  });

  server.begin();
  pixels.setPixelColor(0, pixels.Color(0,10,0)); //Set LED to GREEN
  pixels.show();
  
  if(AD7190_Init()){
    log_i("AD7190 Initialized");
  }
  else{
    log_e("AD7190 Error");
    pixels.setPixelColor(0, pixels.Color(10,0,0)); //Set LED to RED
  }
  
  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M);

  AD7190_RangeSetup(0, AD7190_CONF_GAIN_128);

  UtilsInit(500.0, 0.00197, 0, 5.0, AD7190_CONF_GAIN_128);
  float temporaryCal = UtilsSrv(AD7190_ContinuousReadAvg(100));
  //UtilsInit(1000.0, 0.00197/2.0, temporaryCal, 5.0, AD7190_CONF_GAIN_128);
}

void loop() {
  
  timestamp = micros();
  
  if((timestamp - prevTimestamp) >= 1000000/freq){
    Serial.println(timestamp - prevTimestamp);
    prevTimestamp = timestamp;

    float dat = UtilsSrv(AD7190_ContinuousReadAvg(5));
    if(start){
      char temp[50];
    
      sprintf(temp, "%d,%f\n", timestamp, dat);
      strcat(str, temp);
      cnt++;
    }

    if(cnt == 10000 || (!start && cnt > 0)){
      appendFile(SPIFFS, "/log.csv", str);
      sprintf(str, "");
      cnt = 0;
    }

    //Serial.println(UtilsSrv(AD7190_ContinuousSingleRead()));
  }
  //Serial.print(timestamp);
  //Serial.print(" ");
  
  pixels.show();

} 