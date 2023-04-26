#include <Arduino.h>
#include <SPI.h>
#include "AD7190.h"
#include "ESPAsyncWebServer.h"
#include "FS.h"
#include "SPIFFS.h"
#include "Utils.h"
#include "FileSys.h"

bool start = false;
uint32_t timestamp = 0;

const char *ssid = "Hamownia";
//const char *password = "KPPTR";

AsyncWebServer server(80);


void setup() {
  //Server stuff

  FS_init();


  WiFi.softAP(ssid);

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
    request->send(200, "text/plain", "Succesfully deleted file!");
  });
  
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    start = true;
    request->send(SPIFFS, "/index.html", "text/html", false);
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    start = false;
    request->send(SPIFFS, "/index.html", "text/html", false);
  });

  server.begin();


  Serial.begin(1000000);
  if(AD7190_Init()){
      Serial.println("AD7190 Initialized");
  }
  else{
      Serial.println("AD7190 Error");
  }

  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M);

  AD7190_RangeSetup(0, AD7190_CONF_GAIN_128);


  float temporaryCal = UtilsSrv(AD7190_ContinuousReadAvg(1));
  UtilsInit(2137.0, 0.00197, temporaryCal);
  

}

void loop() {
  float dat = UtilsSrv(AD7190_ContinuousSingleRead());

  if(start){
    char temp[50];
   
    sprintf(temp, "%d,%f", timestamp, dat);
    appendFile(SPIFFS, "/log.csv", temp);
  }

  Serial.print(timestamp);
  Serial.print(" ");
  Serial.println(AD7190_ContinuousSingleRead());

}