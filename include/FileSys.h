#pragma once

#include "FS.h"
#include "SPIFFS.h"
#include <LittleFS.h>

bool FS_init();

void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char * path);
void closeFile(fs::FS &fs, const char * path);
