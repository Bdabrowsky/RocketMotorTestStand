#include "FileSys.h"


bool FS_init(){

    if(!LittleFS.begin(false)){
       
        return false;
    }
    appendFile(LittleFS, "/log.csv", "\nThis is beginning of new log file. This happens after every reboot\n");
    appendFile(LittleFS, "/log.csv", "Timestamp [ms], Force [N]\n");

    return true;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        log_i("- file written");
    } else {
        log_e("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
     log_w("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        log_e("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
       log_i("- message appended");
    } else {
        log_e("- append failed");
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char * path){
        log_w("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        log_i("- file deleted");
    } else {
        log_e("- delete failed");
    }
}

void closeFile(fs::FS &fs, const char * path){
    File file = fs.open(path, FILE_APPEND);
    file.close();
}
