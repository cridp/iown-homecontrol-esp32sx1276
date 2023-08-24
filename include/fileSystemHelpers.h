#pragma once

#include <Arduino.h>
#include <string>
#include <LittleFS.h>

#if defined(ESP32)
    #include <FS.h>
#endif


void listFS(void);
void cat(const char *fname);
void rm(const char *fname);