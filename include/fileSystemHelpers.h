#pragma once

#include <Arduino.h>
#include <string>
#include <LittleFS.h>

#if defined(HELTEC)
    #include <FS.h>
#endif


void listFS();
void cat(const char *fname);
void rm(const char *fname);