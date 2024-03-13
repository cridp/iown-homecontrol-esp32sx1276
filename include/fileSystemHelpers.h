// #pragma once
#ifndef FILESYSTEMHELPERS_H
#define FILESYSTEMHELPERS_H

//#include <Arduino.h>
#include <string>
#include <LittleFS.h>

#if defined(ESP32)
    #include <FS.h>
#endif

void listFS();
void cat(const char *fname);
void rm(const char *fname);
#endif // FILESYSTEMHELPERS_H
