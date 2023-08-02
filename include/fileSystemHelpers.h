#pragma once

#include <Arduino.h>
#include <string>
#include <LittleFS.h>


void listFS(void);
void cat(const char *fname);
void rm(const char *fname);