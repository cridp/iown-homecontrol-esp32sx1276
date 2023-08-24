/*
  General utility functions
*/

#pragma once
#include <vector> 
#include <sstream> 


#if defined(ESP8266)
  #include <TickerUs.h>
#elif defined(ESP32)  	
  #include <TickerUsESP32.h>
#endif

//#include <Ticker.h>


#define MAXCMDS 25

#if defined(DEBUG)
  #ifndef DEBUG_PORT
    #define DEBUG_PORT Serial
  #endif
  #define LOG(func, ...) DEBUG_PORT.func(__VA_ARGS__)
#else
  #define LOG(func, ...) ;
#endif


using Tokens = std::vector<std::string>;
namespace Cmd
{
  char _rxbuffer[256];
  uint8_t __len = 0;
  uint8_t __avail = 0;
//  bool receivingSerial = false;

#if defined(ESP8266)
      Timers::TickerUs kbd_tick;
#elif defined(ESP32)
      //Timers::TickerUsESP32 kbd_tick;
      TickerUsESP32 kbd_tick;
      //Ticker kbd_tick2;
#endif


  struct _cmdEntry{

      char cmd[15];
      char description[61];
      void (*handler)(Tokens*);
  } *_cmdHandler[MAXCMDS];
  uint8_t lastEntry = 0;

  void tokenize(std::string const &str, const char delim, Tokens &out) 
  { 
      // construct a stream from the string 
      std::stringstream ss(str); 

      std::string s; 
      while (std::getline(ss, s, delim)) { 
          out.push_back(s); 
      } 
  }

  bool addHandler(char *cmd, char *description, void (*handler)(Tokens*))
  {
    void *alloc = nullptr;

    for (uint8_t idx=0; idx<MAXCMDS; ++idx)
    {
      if (_cmdHandler[idx] != nullptr)
        ; // Skip already allocated cmd handler
      else
      {
        alloc = malloc(sizeof(struct _cmdEntry));
        if (!alloc)
          return false;

        _cmdHandler[idx] = (struct _cmdEntry *)alloc;
        memset(alloc, 0, sizeof(struct _cmdEntry));
        strncpy(_cmdHandler[idx]->cmd, cmd, strlen(cmd)<sizeof(_cmdHandler[idx]->cmd)?strlen(cmd):sizeof(_cmdHandler[idx]->cmd) - 1);
        strncpy(_cmdHandler[idx]->description, description, strlen(cmd)<sizeof(_cmdHandler[idx]->description)?strlen(description):sizeof(_cmdHandler[idx]->description) - 1);
        _cmdHandler[idx]->handler = handler;
        if (idx > lastEntry)
          lastEntry = idx;
        return true;
      } 
    }
    return false;
  }

  char *cmdReceived(bool echo = false)
  {
      __avail = Serial.available();
      if (__avail) 
      {
        __len += Serial.readBytes(&_rxbuffer[__len], __avail);
  //      receivingSerial = true;
        if (echo)
        {
          _rxbuffer[__len] = '\0';
          Serial.printf("%s", &_rxbuffer[__len - __avail]);
        }
      }
      if (_rxbuffer[__len-1] == 0x0a)
      {
          _rxbuffer[__len-2] = '\0';
          __len = 0;
  //        receivingSerial = false;
          return _rxbuffer;
      }
      return nullptr;
  }

  void cmdFuncHandler(){
    char *_cmd;
    const char delim = ' '; 
    Tokens segments; 

    _cmd = cmdReceived(true);
    if (!_cmd) return;
    if (!strlen(_cmd)) return;

    tokenize(_cmd, delim, segments);
    if (strcmp((char *)"help", segments[0].c_str()) == 0)
    {
      Serial.printf("\nRegistered commands:\n");
      for (uint8_t idx=0; idx<=lastEntry; ++idx)
      {
        if (_cmdHandler[idx] == nullptr) continue;
        Serial.printf("- %s\t%s\n", _cmdHandler[idx]->cmd, _cmdHandler[idx]->description);
      }
      Serial.printf("- %s\t%s\n\n", (char*)"help", (char*)"This command");
      Serial.printf("\n");
      return;
    }
    for (uint8_t idx=0; idx<=lastEntry; ++idx)
    {
      if (_cmdHandler[idx] == nullptr) continue;
      if (strcmp(_cmdHandler[idx]->cmd, segments[0].c_str()) == 0)
      {
        _cmdHandler[idx]->handler(&segments);
        return;
      }
    }
    Serial.printf("*> Unknown <*\n");
  }

  void init(void)
  {
    kbd_tick.attach_ms(100, cmdFuncHandler);
  }  
}