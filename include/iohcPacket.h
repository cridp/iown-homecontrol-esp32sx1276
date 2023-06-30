#pragma once

#include <Arduino.h>
#include <SX1276Helpers.h>

#define RESET_AFTER_LAST_MSG_US 15000

namespace Radio
{
    class iohcPacket
    {
        public:
            iohcPacket(void)
            {
            }


            ~iohcPacket()
            {
                ; // destroyer ... stop rx and put radio in standby
            }

            Radio::payload packet;
            uint8_t buffer_length = 0;
            uint32_t frequency;
            unsigned long millis = 0;
            float rssi = 0;

        protected:

        private:
    };
}