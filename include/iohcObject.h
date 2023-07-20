#pragma once

#include <Arduino.h>
#include <iohcPacket.h>


#define MAX_MANUFACTURER    13


namespace IOHC
{
    typedef uint8_t address[3];
    typedef struct 
    {
        address     node;
        uint8_t     actuator[2];
        uint8_t     flags;
        uint8_t     io_manufacturer;
        address     backbone;
    } iohcObject_t;


    class iohcObject
    {
        public:
            iohcObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags);
            ~iohcObject();

            address *getNode(void);
            address *getBackbone(void);
            std::tuple<uint16_t, uint8_t> getTypeSub(void);
            void dump(void);
        protected:

        private:
            iohcObject_t iohcDevice;
            char man_id[MAX_MANUFACTURER][15] = {"VELUX", "Somfy", "Honeywell", "Hörmann", "ASSA ABLOY", "Niko", "WINDOW MASTER", "Renson", "CIAT", "Secuyou", "OVERKIZ", "Atlantic Group", "Other"};
    };

}