#pragma once

#include <Arduino.h>
#include <string>
#include <iohcRadio.h>


#define IOHC_1W_REMOTE  "/1W.json"


/*
    Singleton class with a full implementation of a VELUX KLIxxx controller
    The type of the controller can be managed changing related value within its profile file (1W.json)
    Type can be multiple, as it would be for KLI310, KLI312 and KLI313
    Also, the address and private key can be configured within the same json file.
*/
namespace IOHC
{
    enum class RemoteButton {
        Pair,
        Add,
        Remove,
        Open,
        Close,
        Stop,
        Vent,
        ForceOpen
    };

    class iohcRemote1W
    {
        public:
            static iohcRemote1W *getInstance();
            virtual ~iohcRemote1W() {};

            void cmd(RemoteButton cmd);

        private:
            iohcRemote1W();
            static iohcRemote1W *_iohcRemote1W;
            bool load(void);
            bool save(void);

        protected:
            address _node;
            uint16_t _sequence;
            uint8_t _key[16];
            std::vector<uint16_t> _type;
            uint8_t _manufacturer;

            IOHC::iohcPacket *packets2send[25];
            IOHC::iohcRadio *_radioInstance;
    };
}