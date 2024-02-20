#ifndef OTHER_2W_DEVICE_H
#define OTHER_2W_DEVICE_H

#include <string>
#include <iohcRadio.h>
#include <vector> 
#include <iohcDevice.h>

#define OTHER_2W_FILE  "/Other2W.json"

/*
    Singleton class with a full implementation of a COZYTOUCH/KIZBOX/CONEXOON controller
    The type of the controller can be managed changing related value within its profile file (Cozy2W.json)
    Type can be multiple, as it would be for KLI310, KLI312 and KLI313
    Also, the address and private key can be configured within the same json file.
*/
namespace IOHC {
    enum class Other2WButton {
        discovery,
        // powerOn,
        // setTemp,
        // setMode,
        // setPresence,
        // midnight,
        // custom,
        // discover28,
        // discover2A_1,
        // discover2A_2,
//        Pair,
//        Add,
//        Remove,
//        Open,
//        Close,
//        Stop,
//        Vent,
//        ForceOpen
    };

    class iohcOtherDevice2W : public iohcDevice {

        public:
            static iohcOtherDevice2W *getInstance();
            ~iohcOtherDevice2W() override = default;

            Memorize memorizeOther2W; //2W only
            
            // Put that in json
//            uint8_t gateway[3] = {0xba, 0x11, 0xad};
//            uint8_t master_from[3] = {0x47, 0x77, 0x06}; // It's the new heater kitchen Address From 
//            uint8_t master_to[3] = {0x48, 0x79, 0x02}; // It's the new heater kitchen Address To 
//            uint8_t slave_from[3] = {0x8C, 0xCB, 0x30}; // It's the new heater kitchen Address From 
//            uint8_t slave_to[3] = {0x8C, 0xCB, 0x31}; // It's the new heater kitchen Address To 

//            bool isFake(address nodeSrc, address nodeDst) override;
            void cmd(Other2WButton cmd/*, const char* data*/);
            bool load() override;
            bool save() override;
            void scanDump() override{}

            static void init(iohcPacket *packet, size_t typn);

        private:
            iohcOtherDevice2W();
            static iohcOtherDevice2W *_iohcOtherDevice2W;

        protected:
//            unsigned long relStamp;
//            uint8_t source_originator[3] = {0};
            address _node{};
            address _dst{};
            std::string _type;
//            uint16_t _sequence;
//            uint8_t _key[16];
//            std::vector<uint16_t> _type;
//            uint8_t _manufacturer;

//            IOHC::iohcPacket *packets2send[2]; //[25];
            // std::array<iohcPacket*, 25> packets2send{};
        std::vector<iohcPacket*> packets2send{};
//            IOHC::iohcRadio *_radioInstance;
    };
}
#endif 