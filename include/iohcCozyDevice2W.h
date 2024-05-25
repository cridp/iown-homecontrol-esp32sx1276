#ifndef COZY_2W_DEVICE_H
#define COZY_2W_DEVICE_H

#include <interact.h>
#include <string>
#include <iohcRadio.h>
#include <iohcDevice.h>
#include <vector>

#define COZY_2W_FILE  "/Cozy2W.json"

/*
    Singleton class with a full implementation of a COZYTOUCH/KIZBOX/CONEXOON controller
    The type of the controller can be managed changing related value within its profile file (Cozy2W.json)
    Type can be multiple, as it would be for KLI310, KLI312 and KLI313
    Also, the address and private key can be configured within the same json file.
*/
namespace IOHC {
/// The `enum class DeviceButton` is defining an enumeration type with different button commands that can be used for a specific device. 
/// Each of the identifiers within the enumeration represents a specific button command that can be sent to the device. 
/// This enumeration provides a convenient way to refer to these commands using descriptive names rather than raw integer values.
    enum class DeviceButton {
        powerOn,
        setTemp,
        setMode,
        setPresence,
        setWindow,
        midnight,
        associate,
        custom,
        custom60,
        discover28,
        discover2A,
        fake0,
        ack,
        pairMode,
        checkCmd,
    };

    class iohcCozyDevice2W : public iohcDevice {
    public:
        static iohcCozyDevice2W* getInstance();

        ~iohcCozyDevice2W() override = default;

        Memorize memorizeSend; //2W only

        // Put that in json
        uint8_t gateway[3] = {0xba, 0x11, 0xad};
        uint8_t master_from[3] = {0x47, 0x77, 0x06}; // It's the new heater kitchen Address From
        uint8_t master_to[3] = {0x48, 0x79, 0x02}; // It's the new heater kitchen Address To
        uint8_t slave_from[3] = {0x8C, 0xCB, 0x30}; // It's the new heater kitchen Address From
        uint8_t slave_to[3] = {0x8C, 0xCB, 0x31}; // It's the new heater kitchen Address To


        typedef std::array<uint8_t, 3> Address;
        std::vector<Address> addresses = {
            Address{0x48, 0x79, 0x02}, //Master_to
            Address{0x8C, 0xCB, 0x31}, //Slave_to
            //        Address{0x47, 0x77, 0x06},
            // Address{0x8C, 0xCB, 0x30},
            // Address{0xba, 0x11, 0xad},
        };

        bool verbosity = true;

        bool isFake(address nodeSrc, address nodeDst) override;
        void cmd(DeviceButton cmd, Tokens* data /*const char* data*/);
        bool load() override;
        bool save() override;
        static void init(iohcPacket* packet);
        void initializeValid();
        // std::vector<uint8_t> valid; //(256);
        std::/*unordered_*/map<uint8_t, int> mapValid;
        // size_t validKey{};
        void scanDump() override;

    private:
        iohcCozyDevice2W();
        static iohcCozyDevice2W* _iohcCozyDevice2W;

    protected:
        //            unsigned long relStamp;
        //            uint8_t source_originator[3] = {0};
        address _node{};
        address _dst{};
        std::string _type;
        // uint16_t _sequence;
        // uint8_t _key[16];
        // std::vector<uint16_t> _type;
        // uint8_t _manufacturer;

        //            IOHC::iohcPacket *packets2send[2]; //[25];
//        std::array<iohcPacket *, 25> packets2send{};
        std::vector<iohcPacket*> packets2send{};

        //            IOHC::iohcRadio *_radioInstance;

    };
}
#endif
