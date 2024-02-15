#ifndef IOHC_1W_DEVICE_H
#define IOHC_1W_DEVICE_H

#include <vector> 
#include <iohcDevice.h>

#define IOHC_1W_REMOTE  "/1W.json"

/*
    Singleton class with a full implementation of a VELUX KLIxxx controller
    The type of the controller can be managed changing related value within its profile file (1W.json)
    Type can be multiple, as it would be for KLI310, KLI312 and KLI313
    Also, the address and private key can be configured within the same json file.
*/
namespace IOHC {
    enum class RemoteButton {
        Pair,
        Add,
        Remove,
        Open,
        Close,
        Stop,
        Vent,
        ForceOpen,
        testKey, Mode1, Mode2
    };

    class iohcRemote1W : public iohcDevice {
        public:
            static iohcRemote1W *getInstance();
            ~iohcRemote1W() override = default;

            void cmd(RemoteButton cmd);
            bool load() override;
            bool save() override;
            void scanDump() override{}

            static void init(iohcPacket *packet, uint16_t/*size_t*/ typn);

        private:
            iohcRemote1W();
            static iohcRemote1W *_iohcRemote1W;

        protected:
            // address _node{};
            // uint16_t _sequence{};
            // uint8_t _key[16]{};
            // uint16_t _type{};
            // uint8_t _manufacturer{};
        struct remote {
            address node{};
            uint16_t sequence{};
            uint8_t key[16]{};
            std::vector<uint8_t> type{};
            uint8_t manufacturer{};
        };
        std::vector<remote> remotes;

//            IOHC::iohcPacket *packets2send[25]{};
            // std::array<iohcPacket*, 25> packets2send{};
            std::vector<iohcPacket*> packets2send{};
//            IOHC::iohcRadio *_radioInstance;
    };
}
#endif