#ifndef IOHC_OBJECT_H
#define IOHC_OBJECT_H

#include <string>
#include <iohcPacket.h>
#include <iohcCryptoHelpers.h>

#define MAX_MANUFACTURER    13

namespace IOHC {

    struct iohcObject_t {
        address     node;
        uint8_t     actuator[2];
        uint8_t     flags;
        uint8_t     io_manufacturer;
        address     backbone;
    };

    class iohcObject {
        public:
            iohcObject();
            ~iohcObject();
            iohcObject(const address node, const address backbone, const uint8_t actuator[2], uint8_t manufacturer, uint8_t flags);
            explicit iohcObject(std::string serialized);

            address *getNode();
            address *getBackbone();
            std::tuple<uint16_t, uint8_t> getTypeSub();
            std::string serialize();
            void dump1W();
            void dump2W();
        protected:

        private:
            iohcObject_t object{};
            char man_id[MAX_MANUFACTURER][15] = {"VELUX", "Somfy", "Honeywell", "Hörmann", "ASSA ABLOY", "Niko", "WINDOW MASTER", "Renson", "CIAT", "Secuyou", "OVERKIZ", "Atlantic Group", "Other"};
    };
}
#endif