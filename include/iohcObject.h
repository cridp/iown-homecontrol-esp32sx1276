#ifndef IOHC_OBJECT_H
#define IOHC_OBJECT_H

#include <string>
#include <iohcPacket.h>
#include <iohcCryptoHelpers.h>

#define MAX_MANUFACTURER    13

namespace IOHC {
//    typedef uint8_t address[3];
    typedef struct {
        address     node;
        uint8_t     actuator[2];
        uint8_t     flags;
        uint8_t     io_manufacturer;
        address     backbone;
    } iohcObject_t;
    typedef struct {
        address     node;
//        uint8_t     actuator[2];
//        uint8_t     flags;
//        uint8_t     io_manufacturer;
        address     backbone;
    } iohc2WObject_t;


    class iohcObject {
        public:
            iohcObject();
//            iohcObject(IOHC::iohcPacket *iohc);
            iohcObject(const address node, const address backbone, const uint8_t actuator[2], uint8_t manufacturer, uint8_t flags);
            explicit iohcObject(std::string serialized);
//            iohc2WObject(std::string serialized);
            ~iohcObject();

            address *getNode();
            address *getBackbone();
            std::tuple<uint16_t, uint8_t> getTypeSub();
            std::string serialize();
            void dump1W();
            void dump2W();
        protected:

        private:
            iohcObject_t object{};
 //           std::vector<uint8_t> *_buffer;
            char man_id[MAX_MANUFACTURER][15] = {"VELUX", "Somfy", "Honeywell", "Hörmann", "ASSA ABLOY", "Niko", "WINDOW MASTER", "Renson", "CIAT", "Secuyou", "OVERKIZ", "Atlantic Group", "Other"};
    };
}
#endif