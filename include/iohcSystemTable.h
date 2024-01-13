#ifndef IOHC_SYSTEMTABLE_H
#define IOHC_SYSTEMTABLE_H

#include <map>
#include <string>
#include <iohcObject.h>

#define IOHC_SYS_TABLE  "/sysTable.json"

/*
    Singleton class to implement the System Object Table.
    System Object Table tracks all managed devices with their base info

    At this time this is only a container.
*/
namespace IOHC {
    class iohcSystemTable {
        public:
            using Objects = std::map<std::string, iohcObject *>;
//            using Objects2W = std::map<std::string, iohc2WObject *>;

            static iohcSystemTable *getInstance();
            virtual ~iohcSystemTable() = default;
            bool addObject(IOHC::iohcPacket *iohc);
            bool addObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags);
            bool addObject(iohcObject *obj);
            bool addObject(std::string key, std::string serialized);
//            bool add2WObject(std::string key, std::string serialized);
            bool empty();
            uint8_t size();
            void clear();
            inline iohcSystemTable::Objects::iterator begin();
            inline iohcSystemTable::Objects::iterator end();
            bool save(bool force = false);
            void dump1W();
            void dump2W();

        private:
            iohcSystemTable();
            bool load();
            bool changed = false;

            static iohcSystemTable *_iohcSystemTable;
            Objects _objects;

        protected:

    };
}
#endif