#pragma once

//#include <Arduino.h>
#include <map>
#include <string>
#include <iohcObject.h>


#define IOHC_SYS_TABLE  "/sysTable.json"


/*
    Singleton class to implement the System Object Table.
    System Object Table tracks all managed devices with their base info

    At this time this is only a container.
*/
namespace IOHC
{
    class iohcSystemTable
    {
        public:
            using Objects = std::map<std::string, iohcObject *>;

            static iohcSystemTable *getInstance();
            virtual ~iohcSystemTable() {};
            bool addObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags);
            bool addObject(iohcObject *obj);
            bool addObject(std::string key, std::string serialized);
            bool empty(void);
            uint8_t size(void);
            void clear(void);
            inline iohcSystemTable::Objects::iterator begin(void);
            inline iohcSystemTable::Objects::iterator end(void);
            bool save(bool force = false);
            void dump(void);

        private:
            iohcSystemTable();
            bool load(void);
            bool changed = false;

            static iohcSystemTable *_iohcSystemTable;
            Objects _objects;

        protected:

    };
}