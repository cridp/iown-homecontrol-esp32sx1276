#pragma once

#include <Arduino.h>
#include <map>
#include <string>
#include <iohcObject.h>


namespace IOHC
{
    class iohcSystemTable
    {
        public:
            static iohcSystemTable *getInstance();
            virtual ~iohcSystemTable() {};
            bool addObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags);
            bool addObject(iohcObject *obj);
            uint8_t size(void);
            void dump(void);

        private:
            iohcSystemTable();
            bool save(bool force);
            bool load(void);

            static iohcSystemTable *_iohcSystemTable;
            using Objects = std::map<std::string, IOHC::iohcObject *>;
            Objects objects;

        protected:

    };
}