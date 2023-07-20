#include <iohcSystemTable.h>
#include <LittleFS.h>

namespace IOHC
{
    iohcSystemTable *iohcSystemTable::_iohcSystemTable = nullptr;


    iohcSystemTable::iohcSystemTable()
    {
        ;
    }

    iohcSystemTable *iohcSystemTable::getInstance()
    {
        if (!_iohcSystemTable)
            _iohcSystemTable = new iohcSystemTable();
        return _iohcSystemTable;
    }

    bool iohcSystemTable::addObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags)
    {
        std::string s0 ((char *)node, 3);
        iohcObject *tmp;
        tmp = new iohcObject (node, backbone, actuator, manufacturer, flags); 
        return(objects.insert_or_assign(s0, tmp).second);
    return true;
    }

    bool iohcSystemTable::addObject(iohcObject *obj)
    {
        std::string s0 ((char *)obj->getNode(), 3);
        return(objects.insert_or_assign(s0, obj).second);
    return true;
    }

    uint8_t iohcSystemTable::size(void)
    {
        return(objects.size());
    }

    bool iohcSystemTable::load(void)
    {
        return true;
    }

    bool iohcSystemTable::save(bool force)
    {
        LittleFSConfig lcfg;

        lcfg.setAutoFormat(false);
        LittleFS.setConfig(lcfg);

        LittleFS.begin();
        fs::File f = LittleFS.open("/sysTable.txt", "w+");
//        f.write();
        f.close();
        return true;
    }

    void iohcSystemTable::dump(void)
    {
        Serial.printf("********************* Discovered Devices **********************\n");
        for (auto entry : objects)
            entry.second->dump();
        Serial.printf("***************************************************************\n");
    }
}