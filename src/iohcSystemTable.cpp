#include <iohcSystemTable.h>
#include <LittleFS.h>
#include <ArduinoJson.h>


namespace IOHC
{
    iohcSystemTable *iohcSystemTable::_iohcSystemTable = nullptr;


    iohcSystemTable::iohcSystemTable()
    {
        load();
    }

    iohcSystemTable *iohcSystemTable::getInstance()
    {
        if (!_iohcSystemTable)
            _iohcSystemTable = new iohcSystemTable();
        return _iohcSystemTable;
    }

    bool iohcSystemTable::addObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags)
    {
        changed = true;
        std::string s0 = bytesToHexString(node, 3);
        iohcObject *tmp;
        tmp = new iohcObject (node, backbone, actuator, manufacturer, flags);
        bool inserted = _objects.insert_or_assign(s0, tmp).second;
        save();
        return(inserted);
    }

    bool iohcSystemTable::addObject(iohcObject *obj)
    {
        changed = true;
        std::string s0 = bytesToHexString((uint8_t *)obj->getNode(), 3);
        bool inserted = _objects.insert_or_assign(s0, obj).second;
        save();
        return(inserted);
    }

    bool iohcSystemTable::addObject(std::string node_id, std::string serialized)
    {
        iohcObject *tmp;
        tmp = new iohcObject (serialized); 
        bool inserted = _objects.insert_or_assign(node_id, tmp).second;
        save();
        return(inserted);
    }

    bool iohcSystemTable::empty(void)
    {
        return(_objects.empty());
    }

    uint8_t iohcSystemTable::size(void)
    {
        return(_objects.size());
    }

    void iohcSystemTable::clear(void)
    {
        return(_objects.clear());
    }

    inline iohcSystemTable::Objects::iterator iohcSystemTable::begin(void)
    {
        return(_objects.begin());
    }

    inline iohcSystemTable::Objects::iterator iohcSystemTable::end(void)
    {
        return(_objects.end());
    }

    bool iohcSystemTable::load(void)
    {
        this->empty();
        if (LittleFS.exists(IOHC_SYS_TABLE))
            Serial.printf("Loading systable objects from %s\n", IOHC_SYS_TABLE);
        else
        {
            Serial.printf("*systable objects not available\n");
            return false;
        }

        fs::File f = LittleFS.open(IOHC_SYS_TABLE, "r");
        DynamicJsonDocument doc(2048);
        deserializeJson(doc, f);
        f.close();

        // Iterate through the JSON object
        for (JsonPair kv : doc.as<JsonObject>())
        {
            const char* key = kv.key().c_str();
            JsonObject obj = kv.value().as<JsonObject>();
            for (JsonPair ov : obj)
                addObject(key, ov.value().as<std::string>());
        }
        return true;
    }

    bool iohcSystemTable::save(bool force)
    {
        if (!changed && force == false)
            return false;

        fs::File f = LittleFS.open(IOHC_SYS_TABLE, "w+");
        DynamicJsonDocument doc(2048);

        for (auto obj : _objects)
        {
            JsonObject jobj = doc.createNestedObject(obj.first);
            jobj["values"]=obj.second->serialize();
        }
        serializeJson(doc, f);
        f.close();
        changed = false;

        return true;
    }

    void iohcSystemTable::dump(void)
    {
        Serial.printf("********************** sysTable objects ***********************\n");
        for (auto entry : _objects)
            entry.second->dump();
        Serial.printf("***************************************************************\n");
        Serial.printf("\n");
    }
}