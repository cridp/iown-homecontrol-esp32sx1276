#include <iohcObject.h>


namespace IOHC
{
    iohcObject::iohcObject(address node, address backbone, uint8_t actuator[2], uint8_t manufacturer, uint8_t flags)
    {
        for (uint8_t i=0; i<3; i++)
        {
            iohcDevice.node[i] = node[i];
            iohcDevice.backbone[i] = backbone[i];
            if (i<2)
                iohcDevice.actuator[i] = actuator[i];
        }

        iohcDevice.flags = flags;
        iohcDevice.io_manufacturer = manufacturer;
    }

    address *iohcObject::getNode(void)
    {
        return((address *)iohcDevice.node);
    }

    address *iohcObject::getBackbone(void)
    {
        return((address *)iohcDevice.backbone);
    }

    std::tuple<uint16_t, uint8_t> iohcObject::getTypeSub(void)
    {
        return std::make_tuple(((iohcDevice.actuator[0]<<8) + (iohcDevice.actuator[1]))>>6, iohcDevice.actuator[1] & 0x3f);
    }

    void iohcObject::dump(void)
    {
        Serial.printf("Address: %2.2x%2.2x%2.2x, ", iohcDevice.node[0], iohcDevice.node[1], iohcDevice.node[2]);
        Serial.printf("Backbone: %2.2x%2.2x%2.2x, ", iohcDevice.backbone[0], iohcDevice.backbone[1], iohcDevice.backbone[2]);
        Serial.printf("Typ/Sub: %2.2x%2.2x, ", iohcDevice.actuator[0], iohcDevice.actuator[1]);
        Serial.printf("lp: %u, io: %u, rf: %u, ta: %2.2ums, ", iohcDevice.flags&0x03?1:0, iohcDevice.flags&0x04?1:0, iohcDevice.flags&0x08?1:0, 5<<(iohcDevice.flags>>6));
        Serial.printf("%s\n", (iohcDevice.io_manufacturer <= MAX_MANUFACTURER)?man_id[iohcDevice.io_manufacturer-1]:man_id[MAX_MANUFACTURER-1]);
    }
}