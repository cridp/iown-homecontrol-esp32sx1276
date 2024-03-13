#include <iohcDevice.h>

namespace IOHC {
        
            bool iohcDevice::isFake(address nodeSrc, address nodeDst) {
                this->Fake = false;
                return this->Fake;
            }
            bool iohcDevice::isHome(address nodeSrc, address nodeDst) {
                this->Home = false;
                return this->Home;
            }

}