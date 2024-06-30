/*
   Copyright (c) 2024. CRIDP https://github.com/cridp

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

           http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

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

    /**
    * @brief Dump the scan result to the console for debugging purposes. \ ingroup iohcCozy
    */
    void iohcDevice::scanDump() {
        printf("*********************** Scan result ***********************\n");

        uint8_t count = 0;

        for (auto &it: mapValid) {
            // Prints the first two bytes of the second.
            // Prints the token and argument.
            if (it.second != 0x08) {
                // Prints the first and second of the token.
                // Prints the argument string representation of the argument.
                if (it.second == 0x3C)
                    printf("%2.2x=AUTH ", it.first, it.second);
                    // Prints the string representation of the argument.
                    // Prints the string representation of the argument.
                else if (it.second == 0x80)
                    printf("%2.2x=NRDY ", it.first, it.second);
                else
                    printf("%2.2x=%2.2x\t", it.first, it.second);
                count++;
                // Prints the number of bytes to the console.
                // Prints the number of bytes to the console.
                if (count % 16 == 0) printf("\n");
            }
        }

        // Prints the number of bytes to the console.
        if (count % 16 != 0) printf("\n");

        printf("%u toCheck \n", count);
    }
}
