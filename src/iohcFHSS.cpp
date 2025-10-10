//   Copyright (c) 2025. CRIDP https://github.com/cridp
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//           http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

//
// Created by Pascal on 10/10/2025.
//

#include "iohcFHSS.hpp"
#include "iohcRadio.h"

namespace IOHC {

        void AdaptiveFHSS::switchToFastScan() {
            if (currentMode == ScanMode::FAST_SCAN) return;

            currentMode = ScanMode::FAST_SCAN;
            radio->scanTimeUs = 20 * 1000;  // 20ms
            this->radio->TickTimer.detach();
            this->radio->TickTimer.attach_us(this->radio->scanTimeUs, FHSSTimer, this->radio);

            // ESP_LOGI("FHSS", "Switched to FAST scan (20ms/freq)");
        }

        void AdaptiveFHSS::switchToSlowScan() {
            if (currentMode == ScanMode::SLOW_SCAN) return;

            currentMode = ScanMode::SLOW_SCAN;
            radio->scanTimeUs = 150 * 1000;  // 150ms
            radio->TickTimer.detach();
            radio->TickTimer.attach_us(this->radio->scanTimeUs, FHSSTimer, this->radio);

            // ESP_LOGI("FHSS", "Switched to SLOW scan (150ms/freq)");
        }

        void AdaptiveFHSS::prepareForConversation() {
            conversationStartTime = esp_timer_get_time() / 1000;
            inConversation = true;
            switchToSlowScan();
        }
}
