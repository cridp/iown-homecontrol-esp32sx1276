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
    /**
    * fast_dwell: 10–20 ms.
    * Pourquoi : avec 10 ms la probabilité devient ≈ 10/13.33 = 75% (pour P=64).
    * Avec 15 ms, on couvre presque tout le préambule (≈112% i.e. sûr).
    */
    void AdaptiveFHSS::switchToFastScan(int ms) {
            if (currentMode == ScanMode::FAST_SCAN) return;
;
            currentMode = ScanMode::FAST_SCAN;
            radio->scanTimeUs = ms != fast_dwell? ms/* * 1000*/: fast_dwell/* * 1000*/;
            radio->TickTimer.detach();
            radio->TickTimer.attach_ms(radio->scanTimeUs, FHSSTimer, radio);

            ets_printf("FHSS Switched to FAST scan (%dms/freq)\n", radio->scanTimeUs/* / 1000*/);
        }

//         void AdaptiveFHSS::switchToSlowScan(int ms) {
//             if (currentMode == ScanMode::SLOW_SCAN) return;
// ;
//             currentMode = ScanMode::SLOW_SCAN;
//             radio->scanTimeUs = ms != slow_dwell? ms/* * 1000*/: slow_dwell/* * 1000*/;
//             radio->TickTimer.detach();
//             radio->TickTimer.attach_ms(radio->scanTimeUs, FHSSTimer, radio);
//
//             ets_printf("FHSS Switched to SLOW scan (%dms/freq)\n", radio->scanTimeUs/* / 1000*/);
//         }


        void AdaptiveFHSS::update() {
            if (!inConversation) return;

            uint32_t now = esp_timer_get_time() / 1000;
            uint32_t timeSinceActivity = now - lastPacketTime;
            uint32_t conversationDuration = now - conversationStartTime;

            // Retour en fast scan si :
            // - Pas d'activité depuis CONVERSATION_TIMEOUT_MS
            // - ET conversation terminée depuis assez longtemps
            if (timeSinceActivity > ACTIVITY_TIMEOUT_MS &&
                conversationDuration > CONVERSATION_TIMEOUT_MS) {
                inConversation = false;
                switchToFastScan(fast_dwell);
                }

        }

        // void AdaptiveFHSS::prepareForConversation() {
        //     conversationStartTime = esp_timer_get_time() / 1000;
        //     inConversation = true;
        //     switchToSlowScan(slow_dwell);
        // }

        // void AdaptiveFHSS::onCollisionDetected() {}
}
