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

#ifndef IOHCFHSS_HPP
#define IOHCFHSS_HPP
#include <cstdint>
#include <esp_timer.h>

// CONFIGURATIONS FHSS POSSIBLES
/*
struct FHSSConfig {
    std::uint32_t dwellTimeMs; // Temps par fréquence
    const char *description;
    const char *pros;
    const char *cons;
    bool recommendedForConversation;
};

constexpr FHSSConfig FHSS_CONFIGS[] = {
        // CONFIG 1 : (PROBLÉMATIQUE)
        {
        .dwellTimeMs = 3, // 2.75ms arrondi
        .description = "Scan très rapide - 3 fréqs en 9ms",
        .pros = "Détection rapide des paquets isolés",
        .cons = "IMPOSSIBLE de maintenir une conversation - trop rapide!",
        .recommendedForConversation = false
        },

        // CONFIG 2 : Scan équilibré (RECOMMANDÉ pour monitoring passif)
        {
        .dwellTimeMs = 20, // 60ms total pour 3 fréquences
        .description = "Scan équilibré - détection préambule court",
        .pros = "Détecte tous les préambules courts (>40ms), bon compromis",
        .cons = "Peut rater des conversations si on saute au mauvais moment",
        .recommendedForConversation = false
        },

        // CONFIG 3 : Scan lent (RECOMMANDÉ pour mode actif/conversation)
        {
        .dwellTimeMs = 150, // 450ms total pour 3 fréquences
        .description = "Scan lent - garantit conversations complètes",
        .pros = "Permet conversations complètes (128ms < 150ms)",
        .cons = "Détection plus lente des paquets sur autres fréquences",
        .recommendedForConversation = true
        },

        // CONFIG 4 : Mode hybride (OPTIMAL)
        {
        .dwellTimeMs = 0, // Géré dynamiquement
        .description = "Mode adaptatif selon activité",
        .pros = "Rapide en veille, lent en conversation",
        .cons = "Plus complexe à implémenter",
        .recommendedForConversation = true
        }
        };
*/
namespace IOHC {
    class iohcRadio;

    class AdaptiveFHSS {
        IOHC::iohcRadio *radio;

        int fast_dwell = 20;
        int slow_dwell = 120;
        // Modes de scanning
        enum class ScanMode {
            FAST_SCAN, // Scan rapide : 20ms/freq (monitoring passif)
            SLOW_SCAN, // Scan lent : 120ms/freq (conversation possible)
            LOCKED // Verrouillé sur une fréquence
        };

        ScanMode currentMode = ScanMode::LOCKED;
        uint32_t lastPacketTime = 0;
        uint32_t conversationStartTime = 0;
        bool inConversation = false;

        // Timeouts
        static constexpr uint32_t CONVERSATION_TIMEOUT_MS = 300; // Retour en fast après 300ms
        static constexpr uint32_t ACTIVITY_TIMEOUT_MS = 1000; // Considéré "inactif" après 1s

    public:
        explicit AdaptiveFHSS(iohcRadio *r) : radio(r) { }

        /**
         * Appelé quand un paquet est détecté (RX ou TX)
         */
        void onPacketActivity(bool expectsResponse = false) {
            uint32_t now = esp_timer_get_time() / 1000;
            lastPacketTime = now;

            // Si on attend une réponse ou qu'on est en conversation
            if (expectsResponse ||
                (now - conversationStartTime) < CONVERSATION_TIMEOUT_MS) {
                if (!inConversation) {
                    conversationStartTime = now;
                    inConversation = true;
                    switchToSlowScan();
                }
            }
        }

        /**
        * Appelé régulièrement pour vérifier si on peut repasser en fast scan
        */
        void update();

        void switchToSlowScan(int ms = 20);

        void switchToFastScan(int ms = 120);

        /**
         * Force slow scan
         */
        void prepareForConversation();
    };
}
#endif //IOHCFHSS_HPP
