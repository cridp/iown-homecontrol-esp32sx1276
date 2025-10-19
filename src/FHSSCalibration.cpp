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
// Created by Pascal on 18/10/2025.
//
/*
 * Outil de calibration FHSS pour SX1276
 * Détermine automatiquement les meilleurs paramètres
 */

#ifndef FHSS_CALIBRATION_H
#define FHSS_CALIBRATION_H

#include <Arduino.h>
#include <vector>
#include <map>
#include <numeric>

#include "iohcRadio.h"
#include "FHSSCalibration.hpp"

namespace IOHC {
    class iohcRadio;
// class FHSSCalibration {
// // STRUCTURE POUR MESURES

// struct PacketMeasurement {
//     uint32_t timestamp;
//     uint32_t preambleDuration;   // µs
//     uint32_t payloadDuration;    // µs
//     uint32_t totalDuration;      // µs
//     uint32_t frequency;
//     bool isLPM;                  // Low Power Mode (1W)
//     bool is2W;                   // 2W mode
//     float rssi;
//     int16_t snr;
//
//     PacketMeasurement() : timestamp(0), preambleDuration(0), payloadDuration(0),
//                          totalDuration(0), frequency(0), isLPM(false),
//                          is2W(true), rssi(0), snr(0) {}
// };
//
// struct ConversationMeasurement {
//     uint32_t timestamp;
//     uint32_t requestToResponseDelay;  // µs
//     uint32_t totalDuration;           // µs
//     uint32_t frequency;
//     bool success;
//     String failReason;
//
//     ConversationMeasurement() : timestamp(0), requestToResponseDelay(0),
//                                totalDuration(0), frequency(0), success(false) {}
// };

// CLASSE DE CALIBRATION

    // iohcRadio* radio;
    //
    // // Mesures collectées
    // std::vector<PacketMeasurement> preambleMeasures1W;
    // std::vector<PacketMeasurement> preambleMeasures2W;
    // std::vector<PacketMeasurement> payloadMeasures;
    // std::vector<ConversationMeasurement> conversationMeasures;
    //
    // // Timestamps pour mesures en cours
    // volatile uint32_t currentPreambleStart = 0;
    // volatile uint32_t currentPayloadStart = 0;
    // volatile bool measuringPreamble = false;
    // volatile bool measuringPayload = false;
    //
    // // Résultats de calibration
    // struct CalibrationResults {
    //     // Durées mesurées (µs)
    //     uint32_t preamble1W_min;
    //     uint32_t preamble1W_max;
    //     uint32_t preamble1W_avg;
    //
    //     uint32_t preamble2W_min;
    //     uint32_t preamble2W_max;
    //     uint32_t preamble2W_avg;
    //
    //     uint32_t payload_min;
    //     uint32_t payload_max;
    //     uint32_t payload_avg;
    //
    //     uint32_t conversation_min;
    //     uint32_t conversation_max;
    //     uint32_t conversation_avg;
    //
    //     // Paramètres recommandés (ms)
    //     uint32_t dwellTime_fast;       // Pour monitoring passif
    //     uint32_t dwellTime_slow;       // Pour conversations
    //     uint32_t preambleTimeout;      // Timeout détection préambule
    //     uint32_t payloadTimeout;       // Timeout réception payload
    //     uint32_t responseTimeout;      // Timeout attente réponse
    //
    //     CalibrationResults() : preamble1W_min(0), preamble1W_max(0), preamble1W_avg(0),
    //                           preamble2W_min(0), preamble2W_max(0), preamble2W_avg(0),
    //                           payload_min(0), payload_max(0), payload_avg(0),
    //                           conversation_min(0), conversation_max(0), conversation_avg(0),
    //                           dwellTime_fast(20), dwellTime_slow(150),
    //                           preambleTimeout(300), payloadTimeout(50), responseTimeout(1000) {}
    // };
    //
    // CalibrationResults results;

    FHSSCalibration::FHSSCalibration(iohcRadio* r) : radio(r) {
        ets_printf("FHSS_CAL Calibration tool initialized\n");
    }

    // ========================================================================
    // PHASE 1 : MESURE DES PRÉAMBULES
    // ========================================================================

    /**
     * Démarre la mesure automatique des préambules
     * Durée : 60 secondes minimum (plus il y a de trafic, mieux c'est)
     */
    void FHSSCalibration::startPreambleMeasurement(uint32_t durationSeconds = 60) {
        ets_printf("FHSS_CAL === PHASE 1: Preamble Measurement ===\n");
        ets_printf("FHSS_CAL Duration: %d seconds\n", durationSeconds);
        ets_printf("FHSS_CAL Please trigger devices during this time...\n");

        preambleMeasures1W.clear();
        preambleMeasures2W.clear();

        uint32_t startTime = millis();
        uint32_t endTime = startTime + (durationSeconds * 1000);

        while (millis() < endTime) {
            // La mesure se fait automatiquement via les callbacks
            vTaskDelay(pdMS_TO_TICKS(100));

            // Afficher progression
            if ((millis() - startTime) % 10000 == 0) {
                ets_printf("FHSS_CAL Progress: %d/%d seconds | 1W: %d samples, 2W: %d samples\n",
                        (millis() - startTime) / 1000, durationSeconds,
                        preambleMeasures1W.size(), preambleMeasures2W.size());
            }
        }

        analyzePreambleMeasurements();
    }

    /**
     * Callback à appeler quand un préambule est détecté
     */
    void FHSSCalibration::onPreambleDetected() {
        if (!measuringPreamble) {
            measuringPreamble = true;
            currentPreambleStart = esp_timer_get_time();
        }
    }

    /**
     * Callback à appeler quand le payload commence
     */
    void FHSSCalibration::onPayloadStart(bool isLPM, uint32_t frequency) {
        if (measuringPreamble) {
            uint32_t now = esp_timer_get_time();
            uint32_t duration = now - currentPreambleStart;

            PacketMeasurement measure;
            measure.timestamp = millis();
            measure.preambleDuration = duration;
            measure.frequency = frequency;
            measure.isLPM = isLPM;
            measure.is2W = !isLPM;

            if (isLPM) {
                preambleMeasures1W.push_back(measure);
            } else {
                preambleMeasures2W.push_back(measure);
            }

            measuringPreamble = false;
            measuringPayload = true;
            currentPayloadStart = now;
        }
    }

    // ========================================================================
    // PHASE 2 : MESURE DES PAYLOADS
    // ========================================================================

    /**
     * Callback à appeler quand le payload est reçu
     */
    void FHSSCalibration::onPayloadComplete(float rssi, int16_t snr) {
        if (measuringPayload) {
            uint32_t now = esp_timer_get_time();
            uint32_t duration = now - currentPayloadStart;

            PacketMeasurement measure;
            measure.timestamp = millis();
            measure.payloadDuration = duration;
            measure.totalDuration = now - currentPreambleStart;
            measure.rssi = rssi;
            measure.snr = snr;

            payloadMeasures.push_back(measure);

            measuringPayload = false;
        }
    }

    // ========================================================================
    // PHASE 3 : MESURE DES CONVERSATIONS
    // ========================================================================

    /**
     * Test des conversations bidirectionnelles
     * Envoie des commandes et mesure le temps jusqu'à la réponse
     */
    // void FHSSCalibration::startConversationTest(uint32_t numTests = 50) {
    //     ets_printf("FHSS_CAL === PHASE 3: Conversation Test ===");
    //     ets_printf("FHSS_CAL Number of tests: %d", numTests);
    //
    //     conversationMeasures.clear();
    //
    //     for (uint32_t i = 0; i < numTests; i++) {
    //         ConversationMeasurement measure;
    //         measure.timestamp = millis();
    //
    //         // Envoyer une commande et mesurer
    //         uint32_t requestTime = esp_timer_get_time();
    //         bool success = sendTestCommand();
    //
    //         if (success) {
    //             // Attendre la réponse (max 2 secondes)
    //             uint32_t timeout = millis() + 2000;
    //             bool gotResponse = false;
    //
    //             while (millis() < timeout && !gotResponse) {
    //                 gotResponse = checkForResponse();
    //                 vTaskDelay(pdMS_TO_TICKS(1));
    //             }
    //
    //             if (gotResponse) {
    //                 uint32_t responseTime = esp_timer_get_time();
    //                 measure.requestToResponseDelay = responseTime - requestTime;
    //                 measure.totalDuration = responseTime - requestTime;
    //                 measure.success = true;
    //             } else {
    //                 measure.success = false;
    //                 measure.failReason = "Timeout";
    //             }
    //         } else {
    //             measure.success = false;
    //             measure.failReason = "Send failed";
    //         }
    //
    //         conversationMeasures.push_back(measure);
    //
    //         // Afficher progression
    //         if ((i + 1) % 10 == 0) {
    //             uint32_t successCount = 0;
    //             for (const auto& m : conversationMeasures) {
    //                 if (m.success) successCount++;
    //             }
    //             ets_printf("FHSS_CAL Progress: %d/%d | Success rate: %.1f%%",
    //                     i + 1, numTests, (float)successCount / (i + 1) * 100.0f);
    //         }
    //
    //         // Délai entre tests
    //         vTaskDelay(pdMS_TO_TICKS(200));
    //     }
    //
    //     analyzeConversationMeasurements();
    // }

    // ========================================================================
    // PHASE 4 : TEST DE DWELL TIME OPTIMAL
    // ========================================================================

    /**
     * Test différents dwell times et mesure le taux de succès
     */
    struct DwellTimeTestResult {
        uint32_t dwellTimeMs;
        uint32_t packetsDetected;
        uint32_t packetsMissed;
        uint32_t conversationsSuccessful;
        uint32_t conversationsFailed;
        float detectionRate;
        float conversationSuccessRate;
    };

    // std::vector<DwellTimeTestResult> FHSSCalibration::testDwellTimes(
    //     std::vector<uint32_t> dwellTimesToTest = {5, 10, 20, 30, 50, 100, 150, 200})
    // {
    //     ets_printf("FHSS_CAL === PHASE 4: Dwell Time Optimization ===");
    //
    //     std::vector<DwellTimeTestResult> results;
    //
    //     for (uint32_t dwellTime : dwellTimesToTest) {
    //         ets_printf("FHSS_CAL Testing dwell time: %d ms", dwellTime);
    //
    //         // Configurer le dwell time
    //         radio->scanTimeUs = dwellTime * 1000;
    //         radio->TickTimer.detach();
    //         radio->TickTimer.attach_us(radio->scanTimeUs, FHSSTimer, radio);
    //
    //         DwellTimeTestResult result;
    //         result.dwellTimeMs = dwellTime;
    //
    //         // Test 1 : Détection passive (30 secondes)
    //         uint32_t startTime = millis();
    //         uint32_t initialPackets = radio->getTotalPacketsReceived();
    //
    //         vTaskDelay(pdMS_TO_TICKS(30000));
    //
    //         result.packetsDetected = radio->getTotalPacketsReceived() - initialPackets;
    //
    //         // Test 2 : Conversations (20 tests)
    //         uint32_t successCount = 0;
    //         for (uint32_t i = 0; i < 20; i++) {
    //             if (sendTestCommand()) {
    //                 if (waitForResponse(1000)) {
    //                     successCount++;
    //                 }
    //             }
    //             vTaskDelay(pdMS_TO_TICKS(200));
    //         }
    //
    //         result.conversationsSuccessful = successCount;
    //         result.conversationsFailed = 20 - successCount;
    //         result.detectionRate = result.packetsDetected / 30.0f;  // pkt/sec
    //         result.conversationSuccessRate = (float)successCount / 20.0f * 100.0f;
    //
    //         results.push_back(result);
    //
    //         ets_printf("FHSS_CAL   Detection rate: %.2f pkt/s", result.detectionRate);
    //         ets_printf("FHSS_CAL   Conversation success: %.1f%%", result.conversationSuccessRate);
    //     }
    //
    //     return results;
    // }

    // ========================================================================
    // ANALYSE ET RÉSULTATS
    // ========================================================================

    void FHSSCalibration::analyzePreambleMeasurements() {
        ets_printf("FHSS_CAL === Preamble Analysis ===\n");

        if (!preambleMeasures1W.empty()) {
            auto [min1W, max1W, avg1W] = calculateStats(preambleMeasures1W,
                [](const PacketMeasurement& m) { return m.preambleDuration; });

            results.preamble1W_min = min1W;
            results.preamble1W_max = max1W;
            results.preamble1W_avg = avg1W;

            ets_printf("FHSS_CAL 1W Preambles (%d samples):\n", preambleMeasures1W.size());
            ets_printf("FHSS_CAL   Min: %d µs (%.2f ms)\n", min1W, min1W / 1000.0f);
            ets_printf("FHSS_CAL   Max: %d µs (%.2f ms)\n", max1W, max1W / 1000.0f);
            ets_printf("FHSS_CAL   Avg: %d µs (%.2f ms)\n", avg1W, avg1W / 1000.0f);
        } else {
            ets_printf("FHSS_CAL No 1W preambles detected");
        }

        if (!preambleMeasures2W.empty()) {
            auto [min2W, max2W, avg2W] = calculateStats(preambleMeasures2W,
                [](const PacketMeasurement& m) { return m.preambleDuration; });

            results.preamble2W_min = min2W;
            results.preamble2W_max = max2W;
            results.preamble2W_avg = avg2W;

            ets_printf("FHSS_CAL 2W Preambles (%d samples):\n", preambleMeasures2W.size());
            ets_printf("FHSS_CAL   Min: %d µs (%.2f ms)\n", min2W, min2W / 1000.0f);
            ets_printf("FHSS_CAL   Max: %d µs (%.2f ms)\n", max2W, max2W / 1000.0f);
            ets_printf("FHSS_CAL   Avg: %d µs (%.2f ms)\n", avg2W, avg2W / 1000.0f);
        } else {
            ets_printf("FHSS_CAL No 2W preambles detected");
        }

        if (!payloadMeasures.empty()) {
            auto [minPay, maxPay, avgPay] = calculateStats(payloadMeasures,
                [](const PacketMeasurement& m) { return m.payloadDuration; });

            results.payload_min = minPay;
            results.payload_max = maxPay;
            results.payload_avg = avgPay;

            ets_printf("FHSS_CAL Payloads (%d samples):\n", payloadMeasures.size());
            ets_printf("FHSS_CAL   Min: %d µs (%.2f ms)\n", minPay, minPay / 1000.0f);
            ets_printf("FHSS_CAL   Max: %d µs (%.2f ms)\n", maxPay, maxPay / 1000.0f);
            ets_printf("FHSS_CAL   Avg: %d µs (%.2f ms)\n", avgPay, avgPay / 1000.0f);
        }
    }

    void FHSSCalibration::analyzeConversationMeasurements() {
        ets_printf("FHSS_CAL === Conversation Analysis ===");

        if (conversationMeasures.empty()) {
            ets_printf("FHSS_CAL No conversation measurements");
            return;
        }

        uint32_t successCount = 0;
        std::vector<uint32_t> successfulDurations;

        for (const auto& m : conversationMeasures) {
            if (m.success) {
                successCount++;
                successfulDurations.push_back(m.totalDuration);
            }
        }

        float successRate = (float)successCount / conversationMeasures.size() * 100.0f;
        ets_printf("FHSS_CAL Success rate: %.1f%% (%d/%d)",
                successRate, successCount, conversationMeasures.size());

        if (!successfulDurations.empty()) {
            auto [minConv, maxConv, avgConv] = calculateStatsVector(successfulDurations);

            results.conversation_min = minConv;
            results.conversation_max = maxConv;
            results.conversation_avg = avgConv;

            ets_printf("FHSS_CAL Conversation duration:");
            ets_printf("FHSS_CAL   Min: %d µs (%.2f ms)", minConv, minConv / 1000.0f);
            ets_printf("FHSS_CAL   Max: %d µs (%.2f ms)", maxConv, maxConv / 1000.0f);
            ets_printf("FHSS_CAL   Avg: %d µs (%.2f ms)", avgConv, avgConv / 1000.0f);
        }
    }

    /**
     * Calcule les paramètres optimaux basés sur les mesures
     */
    void FHSSCalibration::calculateOptimalParameters() {
        ets_printf("FHSS_CAL === CALCULATING OPTIMAL PARAMETERS ===");

        // Dwell time FAST : doit être < min preamble 2W pour détecter
        // Mais pas trop court pour ne pas rater le début
        if (results.preamble2W_min > 0) {
            results.dwellTime_fast = (results.preamble2W_min / 1000) / 2;  // 50% du préambule min
            results.dwellTime_fast = constrain(results.dwellTime_fast, 10, 30);  // 10-30ms
        }

        // Dwell time SLOW : doit être > conversation max pour maintenir la fréquence
        if (results.conversation_max > 0) {
            results.dwellTime_slow = (results.conversation_max / 1000) + 20;  // +20ms marge
            results.dwellTime_slow = constrain(results.dwellTime_slow, 100, 300);  // 100-300ms
        }

        // Timeout préambule : max preamble 1W + marge
        if (results.preamble1W_max > 0) {
            results.preambleTimeout = (results.preamble1W_max / 1000) + 50;  // +50ms marge
        }

        // Timeout payload : max payload + marge
        if (results.payload_max > 0) {
            results.payloadTimeout = (results.payload_max / 1000) + 20;  // +20ms marge
        }

        // Timeout réponse : max conversation + marge
        if (results.conversation_max > 0) {
            results.responseTimeout = (results.conversation_max / 1000) + 200;  // +200ms marge
        }

        printOptimalParameters();
    }

    void FHSSCalibration::printOptimalParameters() {
        ets_printf("FHSS_CAL ╔════════════════════════════════════════════════════════╗");
        ets_printf("FHSS_CAL ║        OPTIMAL FHSS PARAMETERS (RECOMMENDED)          ║");
        ets_printf("FHSS_CAL ╠════════════════════════════════════════════════════════╣");
        ets_printf("FHSS_CAL ║ Dwell Time (Fast Scan):    %4d ms                    ║", results.dwellTime_fast);
        ets_printf("FHSS_CAL ║ Dwell Time (Slow Scan):    %4d ms                    ║", results.dwellTime_slow);
        ets_printf("FHSS_CAL ║ Preamble Timeout:          %4d ms                    ║", results.preambleTimeout);
        ets_printf("FHSS_CAL ║ Payload Timeout:           %4d ms                    ║", results.payloadTimeout);
        ets_printf("FHSS_CAL ║ Response Timeout:          %4d ms                    ║", results.responseTimeout);
        ets_printf("FHSS_CAL ╚════════════════════════════════════════════════════════╝");
        ets_printf("FHSS_CAL \n");

        // Code à copier-coller
        ets_printf("FHSS_CAL Copy-paste this configuration:");
        ets_printf("FHSS_CAL ");
        ets_printf("FHSS_CAL // FHSS Configuration (calibrated)");
        ets_printf("FHSS_CAL #define FHSS_DWELL_TIME_FAST_MS    %d", results.dwellTime_fast);
        ets_printf("FHSS_CAL #define FHSS_DWELL_TIME_SLOW_MS    %d", results.dwellTime_slow);
        ets_printf("FHSS_CAL #define FHSS_PREAMBLE_TIMEOUT_MS   %d", results.preambleTimeout);
        ets_printf("FHSS_CAL #define FHSS_PAYLOAD_TIMEOUT_MS    %d", results.payloadTimeout);
        ets_printf("FHSS_CAL #define FHSS_RESPONSE_TIMEOUT_MS   %d", results.responseTimeout);
        ets_printf("FHSS_CAL ");
    }

    /**
     * Export vers un fichier JSON pour le SX1276
     */
    void FHSSCalibration::exportToJSON(const char* filename) {
        // TODO: Implémenter export JSON pour configuration SX1276
        ets_printf("FHSS_CAL Exporting to %s...", filename);
    }

    // Helpers
    template<typename T, typename F>
    std::tuple<uint32_t, uint32_t, uint32_t> FHSSCalibration::calculateStats(const std::vector<T>& data, F extractor) {
        if (data.empty()) return {0, 0, 0};

        uint32_t min = UINT32_MAX;
        uint32_t max = 0;
        uint64_t sum = 0;

        for (const auto& item : data) {
            uint32_t value = extractor(item);
            min = std::min(min, value);
            max = std::max(max, value);
            sum += value;
        }

        uint32_t avg = sum / data.size();
        return {min, max, avg};
    }

    std::tuple<uint32_t, uint32_t, uint32_t> FHSSCalibration::calculateStatsVector(const std::vector<uint32_t>& data) {
        if (data.empty()) return {0, 0, 0};

        uint32_t min = *std::ranges::min_element(data);
        uint32_t max = *std::ranges::max_element(data);
        uint64_t sum = std::accumulate(data.begin(), data.end(), 0ULL);
        uint32_t avg = sum / data.size();

        return {min, max, avg};
    }

    bool FHSSCalibration::sendTestCommand() {
        // Implémenter l'envoi d'une commande de test
        // Retourne true si envoi réussi
        return true;
    }

    bool FHSSCalibration::checkForResponse() {
        // Implémenter la vérification de réponse
        return false;
    }

    bool FHSSCalibration::waitForResponse(uint32_t timeoutMs) {
        uint32_t start = millis();
        while (millis() - start < timeoutMs) {
            if (checkForResponse()) return true;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return false;
    }
// };

} // namespace IOHC

// ============================================================================
// EXEMPLE D'UTILISATION
// ============================================================================

void runFHSSCalibration() {
    IOHC::iohcRadio* radio = IOHC::iohcRadio::getInstance();
    IOHC::FHSSCalibration calibration(radio);

    Serial.println("\n");
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║       FHSS AUTOMATIC CALIBRATION TOOL v1.0          ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.println("");
    Serial.println("This tool will automatically determine optimal FHSS");
    Serial.println("parameters for your environment.");
    // Serial.println("");
    // Serial.println("Press ENTER to start...");
    // while (!Serial.available()) { delay(10); }

    // Phase 1 : Mesure passive des préambules (60 secondes)
    Serial.println("\n▶ PHASE 1: Passive preamble measurement");
    Serial.println("  Trigger some devices (remotes, sensors, etc.)");
    calibration.startPreambleMeasurement(60);

    // Phase 2 : Test des conversations (automatique)
    // Serial.println("\n▶ PHASE 2: Conversation test");
    // Serial.println("  The system will send test commands");
    // calibration.startConversationTest(50);

    // Phase 3 : Test des différents dwell times
    // Serial.println("\n▶ PHASE 3: Dwell time optimization");
    // auto dwellResults = calibration.testDwellTimes();
    //
    // // Afficher les résultats des tests de dwell time
    // Serial.println("\nDwell Time Test Results:");
    // Serial.println("┌──────────┬─────────────┬────────────────┐");
    // Serial.println("│ Dwell(ms)│ Detect rate │ Conv. success  │");
    // Serial.println("├──────────┼─────────────┼────────────────┤");
    // for (const auto& result : dwellResults) {
    //     Serial.printf("│ %8d │ %9.2f/s │ %13.1f%% │\n",
    //                  result.dwellTimeMs,
    //                  result.detectionRate,
    //                  result.conversationSuccessRate);
    // }
    // Serial.println("└──────────┴─────────────┴────────────────┘");

    // Phase 4 : Calcul et affichage des paramètres optimaux
    // Serial.println("▶ PHASE 4: Calculating optimal parameters");
    // calibration.calculateOptimalParameters();

    Serial.println("✓ Calibration complete!");
    Serial.println("  You can now use these parameters in your code.");
}

#endif // FHSS_CALIBRATION_H