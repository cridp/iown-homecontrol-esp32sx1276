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

#ifndef IOWNHCESP32_FHSSCALIBRATION_HPP
#define IOWNHCESP32_FHSSCALIBRATION_HPP
#include <cstdint>
#include <vector>

#include "WString.h"

namespace IOHC {
    class iohcRadio;
    void runFHSSCalibration();

    class FHSSCalibration {
    public:
        FHSSCalibration(iohcRadio* r);
        ~FHSSCalibration() = default;

        void startPreambleMeasurement(std::uint32_t durationSeconds);
        void onPreambleDetected();
        void onPayloadStart(bool isLPM, uint32_t frequency);
        void onPayloadComplete(float rssi, int16_t snr);

        // void startConversationTest(uint32_t numTests) {}

        struct DwellTimeTestResult {
            uint32_t dwellTimeMs;
            uint32_t packetsDetected;
            uint32_t packetsMissed;
            uint32_t conversationsSuccessful;
            uint32_t conversationsFailed;
            float detectionRate;
            float conversationSuccessRate;
        };

        // std::vector<DwellTimeTestResult> testDwellTimes(std::vector<uint32_t> dwellTimesToTest = {5, 10, 20, 30, 50, 100, 150, 200});

        void analyzePreambleMeasurements();
        void analyzeConversationMeasurements();
        void calculateOptimalParameters();
        void printOptimalParameters();
        void exportToJSON(const char* filename);

        // ============================================================================
        // STRUCTURE POUR MESURES
        // ============================================================================

        struct PacketMeasurement {
            uint32_t timestamp;
            uint32_t preambleDuration;   // µs
            uint32_t payloadDuration;    // µs
            uint32_t totalDuration;      // µs
            uint32_t frequency;
            bool isLPM;                  // Low Power Mode (1W)
            bool is2W;                   // 2W mode
            float rssi;
            int16_t snr;

            PacketMeasurement() : timestamp(0), preambleDuration(0), payloadDuration(0),
                                 totalDuration(0), frequency(0), isLPM(false),
                                 is2W(true), rssi(0), snr(0) {}
        };

        struct ConversationMeasurement {
            uint32_t timestamp;
            uint32_t requestToResponseDelay;  // µs
            uint32_t totalDuration;           // µs
            uint32_t frequency;
            bool success;
            String failReason;

            ConversationMeasurement() : timestamp(0), requestToResponseDelay(0),
                                       totalDuration(0), frequency(0), success(false) {}
        };

    private:
        // Helpers
        template<typename T, typename F>
        std::tuple<uint32_t, uint32_t, uint32_t> calculateStats(const std::vector<T>& data, F extractor);
        std::tuple<uint32_t, uint32_t, uint32_t> calculateStatsVector(const std::vector<uint32_t>& data);
        bool sendTestCommand();
        bool checkForResponse();
        bool waitForResponse(uint32_t timeoutMs);

        iohcRadio* radio;

        // Mesures collectées
        std::vector<PacketMeasurement> preambleMeasures1W;
        std::vector<PacketMeasurement> preambleMeasures2W;
        std::vector<PacketMeasurement> payloadMeasures;
        std::vector<ConversationMeasurement> conversationMeasures;

        // Timestamps pour mesures en cours
        volatile uint32_t currentPreambleStart = 0;
        volatile uint32_t currentPayloadStart = 0;
        volatile bool measuringPreamble = false;
        volatile bool measuringPayload = false;

        // Résultats de calibration
        struct CalibrationResults {
            // Durées mesurées (µs)
            uint32_t preamble1W_min;
            uint32_t preamble1W_max;
            uint32_t preamble1W_avg;

            uint32_t preamble2W_min;
            uint32_t preamble2W_max;
            uint32_t preamble2W_avg;

            uint32_t payload_min;
            uint32_t payload_max;
            uint32_t payload_avg;

            uint32_t conversation_min;
            uint32_t conversation_max;
            uint32_t conversation_avg;

            // Paramètres recommandés (ms)
            uint32_t dwellTime_fast;       // Pour monitoring passif
            uint32_t dwellTime_slow;       // Pour conversations
            uint32_t preambleTimeout;      // Timeout détection préambule
            uint32_t payloadTimeout;       // Timeout réception payload
            uint32_t responseTimeout;      // Timeout attente réponse

            CalibrationResults() : preamble1W_min(0), preamble1W_max(0), preamble1W_avg(0),
                                  preamble2W_min(0), preamble2W_max(0), preamble2W_avg(0),
                                  payload_min(0), payload_max(0), payload_avg(0),
                                  conversation_min(0), conversation_max(0), conversation_avg(0),
                                  dwellTime_fast(20), dwellTime_slow(150),
                                  preambleTimeout(300), payloadTimeout(50), responseTimeout(1000) {}
        };

        CalibrationResults results;


    };
}
#endif //IOWNHCESP32_FHSSCALIBRATION_HPP