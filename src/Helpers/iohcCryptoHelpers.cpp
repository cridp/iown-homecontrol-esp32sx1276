/*
   Copyright (c) 2024-2026. CRIDP https://github.com/cridp

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

#include <iohcCryptoHelpers.h>
#include <rom/ets_sys.h>
#include "mbedtls/aes.h"

/**
    Helper function to convert a string containing hex numbers to a bytes sequence; one byte every two characters
*/
uint8_t hexStringToBytes(const std::string hexString, uint8_t *byteString) {
size_t i;

    if (hexString.length() % 2 != 0)
        return 0;

    for (i = 0; i < hexString.length(); i += 2)
        byteString[i/2] = (char) strtol(hexString.substr(i, 2).c_str(), nullptr, 16);

    return i/2;
}

/**
    Helper function to convert a byte sequence to an hex string
*/
std::string bytesToHexString(const uint8_t *byteString, uint8_t len) {
    char rec[len*2+1];

    for (uint8_t i=0; i<len; i++)
        sprintf((rec+(i*2)), "%2.2x", byteString[i]);

    return std::string(rec);
}

namespace iohcCrypto {
    // Unified keys for 1-Way and 2-Way communication
//    uint8_t system_key[16] = {0xCE, 0x0D, 0x4B, 0x2F, 0x5C, 0x68, 0x24, 0x93, 0x2D, 0xFF, 0xED, 0x7E, 0x70, 0x06, 0xD3, 0x38};
    uint8_t transfert_key[16] = {0x34, 0xc3, 0x46, 0x6e, 0xd8, 0x8f, 0x4e, 0x8e, 0x16, 0xaa, 0x47, 0x39, 0x49, 0x88, 0x43, 0x73};

    uint16_t computeCrc(uint8_t data, uint16_t crc = 0) {
        crc ^= data;
        for (int i = 0; i < 8; ++i) {
            unsigned int remainder = (crc & 1) ? CRC_POLYNOMIAL_CCITT : 0;
            crc = (crc >> 1) ^ remainder;
        }
        return crc;
    }

    /**
    Returns the CRC value for the given data frame.
    Used for whole io-homecontrol frames integrity check
    */
    uint16_t radioPacketComputeCrc(uint8_t *buffer, uint8_t bufferLength) {
        uint16_t crc = 0;

        for (uint8_t i=0; i<bufferLength; i++)
            crc = computeCrc(buffer[i], crc);

        return crc;
    }

    /**
    Returns the CRC value for the given data frame.
    Used for whole io-homecontrol frames integrity check
    */
    uint16_t radioPacketComputeCrc(std::vector<uint8_t>& buffer) {
        uint16_t crc = 0;

        for (uint8_t buf : buffer)
            crc = computeCrc(buf, crc);

        return crc;
    }

    std::tuple<uint8_t, uint8_t> computeChecksum(uint8_t frame_byte, uint8_t chksum1, uint8_t chksum2) {
        uint8_t tmpchksum = frame_byte ^ chksum2;
        chksum2 = ((chksum1 & 0x7f)<<1) & 0xff;
        if (tmpchksum >= 0x80)
            chksum2 |= 1;

        if ((chksum1 & 0x80) == 0)
            return std::make_tuple(chksum2, (tmpchksum<<1)&0xff);

        return std::make_tuple(chksum2^0x55, ((tmpchksum<<1)^0x5b)&0xff);
    }

    std::vector<uint8_t> constructInitialValue(const std::vector<uint8_t>& frame_data, const uint8_t *challenge = nullptr, const uint8_t *sequence_number = nullptr) {
        if (!challenge && !sequence_number) {
            ets_printf("Cannot create initial value: no mode selected\n");
            return {};
        }

        std::vector<uint8_t> initial_value(16, 0);
        initial_value[8] = 0;
        initial_value[9] = 0;
        size_t i = 0;
        while (i < frame_data.size()) {
            std::tie(initial_value[8], initial_value[9]) = computeChecksum(frame_data[i], initial_value[8], initial_value[9]);
            if (i < 8)
                initial_value[i] = frame_data[i];
            i++;
        }

        if (i < 8)
            for (size_t j = i; j < 8; j++)
                initial_value[j] = 0x55;

        if (!challenge && sequence_number) {
            for (i = 12; i < 16; i++)
                initial_value[i] = 0x55;

            initial_value[10] = sequence_number[0];
            initial_value[11] = sequence_number[1];
        }
        else if (challenge) {
            for (i = 10; i < 16; i++)
                initial_value[i] = challenge[i - 10];
        }

        return initial_value;
    }

/**
    Calculate HMAC using as input:
    - Packet Sequence Number
    - Controller key in clear
    - frame data starting from Command byte
*/
    void create_1W_hmac(uint8_t *hmac, const uint8_t *seq_number, uint8_t *controller_key, const std::vector<uint8_t>& frame_data) {
        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        std::vector<uint8_t> iv = constructInitialValue(frame_data, nullptr, seq_number);
        mbedtls_aes_setkey_enc( &aes, controller_key, 128 );
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, iv.data(), hmac);
        mbedtls_aes_free(&aes);
    }

/**
    Encrypt (or decrypt if called with encrypted) the transmitted key using as input:
    - Node address
    - Key in clear (or encrypted to decrypt)
*/
    void encrypt_1W_key(const uint8_t *node_address, uint8_t *key) {
        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);

        uint8_t captured[16] = {};

        std::vector<uint8_t> iv(16, 0);
        for (int i = 0; i < 13; i += 3) {
            iv[i] = node_address[0];
            iv[i + 1] = node_address[1];
            iv[i + 2] = node_address[2];
        }
        iv[15] = node_address[0];

        size_t iv_offset = 0;
        mbedtls_aes_setkey_enc( &aes, transfert_key, 128 );
        mbedtls_aes_crypt_cfb128(&aes, MBEDTLS_AES_ENCRYPT, 16, &iv_offset, iv.data(), (uint8_t *)key, captured);
        for (int i = 0; i < 16; ++i)
            key[i] = captured[i];
        mbedtls_aes_free( &aes );
    }

    /**
     * @brief Encrypts a payload for 2-Way communication using AES/ECB.
     * This function constructs an initial value (IV) based on frame data and a challenge,
     * then encrypts the IV using the provided key.
     * @param frame_data Data to build the IV from.
     * @param challenge 6-byte challenge from the other device.
     * @param key 16-byte encryption key.
     * @return The 16-byte encrypted payload.
     */
    std::vector<uint8_t> encrypt_2W_payload(const std::vector<uint8_t>& frame_data, const std::vector<uint8_t>& challenge, const uint8_t* key) {
        std::vector<uint8_t> iv = constructInitialValue(frame_data, challenge.data(), nullptr);
        std::vector<uint8_t> encrypted_payload(16);

        mbedtls_aes_context aes_ctx;
        mbedtls_aes_init(&aes_ctx);

        mbedtls_aes_setkey_enc(&aes_ctx, key, 128);
        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, iv.data(), encrypted_payload.data());

        mbedtls_aes_free(&aes_ctx);
        return encrypted_payload;
    }

    /**
     * @brief Decrypts a 16-byte payload from 2-Way communication using AES/ECB.
     * @param encrypted_payload The 16-byte data to decrypt.
     * @param key The 16-byte decryption key.
     * @return The 16-byte decrypted payload.
     */
    std::vector<uint8_t> decrypt_2W_payload(const std::vector<uint8_t>& encrypted_payload, const uint8_t* key) {
        std::vector<uint8_t> decrypted_payload(16);

        mbedtls_aes_context aes_ctx;
        mbedtls_aes_init(&aes_ctx);

        mbedtls_aes_setkey_dec(&aes_ctx, key, 128);
        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_DECRYPT, encrypted_payload.data(), decrypted_payload.data());

        mbedtls_aes_free(&aes_ctx);
        return decrypted_payload;
    }
}
