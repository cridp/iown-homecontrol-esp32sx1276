#ifndef CRYPTO_H
#define CRYPTO_H

//#include <Arduino.h>
#include <board-config.h>
#include <string>
#include <vector>
#include <tuple>

#if defined(ESP8266)
    #include <Crypto.h>
    #include <AES.h>
    #include <CTR.h>
#elif defined(HELTEC)
    #include "mbedtls/aes.h"        // AES functions
#endif

#define CRC_POLYNOMIAL_CCITT    0x8408

uint8_t hexStringToBytes(std::string hexString, uint8_t *byteString);
std::string bytesToHexString(const uint8_t *byteString, uint8_t len);

namespace iohcCrypto {
    uint16_t computeCrc(uint8_t data, uint16_t crc);
    uint16_t radioPacketComputeCrc(uint8_t *buffer, uint8_t bufferLength);
    uint16_t radioPacketComputeCrc(std::vector<uint8_t>& buffer);
    void encrypt_1W_key(const uint8_t *node_address, uint8_t *key);
    void create_1W_hmac(uint8_t *hmac, const uint8_t *seq_number, uint8_t *controller_key, const std::vector<uint8_t>& frame_data);
}
#endif