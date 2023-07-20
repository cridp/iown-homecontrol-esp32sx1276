#pragma once

#include <Arduino.h>
#include <string>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
//#include <CBC.h>


extern uint8_t hexStringToBytes(const std::string hexString, uint8_t *byteString);
extern void encrypt_1W_key(const uint8_t *node_address, uint8_t *key);
extern void create_1W_hmac(uint8_t *hmac, const uint8_t *seq_number, uint8_t *controller_key, const std::vector<uint8_t>& frame_data);