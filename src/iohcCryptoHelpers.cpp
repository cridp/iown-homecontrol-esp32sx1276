#include <iohcCryptoHelpers.h>


std::string transfer_key = "34c3466ed88f4e8e16aa473949884373";


AES128 aes128;
CTR<AES128> ctraes128;


uint8_t hexStringToBytes(const std::string hexString, uint8_t *byteString)
{
    uint8_t i;

    if (hexString.length() % 2 != 0)
        return 0;

    for (i = 0; i < hexString.length(); i += 2)
        byteString[i/2] = (char) strtol(hexString.substr(i, 2).c_str(), NULL, 16);

    return i/2;
}

std::tuple<uint8_t, uint8_t> computeChecksum(uint8_t frame_byte, uint8_t chksum1, uint8_t chksum2)
{
    uint8_t tmpchksum = frame_byte ^ chksum2;
    chksum2 = ((chksum1 & 0x7f)<<1) & 0xff;
    if (tmpchksum >= 0x80)
        chksum2 |= 1;

    if ((chksum1 & 0x80) == 0)
        return std::make_tuple(chksum2, (tmpchksum<<1)&0xff);
    else
        return std::make_tuple(chksum2^0x55, ((tmpchksum<<1)^0x5b)&0xff);
}

std::vector<uint8_t> constructInitialValue(const std::vector<uint8_t>& frame_data, const uint8_t *challenge = nullptr, const uint8_t *sequence_number = nullptr)
{
    if (!challenge && !sequence_number)
    {
        Serial.printf("Cannot create initial value: no mode selected\n");
        return {};
    }

    std::vector<uint8_t> initial_value(16, 0);
    initial_value[8] = 0;
    initial_value[9] = 0;
    size_t i = 0;
    while (i < frame_data.size())
    {
        std::tie(initial_value[8], initial_value[9]) = computeChecksum(frame_data[i], initial_value[8], initial_value[9]);
        if (i < 8)
            initial_value[i] = frame_data[i];
        i++;
    }

    if (i < 8)
        for (size_t j = i; j < 8; j++)
            initial_value[j] = 0x55;

    if (!challenge && sequence_number)
    {
        for (size_t i = 12; i < 16; i++)
            initial_value[i] = 0x55;

        initial_value[10] = sequence_number[0];
        initial_value[11] = sequence_number[1];
    }
    else if (challenge)
    {
        for (size_t i = 10; i < 16; i++)
            initial_value[i] = challenge[i - 10];
    }

    return initial_value;
}

void create_1W_hmac(uint8_t *hmac, const uint8_t *seq_number, uint8_t *controller_key, const std::vector<uint8_t>& frame_data) // Ok!!!
{
    std::vector<uint8_t> iv;

    iv = constructInitialValue(frame_data, nullptr, seq_number);

    aes128.setKey(controller_key, 16);
    aes128.encryptBlock(hmac, iv.data());
}

void encrypt_1W_key(const uint8_t *node_address, uint8_t *key) // Ok!!!
{
    uint8_t btransfer[16];

    hexStringToBytes(transfer_key, btransfer);

    std::vector<uint8_t> iv(16, 0);
    for (int i = 0; i < 13; i += 3)
    {
        iv[i] = node_address[0];
        iv[i + 1] = node_address[1];
        iv[i + 2] = node_address[2];
    }
    iv[15] = node_address[0];

/*  
    uint8_t captured[16] = {0};
    aes128.setKey((uint8_t *)btransfer, 16);
    aes128.encryptBlock(captured, iv.data());

    for (int i = 0; i < 16; ++i)
        key[i] ^= captured[i];
*/
//  Use CTR encryption instead of AES. It includes xor with the key
    ctraes128.setKey(btransfer, 16);
    ctraes128.setIV(iv.data(), 16);
    ctraes128.setCounterSize(4);
    ctraes128.encrypt(key, key, 16);
}
