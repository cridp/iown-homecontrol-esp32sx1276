#include <iohcPacket.h>
#include <cstdio>
#include <utils.h>

namespace IOHC {
    void IRAM_ATTR iohcPacket::decode(bool verbosity) {
        //        printf("%lu %lu +%lu\n", packetStamp, relStamp, packetStamp - relStamp);

        if (packetStamp - relStamp > 500000L) {
            printf("\n");
            relStamp = packetStamp; // - this->relStamp;
            for (uint8_t i = 0; i < 3; i++)
                source_originator[i] = this->payload.packet.header.source[i];
        }
        char _dir[3] = {};
        if (!memcmp(source_originator, this->payload.packet.header.source, 3))
            _dir[0] = '>';
        else
            _dir[0] = '<';

        printf("(%2.2u) %1xW S %s E %s ", this->payload.packet.header.CtrlByte1.asStruct.MsgLen,
               this->payload.packet.header.CtrlByte1.asStruct.Protocol ? 1 : 2,
               this->payload.packet.header.CtrlByte1.asStruct.StartFrame ? "1" : "0",
               this->payload.packet.header.CtrlByte1.asStruct.EndFrame ? "1" : "0");

        if (this->payload.packet.header.CtrlByte2.asStruct.LPM) printf("[LPM]");
        if (this->payload.packet.header.CtrlByte2.asStruct.Beacon) printf("[B]");
        if (this->payload.packet.header.CtrlByte2.asStruct.Routed) printf("[R]");
        if (this->payload.packet.header.CtrlByte2.asStruct.Prio) printf("[PRIO]");
        if (this->payload.packet.header.CtrlByte2.asStruct.Unk2) printf("[U2]");
        if (this->payload.packet.header.CtrlByte2.asStruct.Unk3) printf("[U3]");
        if (this->payload.packet.header.CtrlByte2.asStruct.Version) printf(
            "[V]%u", this->payload.packet.header.CtrlByte2.asStruct.Version);

        //            const char *commandName = commands[msg_cmd_id].c_str();
        //            Serial.print(commandName);

        printf("\tFROM %2.2X%2.2X%2.2X TO %2.2X%2.2X%2.2X CMD %2.2X",
               this->payload.packet.header.source[0], this->payload.packet.header.source[1],
               this->payload.packet.header.source[2],
               this->payload.packet.header.target[0], this->payload.packet.header.target[1],
               this->payload.packet.header.target[2],
               this->payload.packet.header.cmd);
        //        printf(" +%03.3f F%03.3f, %03.1fdBm %f %f\t", static_cast<float>(packetStamp - relStamp)/1000.0, static_cast<float>(this->frequency)/1000000.0, this->rssi, this->lna, this->afc);
        printf(" %s ", _dir);

        uint8_t dataLen = this->buffer_length - 9;
        printf(" DATA(%2.2u) ", dataLen);

        // 1W fields
        if (this->payload.packet.header.CtrlByte1.asStruct.Protocol) {
            unsigned data_length = dataLen - 8;

            std::string msg_data = bitrow_to_hex_string(this->payload.buffer + 9, dataLen/*data_length*/);
            printf(" %s", msg_data.c_str());

            switch (this->payload.packet.header.cmd) {
                case 0x30: {
                    printf("\tKEY %s SEQ %s ", bitrow_to_hex_string(this->payload.packet.msg.p0x30.enc_key, 16).c_str(), bitrow_to_hex_string(this->payload.packet.msg.p0x30.sequence, 2).c_str());
                    printf("Manuf %X Data %X ", this->payload.packet.msg.p0x30.man_id, this->payload.packet.msg.p0x30.data);
                    break;
                }
                case 0x2E:
                case 0x39: {
                    printf("\tSEQ %s MAC %s ", bitrow_to_hex_string(this->payload.packet.msg.p0x2e.sequence, 2).c_str(), bitrow_to_hex_string(this->payload.packet.msg.p0x2e.hmac, 6).c_str());
                    printf("Data %X ", this->payload.packet.msg.p0x2e.data);
                    break;
                }
                case 0x28: {
                }
                case 0x01:
                case 0x00: {
                    // auto main = static_cast<unsigned>((this->payload.packet.msg.p0x00.main[0] << 8) | this->payload.packet.msg.p0x00.main[1]);
                    // printf("Org %X Acei %X Main %X fp1 %X fp2 %X ", this->payload.packet.msg.p0x00.origin, this->payload.packet.msg.p0x00.acei, main, this->payload.packet.msg.p0x00.fp1, this->payload.packet.msg.p0x00.fp2);
                    // printf("Seq %s Hmac %s", bitrow_to_hex_string(this->payload.packet.msg.p0x00.sequence, 2).c_str(), bitrow_to_hex_string(this->payload.packet.msg.p0x00.hmac, 6).c_str());
                    int msg_seq_nr = 0;
                    msg_seq_nr = (this->payload.buffer[9 + data_length] << 8) | this->payload.buffer[9 + data_length + 1];
                    printf("\tSEQ %3.2X", msg_seq_nr);
                    std::string msg_mac = bitrow_to_hex_string(this->payload.buffer + 9 + data_length + 2, 6);
                    printf(" MAC %s ", msg_mac.c_str());
                    // if (data_length == 16)
                    //     printf("\tSEQ %s MAC %s ", bitrow_to_hex_string(this->payload.packet.msg.p0x00.sequence, 2).c_str(), bitrow_to_hex_string(this->payload.packet.msg.p0x00.hmac, 6).c_str());
                    // else
                    //     printf("\tSEQ %s MAC %s ", bitrow_to_hex_string(this->payload.packet.msg.p0x00_all.sequence, 2).c_str(), bitrow_to_hex_string(this->payload.packet.msg.p0x00_all.hmac, 6).c_str());
                } break;
                default: {
                    // std::string msg_data = bitrow_to_hex_string(this->payload.buffer + 9, data_length);
                    // printf(" %s", msg_data.c_str());

                    // int msg_seq_nr = 0;
                    // msg_seq_nr = (this->payload.buffer[9 + data_length] << 8) | this->payload.buffer[9 + data_length + 1];
                    // printf("\tSEQ %3.2X", msg_seq_nr);

                    // std::string msg_mac = bitrow_to_hex_string(this->payload.buffer + 9 + data_length + 2, 6);
                    // printf(" MAC %s ", msg_mac.c_str());

                    // printf("\tSEQ %s MAC %s ", bitrow_to_hex_string(this->payload.packet.msg.p0x00.sequence, 2).c_str(), bitrow_to_hex_string(this->payload.packet.msg.p0x00.hmac, 6).c_str());
                }

            }
            uint16_t broadcast = ((this->payload.packet.header.target[1]) << 2) | (
                                     (this->payload.packet.header.target[2] >> 6) & 0x03);
            const char* typeName = sDevicesType[broadcast].c_str();
            printf(" Type %s ", typeName);
        }
        // 2W fields
        else {
            if (dataLen != 0) {
                std::string msg_data = bitrow_to_hex_string(this->payload.buffer + 9, dataLen);
                printf(" %s", msg_data.c_str());
            }
        }
        /* CLOSE GARAGE DOOR
08:11:22.546 > (21) 1W S 1 E 1  FROM AD5F00 TO 00003F CMD 01 >  DATA(13)  0143 00019a    SEQ 214B MAC 452e04769e36  Type All  Org 1 Acei 43 Main 1 fp1 9A fp2 21  Acei 3 0 2 0
08:11:22.700 > (24) 1W S 1 E 1  FROM AD5F00 TO 00003F CMD 20 >  DATA(16)  02ff 0143 0b05ff00      SEQ 214C MAC 500aa91527c6  Type All
        */
        /* PLAYING LIGHT
 08:10:13.647 > (24) 1W S 1 E 1  FROM 28844E TO 0001BF CMD 00 <  DATA(16)  0143000080c80000      SEQ 1233 MAC cbdca06accb8  Type Light  Org 1 Acei 43 Main 0 fp1 80 fp2 C8  Acei 3 0 2 0
 08:10:33.096 > (24) 1W S 1 E 1  FROM 78F37B TO 0001BF CMD 00 >  DATA(16)  0143000080d30000      SEQ 6D5 MAC a5985a832ae1  Type Light  Org 1 Acei 43 Main 0 fp1 80 fp2 D3  Acei 3 0 2 0
         */
        /*TODO 1W ACEI ERROR
16:10:27.537 > (21) 1W S 1 E 1  FROM 9A5CA0 TO 00003F CMD 01 <  DATA(13)  0143 0001b6    SEQ 1B46 MAC c5a7a90d290b  Type All  Org 1 Acei 43 Main 1 fp1 B6 fp2 1B  Acei 3 0 2 0
16:10:27.594 > (24) 1W S 1 E 1  FROM 9A5CA0 TO 00003F CMD 20 <  DATA(16)  02ff 0143 0b05ff00      SEQ 1B47 MAC 5be27b33d3c2  Type All
        */
        if (this->payload.packet.header.cmd == 0x00 || this->payload.packet.header.cmd == 0x01) {
            auto main = static_cast<unsigned>((this->payload.packet.msg.p0x00.main[0] << 8) | this->payload.packet.msg.p0x00.main[1]);
            printf(" Org %X Acei %X Main %X fp1 %X fp2 %X ", this->payload.packet.msg.p0x00.origin,
                   this->payload.packet.msg.p0x00.acei.asByte, main, this->payload.packet.msg.p0x00.fp1,
                   this->payload.packet.msg.p0x00.fp2);

            auto acei = this->payload.packet.msg.p0x00.acei;
            printf(" Acei %u %u %u %u ", acei.asStruct.level, acei.asStruct.service, acei.asStruct.extended,
                   acei.asStruct.isvalid);

            //            std::bitset<8> bits(acei.asByte);
            //            std::cout << "Acei " << bits[7] << " " << bits[6] << " " << bits[5] << " " << bits[4] << " " << bits[3] << " " << bits[2] << " " << bits[1] << " " << bits[0] << std::endl;
        }

        printf("\n");

        relStamp = packetStamp;
    }
}
