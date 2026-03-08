#include <iostream>
#include <cstdint>
#include <cstring>

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int main() {
    // x86是小端序：低位在前，高位在后
    // linear_x = -0.78 = 0xBF47AE14 (十六进制)
    // 小端序存储：0x14 0xAE 0x47 0xBF
    
    // linear_y = 1.23 = 0x3F9D70A4
    // 小端序存储：0xA4 0x70 0x9D 0x3F
  //FA FB 21 0D 7E 80 69 3E 62 F2 FA BF 00 00 00 00 00 0F 41

uint8_t data[] = {
    
    // CMD_ID 和 数据长度\x41\x02\x8B\x01\x5D\xDB
    0x41, 0x02,
    
    // 血量85 (小端序)
    0x8B, 0x01,
};
    
    size_t data_len = 4; // 2 + 4 + 4 + 4 + 1 = 15字节
    
    // 计算CRC
    uint16_t crc = crc16_ccitt(data, data_len);
    uint16_t crc_le = (crc & 0xFF) << 8 | (crc >> 8); // 转为小端
    
    std::cout << "\n计算结果：" << std::endl;
    printf("计算的大端CRC: 0x%04X\n", crc);

    return 0;
}