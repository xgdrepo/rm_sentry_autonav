#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdint>

// 方法1: 使用联合体（union）
union FloatToBytes {
    float f;
    uint8_t bytes[4];
};

void floatToHexLE(float value) {
    FloatToBytes converter;
    converter.f = value;
    
    std::cout << "Float: " << value << std::endl;
    std::cout << "Hex (LE): ";
    for (int i = 0; i < 4; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(converter.bytes[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}
int main() {
    // 原始数据
    float x = -0.8240f;
	floatToHexLE(x);

    return 0;
}