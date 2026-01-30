#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdint>
#include <sstream>

// 将浮点数转换为十六进制字符串
std::string floatToHex(float value) {
    uint32_t intValue;
    std::memcpy(&intValue, &value, sizeof(float));
    
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(8) << intValue;
    return ss.str();
}

// 将整数转换为十六进制字符串
std::string intToHex(int32_t value) {
    uint32_t unsignedValue = static_cast<uint32_t>(value);
    
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(8) << unsignedValue;
    return ss.str();
}

// 将浮点数转换为十六进制（小端序字节数组）
void floatToHexBytes(float value, uint8_t* bytes) {
    uint32_t intValue;
    std::memcpy(&intValue, &value, sizeof(float));
    
    // 转换为小端序字节
    for (int i = 0; i < 4; i++) {
        bytes[i] = (intValue >> (i * 8)) & 0xFF;
    }
}

// 将整数转换为十六进制（小端序字节数组）
void intToHexBytes(int32_t value, uint8_t* bytes) {
    uint32_t unsignedValue = static_cast<uint32_t>(value);
    
    // 转换为小端序字节
    for (int i = 0; i < 4; i++) {
        bytes[i] = (unsignedValue >> (i * 8)) & 0xFF;
    }
}

int main() {
    // 原始数据
    float linear_x = 0.2030f;
    float linear_y = -0.7200f;
    float angular_z = 0.0000f;
    int32_t spin_mode = 0;
    
    std::cout << "原始值和对应的十六进制：" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    // 转换为十六进制字符串
    std::string linear_x_hex = floatToHex(linear_x);
    std::string linear_y_hex = floatToHex(linear_y);
    std::string angular_z_hex = floatToHex(angular_z);
    std::string spin_mode_hex = intToHex(spin_mode);
    
    std::cout << "linear_x = " << linear_x << "  -> 0x" << linear_x_hex << std::endl;
    std::cout << "linear_y = " << linear_y << "  -> 0x" << linear_y_hex << std::endl;
    std::cout << "angular_z = " << angular_z << "  -> 0x" << angular_z_hex << std::endl;
    std::cout << "spin_mode = " << spin_mode << "  -> 0x" << spin_mode_hex << std::endl;
    

    return 0;
}