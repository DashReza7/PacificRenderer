#include "zlib.h"
#include <fstream>
#include <iostream>
#include <vector>


uint16_t read_u16_le(std::istream &stream) {
    unsigned char bytes[2];
    stream.read(reinterpret_cast<char *>(bytes), 2);
    if (stream.gcount() != 2)
        throw std::runtime_error("Unexpected EOF or read error\n");
    return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}
uint32_t read_u32_le(std::istream &stream) {
    unsigned char bytes[4];
    stream.read(reinterpret_cast<char *>(bytes), 4);
    if (stream.gcount() != 4)
        throw std::runtime_error("Unexpected EOF or read error\n");
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8) |
           (static_cast<uint32_t>(bytes[2]) << 16) |
           (static_cast<uint32_t>(bytes[3]) << 24);
}
uint64_t read_u64_le(std::istream &stream) {
    unsigned char bytes[8];
    stream.read(reinterpret_cast<char *>(bytes), 8);
    if (stream.gcount() != 8)
        throw std::runtime_error("Unexpected EOF or read error\n");
    return static_cast<uint64_t>(bytes[0]) |
           (static_cast<uint64_t>(bytes[1]) << 8) |
           (static_cast<uint64_t>(bytes[2]) << 16) |
           (static_cast<uint64_t>(bytes[3]) << 24) |
           (static_cast<uint64_t>(bytes[4]) << 32) |
           (static_cast<uint64_t>(bytes[5]) << 40) |
           (static_cast<uint64_t>(bytes[6]) << 48) |
           (static_cast<uint64_t>(bytes[7]) << 56);
}
uint32_t read_u32_le_buffer(const uint8_t *buf, size_t &offset) {
    uint32_t result = static_cast<uint32_t>(buf[offset]) |
                      (static_cast<uint32_t>(buf[offset+1]) << 8) |
                      (static_cast<uint32_t>(buf[offset+2]) << 16) |
                      (static_cast<uint32_t>(buf[offset+3]) << 24);
    offset += 4;
    return result;
}
uint64_t read_u64_le_buffer(const uint8_t *buf, size_t &offset) {
    uint64_t result = static_cast<uint64_t>(buf[offset]) |
                      (static_cast<uint64_t>(buf[offset+1]) << 8) |
                      (static_cast<uint64_t>(buf[offset+2]) << 16) |
                      (static_cast<uint64_t>(buf[offset+3]) << 24) |
                      (static_cast<uint64_t>(buf[offset+4]) << 32) |
                      (static_cast<uint64_t>(buf[offset+5]) << 40) |
                      (static_cast<uint64_t>(buf[offset+6]) << 48) |
                      (static_cast<uint64_t>(buf[offset+7]) << 56);
    offset += 8;
    return result;
}
float read_float_le_buffer(const uint8_t *buf, size_t &offset) {
    uint32_t int_repr = read_u32_le_buffer(buf, offset);
    float result;
    std::memcpy(&result, &int_repr, sizeof(float));
    return result;
}
double read_double_le_buffer(const uint8_t *buf, size_t &offset) {
    uint64_t int_repr = read_u64_le_buffer(buf, offset);
    double result;
    std::memcpy(&result, &int_repr, sizeof(double));
    return result;
}
std::string read_utf8_string(const uint8_t* ptr, size_t &offset) {
    const char* start = reinterpret_cast<const char*>(ptr + offset);
    std::string result(start);
    offset += result.size() + 1;
    return result;
}
