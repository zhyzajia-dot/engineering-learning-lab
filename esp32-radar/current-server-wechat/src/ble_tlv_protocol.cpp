/**
 * @file ble_tlv_protocol.cpp
 * @brief BLE TLV 数据帧的编码、解码和校验实现。
 *
 * 将小程序发送的二进制数据解析成命令，也负责构造设备响应帧。
 */
#include "ble_tlv_protocol.h"

namespace BleProto {//BLE协议命名空间

// 读取大端格式的16位无符号整数
static uint16_t readBe16(const uint8_t* p) {
    return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}
// 读取大端格式的16位有符号整数
static int16_t readBeI16(const uint8_t* p) {
    return static_cast<int16_t>(readBe16(p));
}
// 读取大端格式的32位无符号整数
static uint32_t readBe32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           p[3];
}

FrameParser::FrameParser() {}

void FrameParser::reset() {
    buffer.clear();
}

// 输入数据并尝试解析出一帧
bool FrameParser::input(const uint8_t* bytes, size_t len, Frame& outFrame) {
    if (bytes != nullptr && len > 0) {
        buffer.insert(buffer.end(), bytes, bytes + len);//将输入的字节追加到缓冲区中
    }
    return tryParseOne(outFrame);
}

// 尝试解析一帧数据
bool FrameParser::tryParseOne(Frame& outFrame) {
    while (buffer.size() >= 2) {
        if (buffer[0] == SOF1 && buffer[1] == SOF2) {
            break;
        }
        buffer.erase(buffer.begin());//如果前两个字节不是帧头，就丢弃第一个字节，继续检查下一个位置
    }

    if (buffer.size() < 9) {
        return false;//最小帧长度为9字节（2字节帧头 + 1字节版本 + 1字节命令 + 1字节序列号 + 2字节数据长度 + 0字节数据 + 2字节CRC）
    }

    uint16_t dataLen = readBe16(&buffer[5]);//从缓冲区中读取数据长度字段（大端格式）
    size_t fullLen = 2 + 1 + 1 + 1 + 2 + dataLen + 2;
    if (buffer.size() < fullLen) {
        return false;//如果缓冲区中的数据不足以构成完整的一帧，就等待更多数据的输入
    }

    uint16_t expectedCrc = readBe16(&buffer[fullLen - 2]);//从缓冲区中读取CRC字段（大端格式）
    uint16_t actualCrc = crc16Ccitt(&buffer[2], fullLen - 4);//计算从版本字段开始到数据字段结束的所有字节的CRC16-CCITT校验和
    if (expectedCrc != actualCrc) {
        buffer.erase(buffer.begin());
        return false;
    }

    outFrame.version = buffer[2];//版本字段位于缓冲区的第3个字节（索引2）
    outFrame.cmd = buffer[3];//命令字段位于缓冲区的第4个字节（索引3）
    outFrame.seq = buffer[4];//序列号字段位于缓冲区的第5个字节（索引4）
    outFrame.data.assign(buffer.begin() + 7, buffer.begin() + 7 + dataLen);//数据字段从缓冲区的第8个字节开始（索引7），长度为dataLen字节

    buffer.erase(buffer.begin(), buffer.begin() + fullLen);
    return true;
}

uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
// 以下是一些辅助函数，用于构建帧数据
void appendU8(std::vector<uint8_t>& out, uint8_t value) {
    out.push_back(value);//添加8位无符号整数到输出向量中
}

// 添加16位无符号整数到输出向量中（大端格式）
void appendU16(std::vector<uint8_t>& out, uint16_t value) {
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));//添加16位无符号整数的高8位
    out.push_back(static_cast<uint8_t>(value & 0xFF));//添加16位无符号整数的低8位
}

void appendI16(std::vector<uint8_t>& out, int16_t value) {
    appendU16(out, static_cast<uint16_t>(value));//将16位有符号整数转换为无符号整数后添加到输出向量中
}

void appendU32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(value & 0xFF));
}

void appendU64(std::vector<uint8_t>& out, uint64_t value) {
    out.push_back(static_cast<uint8_t>((value >> 56) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 48) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 40) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 32) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(value & 0xFF));
}

// 添加字节数组到输出向量中
void appendBytes(std::vector<uint8_t>& out, const uint8_t* data, size_t len) {
    out.insert(out.end(), data, data + len);
}

// 添加字符串到输出向量中
void appendString(std::vector<uint8_t>& out, const String& s) {
    appendBytes(out, reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
}

// 以下是一些函数，用于添加TLV格式的数据到输出向量中
void appendTlvU8(std::vector<uint8_t>& out, uint8_t type, uint8_t value) {
    out.push_back(type);//添加TLV类型
    appendU16(out, 1);//添加TLV长度（1字节）
    out.push_back(value);//添加TLV值（8位无符号整数）
}

void appendTlvI8(std::vector<uint8_t>& out, uint8_t type, int8_t value) {
    out.push_back(type);//添加TLV类型
    appendU16(out, 1);//添加TLV长度（1字节）
    out.push_back(static_cast<uint8_t>(value));//添加TLV值（8位有符号整数，底层补码相同）
}

void appendTlvU16(std::vector<uint8_t>& out, uint8_t type, uint16_t value) {
    out.push_back(type);//添加TLV类型
    appendU16(out, 2);//添加TLV长度（2字节）
    appendU16(out, value);//添加TLV值（16位无符号整数）
}

void appendTlvI16(std::vector<uint8_t>& out, uint8_t type, int16_t value) {
    out.push_back(type);
    appendU16(out, 2);//添加TLV长度（2字节）
    appendI16(out, value);
}

void appendTlvU32(std::vector<uint8_t>& out, uint8_t type, uint32_t value) {
    out.push_back(type);
    appendU16(out, 4);//添加TLV长度（4字节）
    appendU32(out, value);
}

void appendTlvU64(std::vector<uint8_t>& out, uint8_t type, uint64_t value) {
    out.push_back(type);
    appendU16(out, 8);//添加TLV长度（8字节）
    appendU64(out, value);
}

void appendTlvString(std::vector<uint8_t>& out, uint8_t type, const String& value) {
    out.push_back(type);//添加TLV类型
    appendU16(out, static_cast<uint16_t>(value.length()));//添加TLV长度（字符串长度）
    appendString(out, value);
}

void appendTlvBlock(std::vector<uint8_t>& out, uint8_t type, const std::vector<uint8_t>& value) {
    out.push_back(type);
    appendU16(out, static_cast<uint16_t>(value.size()));//添加TLV长度（块数据长度）
    out.insert(out.end(), value.begin(), value.end());//添加TLV值（块数据）
}

// 从数据中读取一个TLV块
bool readTlv(const std::vector<uint8_t>& data, size_t& offset, uint8_t& type, uint16_t& len, const uint8_t*& value) {
    if (offset + 3 > data.size()) {
        return false;//如果剩下的数据不足 3 字节，说明连头部都读不完，直接报错返回
    }

    type = data[offset++];//读取TLV类型
    len = readBe16(&data[offset]);//读取TLV长度（16位，两字节）
    offset += 2;

    if (offset + len > data.size()) {//如果剩下的数据不足TLV值的长度，说明数据不完整，直接报错返回
        return false;//如果剩下的数据不足，说明数据不完整，直接报错返回
    }

    value = &data[offset];//读取TLV值的指针
    offset += len;//移动偏移量到下一个TLV块的开始位置
    return true;
}

// 从结构化数据（Frame）到二进制字节流（Vector）的转换，也就是协议的“封包”过程
std::vector<uint8_t> encodeFrame(const Frame& frame) {
    std::vector<uint8_t> out;//输出字节向量，用于存储编码后的帧数据
    out.push_back(SOF1);
    out.push_back(SOF2);
    out.push_back(frame.version);//添加协议版本
    out.push_back(frame.cmd);//添加命令
    out.push_back(frame.seq);//添加序列号
    appendU16(out, static_cast<uint16_t>(frame.data.size()));//添加数据长度
    out.insert(out.end(), frame.data.begin(), frame.data.end());//添加数据部分

    uint16_t crc = crc16Ccitt(&out[2], out.size() - 2);//计算CRC16校验码，校验范围是从版本号开始到数据末尾
    appendU16(out, crc);//添加CRC16校验码
    return out;
}

// 将字节数组转换为字符串，假设字节数组是以UTF-8编码的文本数据
static String bytesToString(const uint8_t* data, uint16_t len) {
    String out;//输出字符串对象
    out.reserve(len);//预先分配足够的内存以提高性能
    for (uint16_t i = 0; i < len; ++i) {
        out += static_cast<char>(data[i]);
    }
    return out;
  }
}
