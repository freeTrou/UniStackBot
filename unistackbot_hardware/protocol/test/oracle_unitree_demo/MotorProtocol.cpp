#include "MotorProtocol.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iostream>

/**
 * CRC32查找表
 * 用于快速计算CRC32校验值
 * 与Python版本的表完全一致
 * 共256个元素，每个元素32位
 */
const uint32_t MotorProtocol::CRC32_TABLE[256] = {
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
    0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
    0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
    0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
    0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
    0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
    0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
    0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
    0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
    0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
    0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
    0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
    0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
    0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
    0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
    0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
    0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
    0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
    0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
    0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
    0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};

MotorProtocol::MotorProtocol() {
}

/**
 * @brief 构建控制包（20字节）
 *
 * 数据包结构：
 * Byte 0-1:   包头 (0xFE 0xEE)
 * Byte 2:     模式字节 (bit0-3:ID, bit4-6:mode, bit7:timeout)
 * Byte 3:     保留字节
 * Byte 4-5:   扭矩 (int16, 小端)
 * Byte 6-7:   速度 (int16, 小端)
 * Byte 8-11:  位置 (int32, 小端)
 * Byte 12-13: Kp (int16, 小端)
 * Byte 14-15: Kd (int16, 小端)
 * Byte 16-19: CRC32校验 (包含包头)
 *
 * 转换公式：
 * - 扭矩: tor_des = round(torque / RATIO * 2560)
 * - 速度: spd_des = round(speed * RATIO * 64 / PI / 2)
 * - 位置: pos_des = round(position * RATIO * 32768 / PI / 2)
 * - Kp:   k_pos = round(Kp / (RATIO^2) * 12800)
 * - Kd:   k_spd = round(Kd / (RATIO^2) * 51200)
 */
std::vector<uint8_t> MotorProtocol::buildControlPacket(
    uint8_t motor_id, uint8_t mode, bool timeout,
    float torque, float speed, float position,
    float kp, float kd) {

    // ===== 第1步：参数范围限制 =====
    // 确保所有参数在协议允许的范围内
    motor_id = std::min(std::max(motor_id, (uint8_t)0), (uint8_t)15);
    mode = std::min(std::max(mode, (uint8_t)0), (uint8_t)1);
    uint8_t timeout_bit = timeout ? 1 : 0;

    // 物理量范围限制
    kp = clamp(kp, 0.0f, 410.725243f);
    kd = clamp(kd, 0.0f, 102.681311f);
    torque = clamp(torque, -162.133333f, 162.128385f);
    speed = clamp(speed, -253.972964f, 253.965213f);
    position = clamp(position, -32508.539391f, 32508.539376f);

    // ===== 第2步：构建模式字节 =====
    // bit0-3: 电机ID (4位，0-15)
    // bit4-6: 工作模式 (3位，0-7)
    // bit7:   超时保护 (1位)
    uint8_t mode_byte = (motor_id & 0x0F) |          // ID占用低4位
        ((mode & 0x07) << 4) |       // 模式占用bit4-6
        ((timeout_bit & 0x01) << 7); // 超时占用bit7

    // ===== 第3步：物理量转换为协议原始值 =====
    // 使用round()四舍五入，与Python的round()行为一致

    // Kp转换：物理量 -> 协议值 (乘以12800，除以减速比平方)
    int16_t k_pos_val = static_cast<int16_t>(std::round(kp / (RATIO * RATIO) * 12800.0));

    // Kd转换：物理量 -> 协议值 (乘以51200，除以减速比平方)
    int16_t k_spd_val = static_cast<int16_t>(std::round(kd / (RATIO * RATIO) * 51200.0));

    // 位置转换：物理量(rad) -> 协议值 (乘以32768，除以PI/2，乘以减速比)
    int32_t pos_des_val = static_cast<int32_t>(std::round(position * RATIO * 32768.0 / M_PI / 2.0));

    // 速度转换：物理量(rad/s) -> 协议值 (乘以64，除以PI/2，乘以减速比)
    int16_t spd_des_val = static_cast<int16_t>(std::round(speed * RATIO * 64.0 / M_PI / 2.0));

    // 扭矩转换：物理量(Nm) -> 协议值 (乘以2560，除以减速比)
    int16_t tor_des_val = static_cast<int16_t>(std::round(torque / RATIO * 2560.0));

    // ===== 第4步：限制协议值范围 =====
    k_pos_val = clampToInt16(k_pos_val);
    k_spd_val = clampToInt16(k_spd_val);
    spd_des_val = clampToInt16(spd_des_val);
    tor_des_val = clampToInt16(tor_des_val);
    pos_des_val = clampToInt32(pos_des_val);

    // ===== 第5步：构建20字节数据包 =====
    std::vector<uint8_t> packet(20, 0);

    // 包头 (2字节)
    packet[0] = 0xFE;
    packet[1] = 0xEE;

    // 模式字节 + 保留字节 (2字节)
    packet[2] = mode_byte;
    packet[3] = 0x00;  // 保留字节，固定为0

    // 控制数据 (12字节，小端序)
    // 使用memcpy确保正确的字节序
    memcpy(&packet[4], &tor_des_val, 2);   // int16_t 扭矩，2字节
    memcpy(&packet[6], &spd_des_val, 2);   // int16_t 速度，2字节
    memcpy(&packet[8], &pos_des_val, 4);   // int32_t 位置，4字节
    memcpy(&packet[12], &k_pos_val, 2);    // int16_t Kp，2字节
    memcpy(&packet[14], &k_spd_val, 2);    // int16_t Kd，2字节

    // ===== 第6步：计算CRC32 =====
    // CRC计算包含包头在内的前16字节
    uint32_t crc = calculateCRC32(packet.data(), 16);
    memcpy(&packet[16], &crc, 4);  // 将CRC写入最后4字节

    return packet;
}

/**
 * @brief 构建清除指令（20字节）
 *
 */
std::vector<uint8_t> MotorProtocol::buildClearPacket(
    uint8_t motor_id) {

    // ===== 第1步：参数范围限制 =====
    // 确保所有参数在协议允许的范围内
    motor_id = std::min(std::max(motor_id, (uint8_t)0), (uint8_t)15);
    uint8_t mode = 6;
    uint8_t timeout_bit = 0;

    // ===== 第2步：构建模式字节 =====
    // bit0-3: 电机ID (4位，0-15)
    // bit4-6: 工作模式 (3位，0-7)
    // bit7:   超时保护 (1位)
    uint8_t mode_byte = (motor_id & 0x0F) |          // ID占用低4位
        ((mode & 0x07) << 4) |       // 模式占用bit4-6
        ((timeout_bit & 0x01) << 7); // 超时占用bit7

    int16_t k_pos_val = 0;
    int16_t k_spd_val = 0;
    int32_t pos_des_val = 0;
    int16_t spd_des_val = 0;
    int16_t tor_des_val = -256;


    // ===== 第5步：构建20字节数据包 =====
    std::vector<uint8_t> packet(20, 0);

    // 包头 (2字节)
    packet[0] = 0xFE;
    packet[1] = 0xEE;

    // 模式字节 + 保留字节 (2字节)
    packet[2] = mode_byte;
    packet[3] = 0x00;  // 保留字节，固定为0

    // 控制数据 (12字节，小端序)
    // 使用memcpy确保正确的字节序
    memcpy(&packet[4], &tor_des_val, 2);   // int16_t 扭矩，2字节
    memcpy(&packet[6], &spd_des_val, 2);   // int16_t 速度，2字节
    memcpy(&packet[8], &pos_des_val, 4);   // int32_t 位置，4字节
    memcpy(&packet[12], &k_pos_val, 2);    // int16_t Kp，2字节
    memcpy(&packet[14], &k_spd_val, 2);    // int16_t Kd，2字节

    // CRC计算包含包头在内的前16字节
    uint32_t crc = calculateCRC32(packet.data(), 16);
    memcpy(&packet[16], &crc, 4);  // 将CRC写入最后4字节

    return packet;
}

/**
 * @brief 构建复位指令（20字节）
 *
 */
std::vector<uint8_t> MotorProtocol::buildResetPacket(
    uint8_t motor_id) {

    // ===== 第1步：参数范围限制 =====
    // 确保所有参数在协议允许的范围内
    motor_id = std::min(std::max(motor_id, (uint8_t)0), (uint8_t)15);
    uint8_t mode = 7;
    uint8_t timeout_bit = 0;

    // ===== 第2步：构建模式字节 =====
    // bit0-3: 电机ID (4位，0-15)
    // bit4-6: 工作模式 (3位，0-7)
    // bit7:   超时保护 (1位)
    uint8_t mode_byte = (motor_id & 0x0F) |          // ID占用低4位
        ((mode & 0x07) << 4) |       // 模式占用bit4-6
        ((timeout_bit & 0x01) << 7); // 超时占用bit7

    int16_t k_pos_val = 0;
    int16_t k_spd_val = 0;
    int32_t pos_des_val = 0;
    int16_t spd_des_val = 0;
    int16_t tor_des_val = 0;


    // ===== 第5步：构建20字节数据包 =====
    std::vector<uint8_t> packet(20, 0);

    // 包头 (2字节)
    packet[0] = 0xFE;
    packet[1] = 0xEE;

    // 模式字节 + 保留字节 (2字节)
    packet[2] = mode_byte;
    packet[3] = 0x00;  // 保留字节，固定为0

    // 控制数据 (12字节，小端序)
    // 使用memcpy确保正确的字节序
    memcpy(&packet[4], &tor_des_val, 2);   // int16_t 扭矩，2字节
    memcpy(&packet[6], &spd_des_val, 2);   // int16_t 速度，2字节
    memcpy(&packet[8], &pos_des_val, 4);   // int32_t 位置，4字节
    memcpy(&packet[12], &k_pos_val, 2);    // int16_t Kp，2字节
    memcpy(&packet[14], &k_spd_val, 2);    // int16_t Kd，2字节

    // CRC计算包含包头在内的前16字节
    uint32_t crc = calculateCRC32(packet.data(), 16);
    memcpy(&packet[16], &crc, 4);  // 将CRC写入最后4字节

    return packet;
}

/**
 * @brief 解析反馈包（26字节）
 *
 * 反馈包结构：
 * Byte 0-1:   包头 (0xFC 0xEE) - 不参与CRC
 * Byte 2:     模式字节
 * Byte 3:     驱动温度 (int8, -100~127°C)
 * Byte 4:     绕组温度 (uint8, 0~255°C)
 * Byte 5:     电压 (原始值/2 = V)
 * Byte 6-7:   扭矩 (int16, 小端)
 * Byte 8-9:   速度 (int16, 小端)
 * Byte 10-13: 位置 (int32, 小端)
 * Byte 14-17: 错误码 (uint32)
 * Byte 18-19: 保留+警告 (bit13-15为警告码)
 * Byte 20-21: 保留
 * Byte 22-25: CRC32 (从Byte2开始计算)
 *
 * 转换公式（反向）：
 * - 扭矩: torque = tor_raw * RATIO / 2560
 * - 速度: speed = spd_raw * 2 * PI / (64 * RATIO)
 * - 位置: position = pos_raw * 2 * PI / (32768 * RATIO)
 */
std::shared_ptr<MotorFeedback> MotorProtocol::parseFeedbackPacket(const std::vector<uint8_t>& data) {
    // ===== 第1步：基础检查 =====
    // 检查数据包长度是否为26字节
    if (data.size() != 26) {
        std::cerr << "[Parse Error] Invalid packet length: " << data.size() << " (expected 26)" << std::endl;
        return nullptr;
    }

    // 检查包头是否为0xFC 0xEE
    if (data[0] != 0xFC || data[1] != 0xEE) {
        std::cerr << "[Parse Error] Invalid header" << std::endl;
        return nullptr;
    }

    // ===== 第2步：解析模式字节 =====
    uint8_t mode_byte = data[2];
    uint8_t motor_id = mode_byte & 0x0F;        // 提取低4位：电机ID
    uint8_t mode = (mode_byte >> 4) & 0x07;     // 提取bit4-6：工作模式
    uint8_t timeout = (mode_byte >> 7) & 0x01;  // 提取bit7：超时状态

    // ===== 第3步：CRC校验 =====
    // CRC从mode_byte开始，共20字节 (data[2] ~ data[21])
    uint32_t crc_received;
    memcpy(&crc_received, &data[22], 4);  // 提取收到的CRC

    uint32_t crc_computed = calculateCRC32(&data[2], 20);  // 计算CRC

    // 比较CRC，不匹配则丢弃
    if (crc_received != crc_computed) {
        std::cerr << "[CRC Error] Received: 0x" << std::hex << crc_received
            << " Computed: 0x" << crc_computed << std::dec << std::endl;
        return nullptr;
    }

    // ===== 第4步：创建反馈对象 =====
    auto feedback = std::make_shared<MotorFeedback>();
    feedback->motor_id = motor_id;
    feedback->mode = mode;
    feedback->timeout = timeout;

    // ===== 第5步：提取温度数据 =====
    feedback->temp_driver = static_cast<int8_t>(data[3]);  // 驱动温度：有符号
    feedback->temp_winding = data[4];                       // 绕组温度：无符号

    // ===== 第6步：提取电压 =====
    // 原始值除以2得到实际电压（协议规定：2表示1V）
    feedback->voltage = data[5] / 2.0f;

    // ===== 第7步：提取扭矩（2字节，int16，小端） =====
    int16_t torque_raw;
    memcpy(&torque_raw, &data[6], 2);
    // 转换为物理量：原始值 * 减速比 / 2560
    feedback->torque = float(torque_raw / 2560.0f * RATIO);

    // ===== 第8步：提取速度（2字节，int16，小端） =====
    int16_t speed_raw;
    memcpy(&speed_raw, &data[8], 2);
    // 转换为物理量：原始值 * 2 * PI / (64 * 减速比)
    feedback->speed = float((speed_raw / 64.0f) * 2.0f * M_PI / RATIO);

    // ===== 第9步：提取位置（4字节，int32，小端） =====
    int32_t pos_raw;
    memcpy(&pos_raw, &data[10], 4);
    // 转换为物理量：原始值 * 2 * PI / (32768 * 减速比)
    feedback->position = float(2.0f * M_PI * pos_raw / 32768.0f / RATIO);

    // ===== 第10步：提取错误码（4字节） =====
    memcpy(&feedback->error_code, &data[14], 4);

    // ===== 第11步：提取警告码 =====
    // 警告码在保留字段的高3位（bit13-15）
    uint16_t res_exflag;
    memcpy(&res_exflag, &data[18], 2);
    feedback->warning_code = (res_exflag >> 13) & 0x07;  // 右移13位取高3位

    return feedback;
}

/**
 * @brief CRC32查表法计算
 *
 * 算法流程：
 * 1. 初始化CRC为0xFFFFFFFF
 * 2. 每次处理4个字节（小端序：b3,b2,b1,b0）
 * 3. 对每个字节：查表 + 移位
 * 4. 返回最终CRC值
 *
 * 与Python版本的crc32_lookup函数算法完全一致
 */
uint32_t MotorProtocol::calculateCRC32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;  // 初始值
    size_t i = 0;

    // 每次处理4个字节
    while (i + 3 < len) {
        // 小端序：b0是最低字节，b3是最高字节
        uint8_t b0 = data[i + 3];  // 最高字节
        uint8_t b1 = data[i + 2];
        uint8_t b2 = data[i + 1];
        uint8_t b3 = data[i];      // 最低字节

        // 对每个字节进行CRC计算
        crc = CRC32_TABLE[(crc >> 24) ^ b0] ^ ((crc << 8) & 0xFFFFFFFF);
        crc = CRC32_TABLE[(crc >> 24) ^ b1] ^ ((crc << 8) & 0xFFFFFFFF);
        crc = CRC32_TABLE[(crc >> 24) ^ b2] ^ ((crc << 8) & 0xFFFFFFFF);
        crc = CRC32_TABLE[(crc >> 24) ^ b3] ^ ((crc << 8) & 0xFFFFFFFF);

        i += 4;
    }

    return crc;
}

// ===== 辅助函数实现 =====

float MotorProtocol::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(max_val, value));
}

int16_t MotorProtocol::clampToInt16(int32_t value) {
    return static_cast<int16_t>(std::max(-32768, std::min(32767, static_cast<int>(value))));
}

int32_t MotorProtocol::clampToInt32(int64_t value) {
    return static_cast<int32_t>(std::max(INT64_C(-2147483648),
        std::min(INT64_C(2147483647), value)));
}

/**
 * @brief 错误码描述（故障码）
 * 
 * 根据故障码表按位判断：
 * bit0: 过流
 * bit1: 瞬态过压
 * bit2: 持续过压
 * bit3: 瞬态欠压
 * bit4: 芯片过热
 * bit5: MOS过热/冷
 * bit6: MOS温度异常
 * bit8: 壳体温度异常
 * bit9: 绕组过热
 * bit10: 转子编码器1错误
 * bit12: 输出编码器错误
 * bit13: 储存数据错误
 * bit14: 异常复位
 * bit16: 芯片验证错误
 * bit17: 标定模式
 * bit19: 驱动版本过低
 * bit20: 固件型号错误
 * bit22: 硬件过流
 */
std::string MotorProtocol::getErrorDescription(uint32_t error_code) {
    if (error_code == 0) return "Normal";

    std::ostringstream oss;
    if (error_code & 0x00000001) oss << "Overcurrent ";
    if (error_code & 0x00000002) oss << "Transient overvoltage ";
    if (error_code & 0x00000004) oss << "Persistent overvoltage ";
    if (error_code & 0x00000008) oss << "Transient undervoltage ";
    if (error_code & 0x00000010) oss << "Chip overheat ";
    if (error_code & 0x00000020) oss << "MOS overheat/cold ";
    if (error_code & 0x00000040) oss << "MOS temperature abnormal ";
    if (error_code & 0x00000100) oss << "Housing temperature abnormal ";
    if (error_code & 0x00000200) oss << "Winding overheat ";
    if (error_code & 0x00000400) oss << "Rotor encoder1 error ";
    if (error_code & 0x00001000) oss << "Output encoder error ";
    if (error_code & 0x00002000) oss << "Storage data error ";
    if (error_code & 0x00004000) oss << "Abnormal reset ";
    if (error_code & 0x00010000) oss << "Chip verification error ";
    if (error_code & 0x00020000) oss << "Calibration mode ";
    if (error_code & 0x00080000) oss << "Driver version too low ";
    if (error_code & 0x00100000) oss << "Firmware model error ";
    if (error_code & 0x00400000) oss << "Hardware overcurrent ";

    return oss.str();
}

/**
 * @brief 警告码描述
 * 
 * 根据警告码表按位判断：
 * bit0: 低压电源异常
 */
std::string MotorProtocol::getWarningDescription(uint8_t warning_code) {
    if (warning_code == 0) return "Normal";

    std::ostringstream oss;
    if (warning_code & 0x01) oss << "Low voltage power abnormal ";

    return oss.str();
}