#pragma once
#include <cstdint>
#include <vector>
#include <memory>
#include <string>

/**
 * @brief 电机反馈数据结构
 *
 * 存储从电机返回的所有物理量
 * 数据已经过单位转换，可直接使用
 */
struct MotorFeedback
{
    uint8_t motor_id;     // 电机ID (0-14)
    uint8_t mode;         // 工作模式 (0:停机, 1:FOC)
    uint8_t timeout;      // 超时状态 (0:正常, 1:超时)
    int8_t temp_driver;   // 驱动温度 (°C)
    uint8_t temp_winding; // 绕组温度 (°C)
    float voltage;        // 电压 (V)
    float torque;         // 扭矩 (Nm)
    float speed;          // 速度 (rad/s)
    float position;       // 位置 (rad)
    uint32_t error_code;  // 错误码
    uint8_t warning_code; // 警告码
};

/**
 * @brief 电机协议处理类
 *
 * 负责：
 * 1. 打包控制命令（物理量 -> 协议数据）
 * 2. 解包反馈数据（协议数据 -> 物理量）
 * 3. CRC32校验计算
 * 4. 数值范围限制和单位转换
 *
 * 不涉及串口通信，只处理数据
 */
class MotorProtocol
{
public:
    MotorProtocol();
    ~MotorProtocol() = default;

    /**
     * @brief 构建控制包
     *
     * 将物理量转换为20字节的协议数据包
     * 包含：包头、模式字节、控制数据、CRC32校验
     *
     * @param motor_id 电机ID (0-14)
     * @param mode 工作模式 (0:停机, 1:FOC)
     * @param timeout 超时保护 (true:开启, false:关闭)
     * @param torque 目标扭矩 (Nm)
     * @param speed 目标速度 (rad/s)
     * @param position 目标位置 (rad)
     * @param kp 位置刚度 (Nm/rad)
     * @param kd 速度刚度 (Nm/(rad/s))
     * @return 20字节的控制数据包
     */
    std::vector<uint8_t> buildControlPacket(
        uint8_t motor_id,
        uint8_t mode,
        bool timeout,
        float torque,
        float speed,
        float position,
        float kp,
        float kd);// 构建控制包
    std::vector<uint8_t> buildClearPacket(uint8_t motor_id);// 构建清除包
    std::vector<uint8_t> buildResetPacket(uint8_t motor_id);// 构建复位包

    /**
     * @brief 解析反馈包
     *
     * 将26字节的反馈数据包解析为物理量
     * 包括：包头验证、CRC校验、数据提取和单位转换
     *
     * @param data 26字节的反馈数据
     * @return 解析后的反馈数据，失败返回nullptr
     */
    std::shared_ptr<MotorFeedback> parseFeedbackPacket(const std::vector<uint8_t> &data);

    /**
     * @brief 获取错误描述
     * @param error_code 错误码
     * @return 错误描述字符串
     */
    static std::string getErrorDescription(uint32_t error_code);

    /**
     * @brief 获取警告描述
     * @param warning_code 警告码
     * @return 警告描述字符串
     */
    static std::string getWarningDescription(uint8_t warning_code);

private:
    /**
     * @brief 计算CRC32校验值
     *
     * 使用查表法计算，与Python版本算法完全一致
     * 每次处理4个字节（小端序）
     *
     * @param data 数据指针
     * @param len 数据长度
     * @return CRC32校验值
     */
    uint32_t calculateCRC32(const uint8_t *data, size_t len);

    /**
     * @brief 浮点数范围限制
     */
    static float clamp(float value, float min_val, float max_val);

    /**
     * @brief int16范围限制（-32768 ~ 32767）
     */
    static int16_t clampToInt16(int32_t value);

    /**
     * @brief int32范围限制（-2147483648 ~ 2147483647）
     */
    static int32_t clampToInt32(int64_t value);

    // 协议常量定义
    static constexpr double RATIO = 38.0 / 3.0;            // 减速比：电机转子到输出端的减速比
    static constexpr double M_PI = 3.14159265358979323846; // 圆周率

    // CRC32查找表（256个32位值）
    static const uint32_t CRC32_TABLE[256];
};
