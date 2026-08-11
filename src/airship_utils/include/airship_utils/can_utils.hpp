// 灵云01号伴飞电脑 — CAN 帧解析工具
// 提供与 ROS 无关的纯函数,用于解析 CAN 数据帧中的小端整数、
// 按分辨率换算物理量、以及位域提取。所有函数均为纯函数,便于 gtest。
//
// 【前置条件】以下提取函数(get_u16_le/get_u32_le/get_i16_le/get_u8/get_bits2)
//   以裸指针+offset 访问, 本身不做长度检查。调用方(各协议库)必须在调用前
//   依据 DLC 完成帧长度校验, 确保 data 长度 >= offset + 所需字节数,
//   否则行为未定义。当前所有协议库已在 parse 入口校验, 本层为契约约定。
#ifndef AIRSHIP_UTILS__CAN_UTILS_HPP_
#define AIRSHIP_UTILS__CAN_UTILS_HPP_

#include <cstddef>
#include <cstdint>

namespace airship_utils
{

// 从 CAN 数据帧(小端序)提取 16 位无符号整数
// data: 8 字节帧数据, offset: 起始字节(0~6)
uint16_t get_u16_le(const uint8_t * data, size_t offset);

// 从 CAN 数据帧(小端序)提取 32 位无符号整数
uint32_t get_u32_le(const uint8_t * data, size_t offset);

// 从 CAN 数据帧(小端序)提取 16 位有符号整数(补码)
int16_t get_i16_le(const uint8_t * data, size_t offset);

// 从 CAN 数据帧提取单个字节
uint8_t get_u8(const uint8_t * data, size_t offset);

// 提取字节中某 2 位的值(如 DCDC 启停位的 Bit5~4)
// bit_pos: 起始位(0~6), 返回该 2 位组合值
uint8_t get_bits2(const uint8_t * data, size_t byte_offset, uint8_t bit_pos);

// 单位换算: 原始值 * scale + offset(如 电压 raw*0.1V/BIT)
float scale_i16(int16_t raw, float scale, float offset = 0.0f);
float scale_u16(uint16_t raw, float scale, float offset = 0.0f);

// 温度换算: 带 -40 偏移的 1℃/BIT 温度(MPPT/DCDC 用)
float temp_with_offset(int8_t raw);

}  // namespace airship_utils

#endif  // AIRSHIP_UTILS__CAN_UTILS_HPP_
