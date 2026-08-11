#pragma once
#include <QtGlobal>

// 单次采集数据样本
struct Sample {
    qint64  timestampMs;   // 采集时间戳（epoch 毫秒）
    int     nodeId;        // 节点 ID（1~16）
    quint16 raw;           // 寄存器原始值（温度节点：原始温度；压力节点：原始值高16位）
    qreal   tempCelsius;   // 解析后温度（℃），压力节点无效
    qreal   pressurePa;    // 解析后压力（Pa），温度节点为 0
    bool    isPressure;    // true=压力传感器，false=温度传感器
    int     online;        // 1=本次成功读取，0=未读到
    int     alarm;         // 0=正常，1=超上限，-1=超下限（仅温度节点有效）
};

// 温度解析：Modbus 大端 quint16 -> qint16 补码 -> /10.0
// 例：raw=0x00BF(191) -> 19.1℃；raw=0xFF60(65376) -> -16.0℃
inline qreal parseTempCelsius(quint16 raw) {
    return static_cast<qint16>(raw) / 10.0;
}

// 压力解析：2 个 Modbus 寄存器组合成 32 位无符号整数，单位 Pa
// 例：high=0x000F, low=0x4240 -> 1000000 Pa
inline qreal parsePressurePa(quint16 high, quint16 low) {
    return static_cast<qreal>((static_cast<quint32>(high) << 16) | static_cast<quint32>(low));
}

// 报警判定：返回 0 正常 / 1 超上限 / -1 超下限
inline int checkAlarm(qreal temp, qreal low, qreal high) {
    if (temp > high)  return 1;
    if (temp < low)   return -1;
    return 0;
}
