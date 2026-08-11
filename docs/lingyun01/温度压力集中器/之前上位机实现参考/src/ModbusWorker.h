#pragma once
#include "Sample.h"
#include "AppConfig.h"
#include <QObject>
#include <QTimer>
#include <QVector>
#include <QHash>

class QModbusRtuSerialClient;
class QModbusReply;

class ModbusWorker : public QObject {
    Q_OBJECT
public:
    explicit ModbusWorker(QObject *parent = nullptr);
    ~ModbusWorker();
public slots:
    void start(const AppConfig &cfg);
    void stop();
    void setSamplePeriod(int ms);   // 运行时更新采集周期
signals:
    void dataReady(QVector<Sample> samples);
    void error(const QString &msg);
    void statusMessage(const QString &msg);
    void frameLog(const QString &msg);   // 原始 Modbus 帧日志（用于调试）
private slots:
    void onTimeout();
    void onReplyFinished();
private:
    QModbusRtuSerialClient *m_master = nullptr;
    QTimer *m_timer = nullptr;   // 在子线程动态创建，确保事件循环正确
    AppConfig m_cfg;
    int   m_failCount = 0;
    bool  m_running = false;

    // 本轮采集的聚合数据（按 nodeId 合并温度和压力结果）
    qint64 m_batchTimestamp = 0;
    int    m_pendingReplies = 0;
    QHash<int, Sample> m_batchSamples;
};
