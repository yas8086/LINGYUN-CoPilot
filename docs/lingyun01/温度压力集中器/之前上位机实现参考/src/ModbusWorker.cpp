#include "ModbusWorker.h"
#include <QtSerialBus>   // 包含 QModbusRtuSerialClient/QModbusReply/QModbusDataUnit
#include <QSerialPort>
#include <QDateTime>

// Modbus RTU CRC-16 计算
static quint16 modbusCrc16(const QByteArray &data) {
    quint16 crc = 0xFFFF;
    for (char c : data) {
        crc ^= static_cast<quint8>(c);
        for (int i = 0; i < 8; ++i) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// 构造读输入寄存器的 RTU 请求帧
static QByteArray buildReadFrame(int slaveAddr, int funcCode, int startAddr, int quantity) {
    QByteArray frame;
    frame.append(static_cast<char>(slaveAddr));
    frame.append(static_cast<char>(funcCode));
    frame.append(static_cast<char>((startAddr >> 8) & 0xFF));
    frame.append(static_cast<char>(startAddr & 0xFF));
    frame.append(static_cast<char>((quantity >> 8) & 0xFF));
    frame.append(static_cast<char>(quantity & 0xFF));
    quint16 crc = modbusCrc16(frame);
    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));
    return frame;
}

static QString bytesToHex(const QByteArray &ba) {
    return QString(ba.toHex(' ').toUpper());
}

enum class ReadType : int {
    Temperature,
    Pressure
};

ModbusWorker::ModbusWorker(QObject *parent) : QObject(parent) {
}

ModbusWorker::~ModbusWorker() {
    stop();
    if (m_master) m_master->deleteLater();
}

void ModbusWorker::start(const AppConfig &cfg) {
    m_cfg = cfg;
    if (m_master) { m_master->deleteLater(); m_master = nullptr; }
    if (m_timer)  { m_timer->deleteLater(); m_timer = nullptr; }

    m_master = new QModbusRtuSerialClient(this);
    m_master->setConnectionParameter(QModbusDevice::SerialPortNameParameter, cfg.portName);
    m_master->setConnectionParameter(QModbusDevice::SerialBaudRateParameter, cfg.baudRate);
    m_master->setConnectionParameter(QModbusDevice::SerialDataBitsParameter, cfg.dataBits);
    m_master->setConnectionParameter(QModbusDevice::SerialStopBitsParameter,
        cfg.stopBits == 1 ? QSerialPort::OneStop : QSerialPort::TwoStop);
    QSerialPort::Parity p = QSerialPort::NoParity;
    if (cfg.parity == 1) p = QSerialPort::OddParity;
    else if (cfg.parity == 2) p = QSerialPort::EvenParity;
    m_master->setConnectionParameter(QModbusDevice::SerialParityParameter, p);
    m_master->setTimeout(1000);

    if (!m_master->connectDevice()) {
        emit error(QString("串口打开失败: %1").arg(m_master->errorString()));
        m_master->deleteLater();
        m_master = nullptr;
        return;
    }
    m_running = true;
    m_failCount = 0;
    emit statusMessage("采集中");
    // 在子线程动态创建 QTimer，确保事件循环正确运行
    m_timer = new QTimer(this);
    m_timer->setTimerType(Qt::PreciseTimer);
    connect(m_timer, &QTimer::timeout, this, &ModbusWorker::onTimeout);
    m_timer->start(m_cfg.samplePeriodMs);
    onTimeout();   // 立即采集一次
}

void ModbusWorker::stop() {
    if (m_timer) { m_timer->stop(); m_timer->deleteLater(); m_timer = nullptr; }
    m_running = false;
    if (m_master) m_master->disconnectDevice();
    emit statusMessage("已停止");
}

void ModbusWorker::setSamplePeriod(int ms) {
    m_cfg.samplePeriodMs = ms;
    if (m_running && m_timer && m_timer->isActive()) {
        m_timer->start(ms);  // 重启定时器，立即应用新周期
    }
}

void ModbusWorker::onTimeout() {
    if (!m_master || !m_running) return;

    // 本轮采集：每个节点都读温度；压力节点额外读压力
    m_batchTimestamp = QDateTime::currentMSecsSinceEpoch();
    m_batchSamples.clear();
    m_pendingReplies = 0;

    const QSet<int> pressureIds = m_cfg.parsedPressureNodeIds();

    auto sendRead = [&](int nodeId, ReadType type, int addr, int quantity) {
        QModbusDataUnit unit(QModbusDataUnit::InputRegisters, addr, quantity);
        QModbusReply *reply = m_master->sendReadRequest(unit, m_cfg.slaveAddr);
        if (!reply) {
            if (++m_failCount >= 3) emit error("通讯异常: 请求发送失败");
            return;
        }
        reply->setProperty("nodeId", nodeId);
        reply->setProperty("readType", static_cast<int>(type));
        connect(reply, &QModbusReply::finished, this, &ModbusWorker::onReplyFinished);
        ++m_pendingReplies;

        if (type == ReadType::Pressure) {
            QByteArray frame = buildReadFrame(m_cfg.slaveAddr, 0x04, addr, quantity);
            emit frameLog(QString("[发送] ID%1 压力请求: %2").arg(nodeId).arg(bytesToHex(frame)));
        }
    };

    for (int i = 0; i < m_cfg.nodeCount; ++i) {
        int nodeId = m_cfg.startNodeId + i;
        bool isPressure = pressureIds.contains(nodeId);

        // 预创建该节点的默认 sample
        Sample s;
        s.timestampMs = m_batchTimestamp;
        s.nodeId = nodeId;
        s.isPressure = isPressure;
        s.raw = 0;
        s.tempCelsius = 0;
        s.pressurePa = 0;
        s.online = 0;
        s.alarm = 0;
        m_batchSamples.insert(nodeId, s);

        // 所有节点都读温度，保留压力传感器的温度值
        sendRead(nodeId, ReadType::Temperature, m_cfg.tempRegAddr + (nodeId - 1), 1);

        // 压力节点额外读压力
        if (isPressure) {
            sendRead(nodeId, ReadType::Pressure, m_cfg.pressureRegAddrForId(nodeId), 2);
        }
    }

    if (m_pendingReplies == 0) {
        // 没有任何在线请求，直接上报
        m_failCount = 0;
        emit dataReady(m_batchSamples.values().toVector());
        m_batchSamples.clear();
    }
}

void ModbusWorker::onReplyFinished() {
    QModbusReply *reply = qobject_cast<QModbusReply *>(sender());
    if (!reply) return;
    reply->deleteLater();

    int nodeId = reply->property("nodeId").toInt();
    ReadType type = static_cast<ReadType>(reply->property("readType").toInt());

    auto it = m_batchSamples.find(nodeId);
    if (it == m_batchSamples.end()) return;   // 不应该发生
    Sample &s = it.value();

    if (reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();
        s.online = 1;   // 至少有一个读成功
        if (type == ReadType::Pressure) {
            // 打印原始应答帧（尽可能还原完整 RTU 帧，含 CRC）
            QModbusResponse raw = reply->rawResult();
            QByteArray respFrame;
            respFrame.append(static_cast<char>(m_cfg.slaveAddr));
            respFrame.append(static_cast<char>(raw.functionCode()));
            respFrame.append(raw.data());
            quint16 crc = modbusCrc16(respFrame);
            respFrame.append(static_cast<char>(crc & 0xFF));
            respFrame.append(static_cast<char>((crc >> 8) & 0xFF));
            emit frameLog(QString("[接收] ID%1 压力应答: %2").arg(nodeId).arg(bytesToHex(respFrame)));

            if (unit.valueCount() >= 2) {
                s.pressurePa = parsePressurePa(unit.value(0), unit.value(1));
            }
        } else {   // Temperature
            if (unit.valueCount() >= 1) {
                s.raw = unit.value(0);
                s.tempCelsius = parseTempCelsius(unit.value(0));
                s.alarm = checkAlarm(s.tempCelsius, m_cfg.alarmLow, m_cfg.alarmHigh);
            }
        }
    } else {
        if (++m_failCount >= 3)
            emit error(QString("通讯异常: %1").arg(reply->errorString()));
    }

    --m_pendingReplies;

    if (m_pendingReplies <= 0) {
        // 本轮全部完成，清零失败计数并上报
        m_failCount = 0;
        emit dataReady(m_batchSamples.values().toVector());
        m_batchSamples.clear();
    }
}
