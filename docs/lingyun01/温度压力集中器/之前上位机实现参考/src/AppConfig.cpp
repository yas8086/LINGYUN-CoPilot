#include "AppConfig.h"
#include <QSettings>
#include <QStandardPaths>
#include <QDir>
#include <QCoreApplication>
#include <QRegularExpression>

static QString defaultCsvDir() {
    return QCoreApplication::applicationDirPath() + "/data";
}

void AppConfig::ensureDefaults() {
    if (csvDir.isEmpty()) {
        csvDir = defaultCsvDir();
    }
    // 报警阈值全局默认
    if (alarmLow > alarmHigh) alarmLow = alarmHigh;  // 防呆
}

QString AppConfig::resolvedCsvDir() const {
    if (csvDir.isEmpty())
        return QDir::cleanPath(defaultCsvDir());

    // 若以当前可执行目录为基准的相对路径，始终解析到 <appDir>/data
    if (csvDir == defaultCsvDir() || csvDir == "data")
        return QDir::cleanPath(defaultCsvDir());

    return QDir::cleanPath(csvDir);
}

QSet<int> AppConfig::parsedPressureNodeIds() const {
    QSet<int> ids;
    for (const QString &tok : pressureNodeIds.split(QRegularExpression("[,;\\s]+"), Qt::SkipEmptyParts)) {
        bool ok = false;
        int id = tok.toInt(&ok);
        if (ok && id > 0) ids.insert(id);
    }
    return ids;
}

bool AppConfig::isPressureNode(int id) const {
    return parsedPressureNodeIds().contains(id);
}

quint16 AppConfig::pressureRegAddrForId(int id) const {
    // 节点ID从1开始，每节点占2个寄存器
    return static_cast<quint16>(pressureRegAddr + 2 * (id - 1));
}

void AppConfig::load() {
    QSettings s;
    portName        = s.value("serial/portName", portName).toString();
    baudRate        = s.value("serial/baudRate", baudRate).toInt();
    dataBits        = s.value("serial/dataBits", dataBits).toInt();
    stopBits        = s.value("serial/stopBits", stopBits).toInt();
    parity          = s.value("serial/parity", parity).toInt();
    slaveAddr       = s.value("serial/slaveAddr", slaveAddr).toInt();
    startNodeId     = s.value("node/startNodeId", startNodeId).toInt();
    nodeCount       = s.value("node/nodeCount", nodeCount).toInt();
    samplePeriodMs  = s.value("node/samplePeriodMs", samplePeriodMs).toInt();
    tempRegAddr     = static_cast<quint16>(s.value("node/tempRegAddr", tempRegAddr).toUInt());
    pressureRegAddr = static_cast<quint16>(s.value("node/pressureRegAddr", pressureRegAddr).toUInt());
    pressureNodeIds = s.value("node/pressureNodeIds", pressureNodeIds).toString();
    // csvDir：用户未显式选择目录时，使用相对路径 "data"
    csvDirUserSet    = s.value("csv/userSet", false).toBool();
    QString savedDir = s.value("csv/dir").toString();
    QString appDir   = QCoreApplication::applicationDirPath();
    if (csvDirUserSet && !savedDir.isEmpty()) {
        // 保留用户自定义目录；但如果保存的目录和当前程序不在同一目录树（例如旧 build 目录），
        // 则回退到默认相对路径，避免移动/重装包后仍指向失效路径。
        QString canonicalSaved = QDir(savedDir).canonicalPath();
        QString canonicalApp   = QDir(appDir).canonicalPath();
        bool sameTree = !canonicalSaved.isEmpty()
                        && (canonicalSaved == canonicalApp
                            || canonicalSaved.startsWith(canonicalApp + "/"));
        if (!sameTree) {
            csvDir = "data";
            csvDirUserSet = false;
        } else {
            csvDir = savedDir;
        }
    } else {
        csvDir = "data";
        csvDirUserSet = false;
    }
    // 报警阈值（全局）
    s.beginGroup("alarm");
    alarmLow  = s.value("global/low",  -10.0).toDouble();
    alarmHigh = s.value("global/high", 60.0).toDouble();
    s.endGroup();

    // UI 布局
    splitterState = s.value("ui/splitterState").toByteArray();
}

void AppConfig::save() {
    QSettings s;
    s.setValue("serial/portName", portName);
    s.setValue("serial/baudRate", baudRate);
    s.setValue("serial/dataBits", dataBits);
    s.setValue("serial/stopBits", stopBits);
    s.setValue("serial/parity", parity);
    s.setValue("serial/slaveAddr", slaveAddr);
    s.setValue("node/startNodeId", startNodeId);
    s.setValue("node/nodeCount", nodeCount);
    s.setValue("node/samplePeriodMs", samplePeriodMs);
    s.setValue("node/tempRegAddr", tempRegAddr);
    s.setValue("node/pressureRegAddr", pressureRegAddr);
    s.setValue("node/pressureNodeIds", pressureNodeIds);
    // csvDir：默认保存相对路径 "data"，移动/重装包后仍能自动解析到 <appDir>/data
    if (csvDirUserSet && !csvDir.isEmpty() && csvDir != "data") {
        s.setValue("csv/dir", csvDir);
        s.setValue("csv/userSet", true);
    } else {
        s.setValue("csv/dir", "data");
        s.setValue("csv/userSet", false);
    }
    // 报警阈值（全局）
    s.beginGroup("alarm");
    s.setValue("global/low",  alarmLow);
    s.setValue("global/high", alarmHigh);
    s.endGroup();

    // UI 布局
    s.setValue("ui/splitterState", splitterState);
}
