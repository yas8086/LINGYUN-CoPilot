#include "CsvWriter.h"
#include <QDir>
#include <QDateTime>

bool CsvWriter::startSession(const QString &dir, const QDateTime &startTime) {
    QDir().mkpath(dir);
    QString name = "temp_" + startTime.toString("yyyyMMdd_HHmmss") + ".csv";
    QString path = dir + "/" + name;
    m_file.setFileName(path);
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        return false;
    m_out.setDevice(&m_file);
    m_out.setEncoding(QStringConverter::Utf8);
    m_out.setGenerateByteOrderMark(true);   // UTF-8 BOM，Excel 友好
    m_out << "timestamp,node_id,temp_celsius,pressure_pa,raw,online,alarm\n";
    m_out.flush();
    return true;
}

void CsvWriter::write(const QVector<Sample> &samples) {
    if (!m_file.isOpen()) return;
    for (const auto &s : samples) {
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(s.timestampMs);
        m_out << dt.toString("yyyy-MM-dd HH:mm:ss.zzz") << ","
              << s.nodeId << ","
              << QString::number(s.tempCelsius, 'f', 1) << ","
              << (s.isPressure ? QString::number(s.pressurePa, 'f', 0) : "") << ","
              << QString("0x%1").arg(s.raw, 4, 16, QChar('0')).toUpper() << ","
              << s.online << ","
              << s.alarm << "\n";
    }
    m_out.flush();
}

void CsvWriter::close() {
    if (m_file.isOpen()) {
        m_out.flush();
        m_file.close();
    }
}
