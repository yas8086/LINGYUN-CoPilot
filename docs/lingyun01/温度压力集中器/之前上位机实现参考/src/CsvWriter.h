#pragma once
#include "Sample.h"
#include <QString>
#include <QFile>
#include <QTextStream>

class CsvWriter {
public:
    // 在 dir 下创建 temp_YYYYMMDD_HHMMSS.csv，写表头（UTF-8 BOM）
    bool startSession(const QString &dir, const class QDateTime &startTime);
    void write(const QVector<Sample> &samples);
    void close();
    QString currentFile() const { return m_file.fileName(); }
    bool isOpen() const { return m_file.isOpen(); }
private:
    QFile m_file;
    QTextStream m_out;
};
