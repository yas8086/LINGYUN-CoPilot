#pragma once
#include "Sample.h"
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QHash>
#include <QDateTimeAxis>
#include <QValueAxis>

#include <QColor>

class ChartManager {
public:
    ChartManager();
    QChart* chart() { return m_chart; }
    void setupNodes(int startId, int count);
    void append(const QVector<Sample> &samples);
    void clear();
    void setVisible(int nodeId, bool visible);

    // 8 种唯一颜色，ID 选择器和曲线共用
    static constexpr int kMaxColors = 8;
    static const QColor kColors[kMaxColors];
    static QColor colorForIndex(int index) {
        return kColors[index % kMaxColors];
    }
private:
    QChart *m_chart;
    QDateTimeAxis *m_axisX;
    QValueAxis    *m_axisY;
    QHash<int, QLineSeries*> m_series;  // key=nodeId
    int m_startId = 1;
    int m_count   = 0;
    static constexpr int kMaxPointsPerSeries = 900; // 30min @ 2s
};
