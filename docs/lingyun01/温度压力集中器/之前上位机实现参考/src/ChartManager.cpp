#include "ChartManager.h"
#include <QDateTime>

// 8 种唯一颜色，色相均匀分布，区分度高（与 MainWindow 的 ID 选择器共用）
const QColor ChartManager::kColors[kMaxColors] = {
    QColor("#e74c3c"),  // ID1 红
    QColor("#e67e22"),  // ID2 橙
    QColor("#f1c40f"),  // ID3 黄
    QColor("#27ae60"),  // ID4 绿
    QColor("#00bcd4"),  // ID5 青
    QColor("#2980b9"),  // ID6 蓝
    QColor("#8e44ad"),  // ID7 紫
    QColor("#e91e63"),  // ID8 粉
};

ChartManager::ChartManager() {
    m_chart = new QChart();
    m_chart->setTitle("实时温度曲线");
    m_chart->legend()->setVisible(true);

    m_axisX = new QDateTimeAxis();
    m_axisX->setFormat("HH:mm:ss");
    m_axisX->setTitleText("时间");
    m_chart->addAxis(m_axisX, Qt::AlignBottom);

    m_axisY = new QValueAxis();
    m_axisY->setTitleText("温度 (℃)");
    m_axisY->setRange(-20, 80);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);
}

void ChartManager::setupNodes(int startId, int count) {
    // 清理旧曲线
    for (auto s : m_series) { m_chart->removeSeries(s); s->deleteLater(); }
    m_series.clear();
    m_startId = startId;
    m_count   = count;
    // 始终创建 8 个 series，通过 setVisible 控制显示
    const int totalIds = 8;
    for (int i = 0; i < totalIds; ++i) {
        int id = startId + i;
        auto *s = new QLineSeries();
        s->setName(QString("ID%1").arg(id));
        s->setPen(QPen(colorForIndex(i), 2));
        m_chart->addSeries(s);
        s->attachAxis(m_axisX);
        s->attachAxis(m_axisY);
        // 默认只显示前 count 个
        s->setVisible(i < count);
        m_series.insert(id, s);
    }
}

void ChartManager::append(const QVector<Sample> &samples) {
    for (const auto &s : samples) {
        auto it = m_series.find(s.nodeId);
        if (it == m_series.end()) continue;
        (*it)->append(s.timestampMs, s.tempCelsius);
        // 限制点数
        while ((*it)->count() > kMaxPointsPerSeries)
            (*it)->remove(0);
    }
    // 自适应 X 范围（最近 5 分钟，每格 30 秒 = 11 个刻度）
    if (!samples.isEmpty()) {
        qint64 now = samples.last().timestampMs;
        const qint64 windowMs = 5 * 60 * 1000;  // 5 分钟窗口
        m_axisX->setRange(QDateTime::fromMSecsSinceEpoch(now - windowMs),
                          QDateTime::fromMSecsSinceEpoch(now));
        m_axisX->setTickCount(11);  // 0,30s,60s,...,300s 共 11 个刻度
    }
}

void ChartManager::clear() {
    for (auto s : m_series) s->clear();
}

void ChartManager::setVisible(int nodeId, bool visible) {
    auto it = m_series.find(nodeId);
    if (it != m_series.end()) {
        (*it)->setVisible(visible);
    }
}
