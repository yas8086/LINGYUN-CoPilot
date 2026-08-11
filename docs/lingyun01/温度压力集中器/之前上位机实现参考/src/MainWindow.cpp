#include "MainWindow.h"
#include "ModbusWorker.h"
#include "Sample.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QStatusBar>
#include <QFileDialog>
#include <QSplitter>
#include <QDateTime>
#include <QSerialPortInfo>
#include <QChartView>
#include <QFileInfo>
#include <QFrame>
#include <QMetaType>
#include <QColor>
#include <QTimer>
#include <QtMath>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QStringList>
#include <QLineEdit>

// 统一美化 QGroupBox：标题与边框留空，内容与边框留空
static void styleGroupBox(QWidget *box) {
    box->setStyleSheet(
        "QGroupBox {"
        "  border: 1px solid #aaaaaa;"
        "  border-radius: 4px;"
        "  margin-top: 8px;"
        "  padding: 6px;"
        "  padding-top: 10px;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  subcontrol-position: top left;"
        "  left: 10px;"
        "  padding: 0 5px;"
        "  color: #333333;"
        "}");
}

static QIcon makeThermometerIcon() {
    const int sz = 64;
    QPixmap pix(sz, sz);
    pix.fill(Qt::transparent);
    QPainter p(&pix);
    p.setRenderHint(QPainter::Antialiasing);
    // 温度计球部（红色圆）
    p.setPen(Qt::NoPen);
    p.setBrush(QColor("#e74c3c"));
    p.drawEllipse(QPointF(sz/2.0, sz*0.82), sz*0.18, sz*0.18);
    // 玻璃管（白色描边）
    QPainterPath tube;
    qreal tubeW = sz*0.14;
    qreal tubeX = sz/2.0 - tubeW/2.0;
    qreal tubeTop = sz*0.12;
    qreal tubeBottom = sz*0.74;
    tube.addRoundedRect(QRectF(tubeX, tubeTop, tubeW, tubeBottom-tubeTop),
                        tubeW/2.0, tubeW/2.0);
    p.setBrush(QColor("#ecf0f1"));
    p.setPen(QPen(QColor("#bdc3c7"), 1.5));
    p.drawPath(tube);
    // 水银柱（红色，从球部向上填充约 60%）
    QPainterPath mercury;
    qreal mW = tubeW*0.6;
    qreal mX = sz/2.0 - mW/2.0;
    qreal mTop = sz*0.35;
    qreal mBottom = sz*0.78;
    mercury.addRoundedRect(QRectF(mX, mTop, mW, mBottom-mTop),
                           mW/2.0, mW/2.0);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor("#e74c3c"));
    p.drawPath(mercury);
    // 顶部刻度线
    p.setPen(QPen(QColor("#7f8c8d"), 1.0));
    for (int i = 0; i < 4; ++i) {
        qreal y = tubeTop + (tubeBottom-tubeTop)*(i+1)/5.0;
        p.drawLine(QPointF(tubeX+tubeW+2, y), QPointF(tubeX+tubeW+7, y));
    }
    return QIcon(pix);
}

// ID 颜色映射（最多 8 个，与 ChartManager 共用同一套颜色）
const QColor MainWindow::kIdColors[] = {
    ChartManager::kColors[0],
    ChartManager::kColors[1],
    ChartManager::kColors[2],
    ChartManager::kColors[3],
    ChartManager::kColors[4],
    ChartManager::kColors[5],
    ChartManager::kColors[6],
    ChartManager::kColors[7],
};

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    // 跨线程信号槽需要注册自定义类型
    qRegisterMetaType<QVector<Sample>>("QVector<Sample>");
    qRegisterMetaType<AppConfig>("AppConfig");

    setWindowTitle("LoRa 温度监测");
    setWindowIcon(makeThermometerIcon());
    resize(1200, 880);
    buildUi();
    loadConfig();
    applyConfigToUi();
    rebuildCards();
    rebuildAlarms();
    rebuildIdSelectors();
    m_chartMgr.setupNodes(m_cfg.startNodeId, m_cfg.nodeCount);

    // 恢复上次拖动后的 splitter 布局（saveState 递归保存了所有嵌套子 splitter）
    if (!m_cfg.splitterState.isEmpty())
        m_outerSplitter->restoreState(m_cfg.splitterState);
}

MainWindow::~MainWindow() {
    if (m_thread) {
        if (m_worker) QMetaObject::invokeMethod(m_worker, &ModbusWorker::stop);
        m_thread->quit();
        m_thread->wait();
    }
    m_csv.close();
    if (m_outerSplitter)
        m_cfg.splitterState = m_outerSplitter->saveState();
    m_cfg.save();
}

void MainWindow::buildUi() {
    // 外层垂直分隔：上=主面板，下=log框
    m_outerSplitter = new QSplitter(Qt::Vertical);
    m_outerSplitter->setHandleWidth(8);
    m_outerSplitter->setChildrenCollapsible(false);
    m_outerSplitter->setStyleSheet(
        "QSplitter::handle { background: #d0d0d0; }"
        "QSplitter::handle:horizontal { width: 8px; }"
        "QSplitter::handle:vertical { height: 8px; }");

    // 上方主面板（左右分隔）
    m_mainSplitter = new QSplitter(Qt::Horizontal);
    m_mainSplitter->setHandleWidth(8);
    m_mainSplitter->setChildrenCollapsible(false);

    // ===== 左侧面板（配置 + 报警 + 卡片，垂直分隔） =====
    m_leftSplitter = new QSplitter(Qt::Vertical);
    m_leftSplitter->setHandleWidth(8);
    m_leftSplitter->setChildrenCollapsible(false);

    // --- 配置区域 ---
    auto *configGroup = new QGroupBox("配置");
    styleGroupBox(configGroup);
    auto *form = new QFormLayout(configGroup);
    m_portCombo = new QComboBox;
    m_refreshPortBtn = new QPushButton("刷新");
    for (const auto &info : QSerialPortInfo::availablePorts())
        m_portCombo->addItem(info.portName());
    // 串口选择行：下拉框 + 刷新按钮
    auto *portRow = new QWidget;
    auto *portLay = new QHBoxLayout(portRow);
    portLay->setContentsMargins(0, 0, 0, 0);
    portLay->addWidget(m_portCombo, 1);
    portLay->addWidget(m_refreshPortBtn);
    m_baudCombo = new QComboBox;
    m_baudCombo->addItems({"9600","19200","38400","57600","115200"});
    m_slaveSpin = new QSpinBox; m_slaveSpin->setRange(1, 247);
    m_startIdSpin = new QSpinBox; m_startIdSpin->setRange(1, 200);
    m_nodeCountSpin = new QSpinBox; m_nodeCountSpin->setRange(1, 16);
    m_periodSpin = new QSpinBox; m_periodSpin->setRange(500, 60000); m_periodSpin->setSingleStep(500);
    m_pressureIdsEdit = new QLineEdit;
    m_pressureIdsEdit->setPlaceholderText("逗号分隔，如 6");
    m_csvDirLabel = new QLabel;
    m_csvBtn = new QPushButton("选择...");
    m_startBtn = new QPushButton("开始");
    m_stopBtn = new QPushButton("停止");
    m_stopBtn->setEnabled(false);

    form->addRow("串口", portRow);
    form->addRow("波特率", m_baudCombo);
    form->addRow("从机地址", m_slaveSpin);
    form->addRow("起始ID", m_startIdSpin);
    form->addRow("节点数", m_nodeCountSpin);
    form->addRow("采样周期(ms)", m_periodSpin);
    form->addRow("压力传感器ID", m_pressureIdsEdit);
    form->addRow("CSV目录", m_csvDirLabel);
    form->addRow("", m_csvBtn);
    form->addRow(m_startBtn);
    form->addRow(m_stopBtn);

    // --- 数据卡片区域（网格布局，8 个卡片） ---
    auto *cardGroup = new QGroupBox("当前数据");
    styleGroupBox(cardGroup);
    m_cardLayout = new QGridLayout(cardGroup);
    m_cardLayout->setContentsMargins(4, 4, 4, 4);
    m_cardLayout->setSpacing(4);
    m_cardContainer = new QWidget;

    // --- 报警设置组（节点阈值，可实时修改） ---
    m_alarmGroup = new QGroupBox("报警温度阈值 (℃)");
    styleGroupBox(m_alarmGroup);
    m_alarmLayout = new QGridLayout(m_alarmGroup);
    m_alarmLayout->setContentsMargins(4, 4, 4, 4);
    m_alarmLayout->setSpacing(4);

    configGroup->setMinimumHeight(280);
    m_alarmGroup->setMinimumHeight(90);
    cardGroup->setMinimumHeight(200);
    m_leftSplitter->addWidget(configGroup);
    m_leftSplitter->addWidget(m_alarmGroup);
    m_leftSplitter->addWidget(cardGroup);
    m_leftSplitter->setSizes({300, 90, 400});

    // ===== 右侧面板（曲线 + ID选择器 + 表格，垂直分隔） =====
    m_rightSplitter = new QSplitter(Qt::Vertical);
    m_rightSplitter->setHandleWidth(8);
    m_rightSplitter->setChildrenCollapsible(false);

    auto *chartView = new QChartView(m_chartMgr.chart());
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setMinimumHeight(200);

    // --- ID 选择器区域（放在曲线下方） ---
    auto *idGroup = new QGroupBox("曲线显示");
    styleGroupBox(idGroup);
    m_idSelectorLayout = new QGridLayout(idGroup);
    m_idSelectorLayout->setContentsMargins(4, 4, 4, 4);
    m_idSelectorLayout->setSpacing(4);
    m_idSelectorPanel = new QWidget;

    m_table = new QTableWidget(0, 6);
    m_table->setHorizontalHeaderLabels({"时间","节点","温度(℃)","压力(Pa)","原始值","报警"});
    m_table->horizontalHeader()->setStretchLastSection(true);

    // 图表:表格 = 3:2，即图表是表格的 1.5 倍（满足 1.2~1.8 范围）
    idGroup->setMinimumHeight(80);
    m_table->setMinimumHeight(150);
    m_rightSplitter->addWidget(chartView);   // 曲线
    m_rightSplitter->addWidget(idGroup);     // ID选择器
    m_rightSplitter->addWidget(m_table);     // 表格
    m_rightSplitter->setSizes({390, 80, 260});

    // ===== 组装主面板 =====
    m_mainSplitter->addWidget(m_leftSplitter);   // 左侧
    m_mainSplitter->addWidget(m_rightSplitter);  // 右侧
    m_mainSplitter->setSizes({380, 820});

    // ===== 底部 log 框（宽度占整个应用） =====
    auto *logGroup = new QGroupBox("日志");
    styleGroupBox(logGroup);
    auto *logLay = new QVBoxLayout(logGroup);
    logLay->setContentsMargins(4, 4, 4, 4);
    m_logBox = new QPlainTextEdit;
    m_logBox->setReadOnly(true);
    m_logBox->setMaximumBlockCount(500);  // 最多 500 行
    m_logBox->setMinimumHeight(100);      // 最小高度，可由 splitter 拖动调整
    QFont logFont("Monospace");
    logFont.setStyleHint(QFont::Monospace);
    m_logBox->setFont(logFont);
    logLay->addWidget(m_logBox);

    logGroup->setMinimumHeight(80);
    m_outerSplitter->addWidget(m_mainSplitter);   // 主面板
    m_outerSplitter->addWidget(logGroup);         // log框
    m_outerSplitter->setSizes({760, 120});
    setCentralWidget(m_outerSplitter);
    statusBar()->showMessage("就绪");
    appendLog("应用启动");

    // 信号槽连接
    connect(m_startBtn, &QPushButton::clicked, this, &MainWindow::onStart);
    connect(m_stopBtn,  &QPushButton::clicked, this, &MainWindow::onStop);
    connect(m_csvBtn,   &QPushButton::clicked, this, &MainWindow::onChooseCsvDir);
    // 刷新串口列表
    connect(m_refreshPortBtn, &QPushButton::clicked, this, [this]() {
        QString cur = m_portCombo->currentText();
        m_portCombo->clear();
        for (const auto &info : QSerialPortInfo::availablePorts())
            m_portCombo->addItem(info.portName());
        // 尝试恢复之前选中的串口
        int idx = m_portCombo->findText(cur);
        if (idx >= 0) m_portCombo->setCurrentIndex(idx);
        appendLog(QString("串口列表已刷新，共 %1 个").arg(m_portCombo->count()));
    });
    // 采集周期变更：运行中实时生效
    connect(m_periodSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this](int ms) {
        if (m_worker) {
            // 转发给子线程，立即生效
            QMetaObject::invokeMethod(m_worker, "setSamplePeriod", Qt::QueuedConnection, Q_ARG(int, ms));
            appendLog(QString("采集周期已更新为 %1 ms").arg(ms));
        }
    });
    connect(m_startIdSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int){
        applyUiToConfig(); rebuildCards(); rebuildAlarms(); rebuildIdSelectors();
        m_chartMgr.setupNodes(m_cfg.startNodeId, m_cfg.nodeCount);
    });
    connect(m_nodeCountSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int){
        applyUiToConfig(); rebuildCards(); rebuildAlarms(); rebuildIdSelectors();
        m_chartMgr.setupNodes(m_cfg.startNodeId, m_cfg.nodeCount);
    });
    // 压力ID变更：实时更新配置和卡片
    connect(m_pressureIdsEdit, &QLineEdit::textChanged, this, [this](const QString &){
        applyUiToConfig(); rebuildCards(); rebuildIdSelectors();
    });
}

void MainWindow::loadConfig() {
    m_cfg.load();
}

void MainWindow::applyConfigToUi() {
    m_portCombo->setCurrentText(m_cfg.portName);
    m_baudCombo->setCurrentText(QString::number(m_cfg.baudRate));
    m_slaveSpin->setValue(m_cfg.slaveAddr);
    m_startIdSpin->setValue(m_cfg.startNodeId);
    m_nodeCountSpin->setValue(m_cfg.nodeCount);
    m_periodSpin->setValue(m_cfg.samplePeriodMs);
    m_pressureIdsEdit->setText(m_cfg.pressureNodeIds);
    QString csvDir = m_cfg.resolvedCsvDir();
    // 首次运行若目录不存在则创建（避免空目录被误判为异常）
    if (!QDir(csvDir).exists()) {
        QDir().mkpath(csvDir);
    }
    m_csvDirLabel->setText(csvDir);
}

void MainWindow::applyUiToConfig() {
    m_cfg.portName       = m_portCombo->currentText();
    m_cfg.baudRate       = m_baudCombo->currentText().toInt();
    m_cfg.slaveAddr      = m_slaveSpin->value();
    m_cfg.startNodeId    = m_startIdSpin->value();
    m_cfg.nodeCount      = m_nodeCountSpin->value();
    m_cfg.samplePeriodMs = m_periodSpin->value();
    m_cfg.pressureNodeIds = m_pressureIdsEdit->text().trimmed();
    // csvDir 不再由 UI 控制，始终使用项目根目录/data
    m_cfg.ensureDefaults();
}

void MainWindow::rebuildCards() {
    // 清空旧卡片
    while (m_cardLayout->count()) {
        auto *item = m_cardLayout->takeAt(0);
        if (item->widget()) item->widget()->deleteLater();
    }
    m_cardLabels.clear();

    const QSet<int> pressureIds = m_cfg.parsedPressureNodeIds();
    const int cols = 2;  // 每行 2 个卡片
    const int totalCards = 8;  // 始终显示 8 个卡片
    for (int i = 0; i < totalCards; ++i) {
        int id = m_cfg.startNodeId + i;
        bool isPressure = pressureIds.contains(id);
        auto *card = new QFrame;
        card->setFrameShape(QFrame::Box);
        // 超出节点数的卡片用更暗的背景表示无数据
        bool active = (i < m_cfg.nodeCount);
        QString bg = active ? "#f5f5f5" : "#ececec";
        card->setStyleSheet(QString("QFrame{background:%1;border:1px solid #ccc;border-radius:6px;}").arg(bg));
        auto *l = new QVBoxLayout(card);
        l->setContentsMargins(4, 4, 4, 4);
        auto *title = new QLabel(QString("ID%1 %2").arg(id).arg(isPressure ? "压力" : "温度"));
        title->setAlignment(Qt::AlignCenter);
        auto *val = new QLabel(active ? "--" : "无");
        val->setAlignment(Qt::AlignCenter);
        if (!active) val->setStyleSheet("color:#999;");
        QFont f = val->font(); f.setPointSize(18); f.setBold(true); val->setFont(f);
        l->addWidget(title);
        l->addWidget(val);
        m_cardLabels.insert(id, val);
        m_cardLayout->addWidget(card, i / cols, i % cols);
    }
}

void MainWindow::rebuildAlarms() {
    if (!m_alarmLayout) return;
    // 清空旧控件
    while (m_alarmLayout->count()) {
        auto *item = m_alarmLayout->takeAt(0);
        if (item->widget()) item->widget()->deleteLater();
    }
    m_alarmLowSpin  = nullptr;
    m_alarmHighSpin = nullptr;

    // 表头
    auto *hLow  = new QLabel("下限 (℃)");
    auto *hHigh = new QLabel("上限 (℃)");
    QFont hf = hLow->font(); hf.setBold(true);
    hLow->setFont(hf); hHigh->setFont(hf);
    hLow->setAlignment(Qt::AlignCenter);
    hHigh->setAlignment(Qt::AlignCenter);
    m_alarmLayout->addWidget(hLow,  0, 0);
    m_alarmLayout->addWidget(hHigh, 0, 1);

    // 全局共用的下限/上限
    auto *lowSpin  = new QDoubleSpinBox;
    lowSpin->setRange(-100.0, 200.0);
    lowSpin->setDecimals(1);
    lowSpin->setSingleStep(0.5);
    lowSpin->setValue(m_cfg.alarmLow);
    connect(lowSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double v) {
                if (v > m_cfg.alarmHigh) {
                    // 防呆：不允许下限超过上限
                    m_alarmLowSpin->blockSignals(true);
                    m_alarmLowSpin->setValue(m_cfg.alarmHigh);
                    m_alarmLowSpin->blockSignals(false);
                    return;
                }
                m_cfg.alarmLow = v;
                appendLog(QString("报警下限已更新为 %1℃ (全局生效)").arg(v));
            });

    auto *highSpin = new QDoubleSpinBox;
    highSpin->setRange(-100.0, 200.0);
    highSpin->setDecimals(1);
    highSpin->setSingleStep(0.5);
    highSpin->setValue(m_cfg.alarmHigh);
    connect(highSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double v) {
                if (v < m_cfg.alarmLow) {
                    m_alarmHighSpin->blockSignals(true);
                    m_alarmHighSpin->setValue(m_cfg.alarmLow);
                    m_alarmHighSpin->blockSignals(false);
                    return;
                }
                m_cfg.alarmHigh = v;
                appendLog(QString("报警上限已更新为 %1℃ (全局生效)").arg(v));
            });

    m_alarmLowSpin  = lowSpin;
    m_alarmHighSpin = highSpin;
    m_alarmLayout->addWidget(lowSpin,  1, 0);
    m_alarmLayout->addWidget(highSpin, 1, 1);
}

void MainWindow::rebuildIdSelectors() {
    // 清空旧选择器
    while (m_idSelectorLayout->count()) {
        auto *item = m_idSelectorLayout->takeAt(0);
        if (item->widget()) item->widget()->deleteLater();
    }
    m_idToggles.clear();

    const int cols = 4;  // 每行 4 个，紧凑显示
    for (int i = 0; i < kMaxSelectableIds; ++i) {
        int id = m_cfg.startNodeId + i;
        auto *cb = new QCheckBox(QString("ID%1").arg(id));
        cb->setChecked(i < m_cfg.nodeCount);  // 默认勾选当前节点数

        // 颜色色块（使用与曲线相同的颜色）
        auto *colorLabel = new QLabel;
        colorLabel->setFixedSize(16, 16);
        colorLabel->setStyleSheet(QString("background-color: %1; border: 1px solid #999; border-radius: 3px;")
            .arg(kIdColors[i % 8].name()));

        // 布局：色块 + 复选框
        auto *rowWidget = new QWidget;
        auto *rowLay = new QHBoxLayout(rowWidget);
        rowLay->setContentsMargins(0, 0, 0, 0);
        rowLay->setSpacing(4);
        rowLay->addWidget(colorLabel);
        rowLay->addWidget(cb);
        rowLay->addStretch();

        m_idToggles.insert(id, cb);
        m_idSelectorLayout->addWidget(rowWidget, i / cols, i % cols);

        connect(cb, &QCheckBox::toggled, this, &MainWindow::onIdToggled);
    }
}

void MainWindow::onIdToggled(bool checked) {
    auto *cb = qobject_cast<QCheckBox*>(sender());
    if (!cb) return;

    // 找到对应的 ID
    for (auto it = m_idToggles.begin(); it != m_idToggles.end(); ++it) {
        if (it.value() == cb) {
            m_chartMgr.setVisible(it.key(), checked);
            appendLog(QString("曲线 %1 ID%2")
                .arg(checked ? "显示" : "隐藏").arg(it.key()));
            break;
        }
    }
}

void MainWindow::onChooseCsvDir() {
    QString dir = QFileDialog::getExistingDirectory(this, "选择 CSV 保存目录", m_csvDirLabel->text());
    if (!dir.isEmpty()) {
        m_csvDirLabel->setText(dir);
        m_cfg.csvDir = dir;
        m_cfg.csvDirUserSet = true;  // 标记用户显式选择，下次启动仍用此目录
        appendLog("CSV 目录已更改为: " + dir);
    }
}

void MainWindow::onStart() {
    applyUiToConfig();
    m_cfg.save();
    if (m_cfg.portName.isEmpty()) {
        statusBar()->showMessage("请选择串口", 5000);
        appendLog("[警告] 未选择串口");
        return;
    }
    // 新建 CSV 会话
    if (!m_csv.startSession(m_cfg.resolvedCsvDir(), QDateTime::currentDateTime())) {
        statusBar()->showMessage("CSV 文件创建失败", 5000);
        appendLog("[错误] CSV 文件创建失败");
        return;
    }
    m_chartMgr.clear();
    appendLog(QString("开始采集 | 串口=%1 波特率=%2 从机=%3 节点ID%4~ID%5 周期=%6ms")
        .arg(m_cfg.portName).arg(m_cfg.baudRate).arg(m_cfg.slaveAddr)
        .arg(m_cfg.startNodeId).arg(m_cfg.startNodeId + m_cfg.nodeCount - 1)
        .arg(m_cfg.samplePeriodMs));
    if (!m_cfg.parsedPressureNodeIds().isEmpty()) {
        QStringList ids;
        for (int id : m_cfg.parsedPressureNodeIds()) ids << QString::number(id);
        appendLog(QString("压力传感器ID: %1").arg(ids.join(",")));
    }
    appendLog(QString("CSV 文件: %1").arg(QFileInfo(m_csv.currentFile()).fileName()));

    // 启动子线程
    m_thread = new QThread(this);
    m_worker = new ModbusWorker;
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    connect(m_worker, &ModbusWorker::dataReady, this, &MainWindow::onDataReady);
    connect(m_worker, &ModbusWorker::error, this, &MainWindow::onError);
    connect(m_worker, &ModbusWorker::statusMessage, this, &MainWindow::onStatus);
    connect(m_worker, &ModbusWorker::frameLog, this, &MainWindow::appendLog);
    m_thread->start();
    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection,
                              Q_ARG(AppConfig, m_cfg));
    m_startBtn->setEnabled(false);
    m_stopBtn->setEnabled(true);
}

void MainWindow::onStop() {
    if (m_worker) QMetaObject::invokeMethod(m_worker, &ModbusWorker::stop, Qt::QueuedConnection);
    if (m_thread) { m_thread->quit(); m_thread->wait(); m_thread->deleteLater(); m_thread = nullptr; m_worker = nullptr; }
    m_csv.close();
    m_startBtn->setEnabled(true);
    m_stopBtn->setEnabled(false);
    statusBar()->showMessage("已停止");
    appendLog("采集已停止，CSV 文件已关闭");
}

void MainWindow::onDataReady(QVector<Sample> samples) {
    m_csv.write(samples);
    // 所有节点的温度都绘制到曲线（包括压力节点的温度）
    m_chartMgr.append(samples);
    for (const auto &s : samples) {
        auto it = m_cardLabels.find(s.nodeId);
        if (it != m_cardLabels.end()) {
            if (s.online == 0) {
                (*it)->setText("--");
                (*it)->setStyleSheet("color:#999;background:#f5f5f5;");
            } else if (s.isPressure) {
                // 压力节点同时显示压力和温度
                QString txt = QString::number(s.pressurePa / 1000.0, 'f', 3) + " kPa\n"
                            + QString::number(s.tempCelsius, 'f', 1) + " ℃";
                (*it)->setText(txt);
                if (s.alarm == 1)      (*it)->setStyleSheet("color:white;background:#c0392b;");
                else if (s.alarm == -1)(*it)->setStyleSheet("color:white;background:#2980b9;");
                else                   (*it)->setStyleSheet("color:#222;background:#f5f5f5;");
            } else {
                (*it)->setText(QString::number(s.tempCelsius, 'f', 1) + " ℃");
                if (s.alarm == 1)      (*it)->setStyleSheet("color:white;background:#c0392b;");
                else if (s.alarm == -1)(*it)->setStyleSheet("color:white;background:#2980b9;");
                else                   (*it)->setStyleSheet("color:#222;background:#f5f5f5;");
            }
        }
        // 报警日志（所有节点的温度都参与报警）
        if (s.alarm == 1) {
            appendLog(QString("[报警] ID%1 超上限: %2℃").arg(s.nodeId).arg(QString::number(s.tempCelsius, 'f', 1)));
        } else if (s.alarm == -1) {
            appendLog(QString("[报警] ID%1 超下限: %2℃").arg(s.nodeId).arg(QString::number(s.tempCelsius, 'f', 1)));
        }
        // 表格插行（顶部插入，最新在上）
        int row = 0;
        m_table->insertRow(row);
        QDateTime dt = QDateTime::fromMSecsSinceEpoch(s.timestampMs);
        m_table->setItem(row, 0, new QTableWidgetItem(dt.toString("HH:mm:ss.zzz")));
        m_table->setItem(row, 1, new QTableWidgetItem(QString::number(s.nodeId)));
        m_table->setItem(row, 2, new QTableWidgetItem(QString::number(s.tempCelsius, 'f', 1)));
        m_table->setItem(row, 3, new QTableWidgetItem(
            s.isPressure ? QString::number(s.pressurePa, 'f', 0) : ""));
        m_table->setItem(row, 4, new QTableWidgetItem(
            QString("0x%1").arg(s.raw, 4, 16, QChar('0')).toUpper()));
        m_table->setItem(row, 5, new QTableWidgetItem(
            s.alarm==1?"超上限":(s.alarm==-1?"超下限":"正常")));
        // 限制行数
        while (m_table->rowCount() > 500) m_table->removeRow(m_table->rowCount()-1);
    }
    statusBar()->showMessage(QString("采集中 | 上次: %1 | 文件: %2")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
        .arg(QFileInfo(m_csv.currentFile()).fileName()));
}

void MainWindow::onError(const QString &msg) {
    statusBar()->showMessage("错误: " + msg, 10000);
    appendLog("[错误] " + msg);
}

void MainWindow::onStatus(const QString &msg) {
    statusBar()->showMessage(msg);
    appendLog(msg);
}

void MainWindow::appendLog(const QString &msg) {
    if (!m_logBox) return;
    QString line = QDateTime::currentDateTime().toString("[HH:mm:ss.zzz] ") + msg;
    m_logBox->appendPlainText(line);
}
