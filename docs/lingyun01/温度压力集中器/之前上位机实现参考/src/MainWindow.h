#pragma once
#include "AppConfig.h"
#include "CsvWriter.h"
#include "ChartManager.h"
#include <QMainWindow>
#include <QThread>
#include <QHash>
#include <QColor>
class ModbusWorker;
class QComboBox;
class QSpinBox;
class QHBoxLayout;
class QGridLayout;
class QSplitter;
class QTableWidget;
class QLabel;
class QPushButton;
class QCheckBox;
class QPlainTextEdit;
class QDoubleSpinBox;
class QLineEdit;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
private slots:
    void onStart();
    void onStop();
    void onDataReady(QVector<Sample> samples);
    void onError(const QString &msg);
    void onStatus(const QString &msg);
    void onChooseCsvDir();
    void onIdToggled(bool checked);
private:
    void buildUi();
    void loadConfig();
    void applyConfigToUi();
    void applyUiToConfig();
    void rebuildCards();
    void rebuildIdSelectors();
    void rebuildAlarms();
    void appendLog(const QString &msg);

    // 配置面板控件
    QComboBox *m_portCombo;
    QComboBox *m_baudCombo;
    QSpinBox  *m_slaveSpin;
    QSpinBox  *m_startIdSpin;
    QSpinBox  *m_nodeCountSpin;
    QSpinBox  *m_periodSpin;
    QLineEdit *m_pressureIdsEdit;   // 压力传感器ID列表
    QLabel    *m_csvDirLabel;
    QPushButton *m_csvBtn;
    QPushButton *m_startBtn;
    QPushButton *m_stopBtn;
    QPushButton *m_refreshPortBtn = nullptr;   // 刷新串口列表

    // 报警设置（全局共用一组上下限）
    QWidget         *m_alarmGroup = nullptr;
    QGridLayout     *m_alarmLayout = nullptr;
    QDoubleSpinBox  *m_alarmLowSpin  = nullptr;
    QDoubleSpinBox  *m_alarmHighSpin = nullptr;

    // 温度卡片（网格布局，可多显示几个）
    QWidget       *m_cardContainer;
    QGridLayout   *m_cardLayout;
    QHash<int, QLabel*> m_cardLabels;

    // ID 选择器（多选按钮 + 颜色色块）
    QWidget             *m_idSelectorPanel;
    QGridLayout         *m_idSelectorLayout;
    QHash<int, QCheckBox*> m_idToggles;
    static constexpr int kMaxSelectableIds = 8;
    static const QColor kIdColors[];

    ChartManager   m_chartMgr;
    QTableWidget  *m_table;
    QPlainTextEdit *m_logBox = nullptr;

    AppConfig  m_cfg;
    CsvWriter  m_csv;
    ModbusWorker *m_worker = nullptr;
    QThread   *m_thread = nullptr;
    QSplitter *m_outerSplitter = nullptr;   // 顶层垂直分隔（主面板/日志），saveState 递归保存所有子 splitter
    QSplitter *m_mainSplitter  = nullptr;   // 主面板水平分隔（左/右）
    QSplitter *m_leftSplitter  = nullptr;   // 左侧垂直分隔（配置/报警/卡片）
    QSplitter *m_rightSplitter = nullptr;   // 右侧垂直分隔（曲线/ID选择器/表格）
};
