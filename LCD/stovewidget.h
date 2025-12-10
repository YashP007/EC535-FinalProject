#ifndef STOVEWIDGET_H
#define STOVEWIDGET_H

#include <QWidget>
// #include <QNetworkAccessManager>  // Adafruit IO disabled
#include <QVector>

class QPushButton;
class QSlider;
class QLabel;
class QTimer;

class StoveWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StoveWidget(QWidget *parent = nullptr);

public slots:
    void toggleStove();                // on/off
    void setTargetTemp(int temp);      // slider
    void setSoftwareLimit(int temp);   // software limit slider
    void setCurrentTemp(double tempF);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    void updateTempLabel();
    void updateStatusLabel();          // Update status text
    // void sendTempToAdafruit();         // Adafruit IO disabled

    void initLedDevice();                      // open /dev/mysignal
    void writeLedCommand(const QByteArray &);  // send commands
    void syncLedWithSimState();                // LED Commands

    // Stove state
    bool   m_on;
    double m_currentTempF;
    int    m_targetTempF;
    int    m_softwareLimitF;      // Software limit (300°F default)
    const int m_hardwareLimitF = 450;  // Hardware limit (comparator threshold)

    // historh graph
    QVector<double> m_tempHistory;

    // UI widgets
    QPushButton *m_powerButton;
    QSlider     *m_tempSlider;
    QSlider     *m_softwareLimitSlider;  // Slider for software limit
    QLabel      *m_tempLabel;
    QLabel      *m_statusLabel;        // Status text above slider
    QLabel      *m_emergencyLabel;     // Emergency text at top when HW limit met
    QTimer      *m_simTimer;           // simulation timer

    // Adafruit IO HTTP - disabled (no web connectivity)
    // QNetworkAccessManager m_net;
    // QString m_aioUser;
    // QString m_aioKey;

    // LED
    int  m_ledFd;
    bool m_lastTempAbove;
    
    // Hardware Comparator
    int  m_compFd;                  // File descriptor for /dev/mytempsensor_comp
    bool m_comparatorTriggered;     // Comparator threshold exceeded state
    QTimer *m_compTimer;            // Timer to poll comparator status
    
    void initComparatorDevice();    // Open comparator device
    void readComparatorStatus();    // Read and parse comparator status
};

#endif //
