#ifndef STOVEWIDGET_H
#define STOVEWIDGET_H

#include <QWidget>
#include <QNetworkAccessManager>
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
    void setCurrentTemp(double tempF);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void updateTempLabel();
    void sendTempToAdafruit();         // adafruit

    void initLedDevice();                      // open /dev/mysignal
    void writeLedCommand(const QByteArray &);  // send commands
    void syncLedWithSimState();                // LED Commands

    // Stove state
    bool   m_on;
    double m_currentTempF;
    int    m_targetTempF;

    // historh graph
    QVector<double> m_tempHistory;

    // UI widgets
    QPushButton *m_powerButton;
    QSlider     *m_tempSlider;
    QLabel      *m_tempLabel;
    QTimer      *m_simTimer;           // simulation timer

    // Adafruit IO HTTP
    QNetworkAccessManager m_net;
    QString m_aioUser;
    QString m_aioKey;

    // LED
    int  m_ledFd;
    bool m_lastTempAbove;
};

#endif //
