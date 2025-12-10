#ifndef STOVEWIDGET_H
#define STOVEWIDGET_H

#include <QWidget>
#include <QNetworkAccessManager>

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
    void setCurrentTemp(double tempF); // ADC / real temp

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void updateTempLabel();
    void sendTempToAdafruit();         // current temp → Adafruit IO

    bool   m_on;
    double m_currentTempF;
    int    m_targetTempF;

    QPushButton *m_powerButton;
    QSlider     *m_tempSlider;
    QLabel      *m_tempLabel;

    QTimer      *m_simTimer;           // simulation

    // adafruit
    QNetworkAccessManager m_net;
    QString m_aioUser;
    QString m_aioKey;

    // LED stuff
    int  m_ledFd;          // /dev/mysignal
    bool m_lastTempAbove;  // last above/below state

    void initLedDevice();
    void writeLedCommand(const QByteArray &);
    void syncLedWithSimState();
};

#endif // STOVEWIDGET_H
