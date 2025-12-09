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
    void toggleStove();                //on off
    void setTargetTemp(int temp);      // slider
    void setCurrentTemp(double tempF);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void updateTempLabel();
    void sendTempToAdafruit();         // current temp to adafruit

    bool   m_on;
    double m_currentTempF;
    int    m_targetTempF;

    QPushButton *m_powerButton;
    QSlider     *m_tempSlider;
    QLabel      *m_tempLabel;

    QTimer      *m_simTimer;           // simulation

    // --- Adafruit IO / networking ---
    QNetworkAccessManager m_net;
    QString m_aioUser;
    QString m_aioKey;

private:
    int  m_ledFd;          // /dev/mysignal
    bool m_lastTempAbove;  // last “above/below” state we sent

    void initLedDevice();                      // open /dev/mysignal
    void writeLedCommand(const QByteArray &);  // send commands like "temp_above"
    void syncLedWithSimState();                // map stove/temperature -> LED commands

};

#endif // STOVEWIDGET_H
