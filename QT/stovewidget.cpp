#include "stovewidget.h"

#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QFont>
#include <QTimer>
#include <QtMath>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <QDebug>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonObject>
#define O_NONBLOCK 0


StoveWidget::StoveWidget(QWidget *parent)
    : QWidget(parent),
      m_on(false),
      m_currentTempF(70.0),
      m_targetTempF(70),
      m_powerButton(nullptr),
      m_tempSlider(nullptr),
      m_tempLabel(nullptr),
      m_simTimer(nullptr),
      m_ledFd(-1),
      m_lastTempAbove(false)

{

    m_aioUser = "ldsj";
    m_aioKey  = "aio_Fmad035K4dRzhUgbvKHHZfjT9QF7";


    //UI
    m_powerButton = new QPushButton(tr("Stove: OFF"), this);
    m_powerButton->setCheckable(true);

    m_tempSlider = new QSlider(Qt::Horizontal, this);
    m_tempSlider->setRange(65, 150);        // allowed temperature in far
    m_tempSlider->setValue(m_targetTempF);

    m_tempLabel = new QLabel(this);
    m_tempLabel->setAlignment(Qt::AlignCenter);

    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    QLabel *title = new QLabel(tr("EC535 Smart Stove"), this);
    title->setAlignment(Qt::AlignCenter);
    QFont titleFont = title->font();
    titleFont.setPointSize(18);
    titleFont.setBold(true);
    title->setFont(titleFont);

    mainLayout->addWidget(title);
    mainLayout->addStretch();

    mainLayout->addWidget(m_tempLabel);

    QHBoxLayout *bottomRow = new QHBoxLayout;
    bottomRow->addWidget(m_powerButton);
    bottomRow->addWidget(new QLabel(tr("Target Temp"), this));
    bottomRow->addWidget(m_tempSlider);

    mainLayout->addLayout(bottomRow);

    setLayout(mainLayout);

    // signal
    connect(m_powerButton, &QPushButton::clicked,
            this, &StoveWidget::toggleStove);
    connect(m_tempSlider, &QSlider::valueChanged,
            this, &StoveWidget::setTargetTemp);

    initLedDevice();
    syncLedWithSimState();

    //
    // adc change later
    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(500); // 0.5s
    connect(m_simTimer, &QTimer::timeout, [this]() {
        if (!m_on)
            return;

        // Move current temp toward target temp
        if (m_currentTempF < m_targetTempF)
            m_currentTempF += 5.0;
        else if (m_currentTempF > m_targetTempF)
            m_currentTempF -= 3.0;

        updateTempLabel();
        update(); // repaint

        syncLedWithSimState();
        sendTempToAdafruit();
    });

    updateTempLabel();
    resize(480, 272); // nice for the LCD hopefullt
}

void StoveWidget::toggleStove()
{
    m_on = !m_on;

    if (m_on) {
        m_powerButton->setText(tr("Stove: ON"));
        m_simTimer->start();
    } else {
        m_powerButton->setText(tr("Stove: OFF"));
        m_simTimer->stop();
    }

    updateTempLabel();
    update(); // repaint

    syncLedWithSimState();
}

void StoveWidget::setTargetTemp(int temp)
{
    m_targetTempF = temp;
    updateTempLabel();
    update();

    syncLedWithSimState();
}

void StoveWidget::setCurrentTemp(double tempF)
{
    m_currentTempF = tempF;
    updateTempLabel();
    update();

    syncLedWithSimState();
    sendTempToAdafruit();
}

void StoveWidget::updateTempLabel()
{
    m_tempLabel->setText(
        tr("Current: %1°F    Target: %2°F")
            .arg(m_currentTempF, 0, 'f', 1)
            .arg(m_targetTempF)
    );
}

void StoveWidget::initLedDevice()
{
    m_ledFd = ::open("/dev/mysignal", O_WRONLY | O_NONBLOCK);
    if (m_ledFd < 0) {
        qWarning() << "Failed to open /dev/mysignal:" << strerror(errno);
        return;
    }

    //  connected so green LED comes on
    writeLedCommand("server_connected\n");
}

void StoveWidget::writeLedCommand(const QByteArray &cmd)
{
    if (m_ledFd < 0)
        return;

    ssize_t n = ::write(m_ledFd, cmd.constData(), cmd.size());
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        qWarning() << "write(/dev/mysignal) failed:" << strerror(errno)
        << "cmd =" << cmd;
    }
}


void StoveWidget::syncLedWithSimState()
{
    if (m_ledFd < 0)
        return;

    // blue (on/off)
    if (m_on) {
        writeLedCommand("stove_on\n");
    } else {
        writeLedCommand("stove_off\n");
    }

    // red led (above or below thereshbold)
    static constexpr double kThresholdF = 150.0;

    bool nowAbove = (m_currentTempF >= kThresholdF);

    if (nowAbove != m_lastTempAbove) {
        if (nowAbove) {
            writeLedCommand("temp_above\n");
        } else {
            writeLedCommand("temp_below\n");
        }
        m_lastTempAbove = nowAbove;
    }


}

void StoveWidget::sendTempToAdafruit()
{
    if (m_aioUser.isEmpty() || m_aioKey.isEmpty()) {
        qWarning() << "Adafruit IO user/key not set";
        return;
    }

    //
    QString urlStr = QString("https://io.adafruit.com/api/v2/ldsj/feeds/stove-temperature/data")
                         .arg(m_aioUser);
    QUrl url(urlStr);
    QNetworkRequest req(url);

    //
    QJsonObject obj;
    obj["value"] = m_currentTempF;
    QJsonDocument doc(obj);
    QByteArray body = doc.toJson(QJsonDocument::Compact);

    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    req.setRawHeader("X-AIO-Key", m_aioKey.toUtf8());

    QNetworkReply *reply = m_net.post(req, body);

    // debug
    QObject::connect(reply, &QNetworkReply::finished, reply, [reply]() {
        if (reply->error() != QNetworkReply::NoError) {
            qWarning() << "Adafruit POST failed:" << reply->errorString();
        }
        reply->deleteLater();
    });
}



void StoveWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // background
    {
        QLinearGradient bgGrad(rect().topLeft(), rect().bottomLeft());
        bgGrad.setColorAt(0.0, QColor(15, 15, 20));
        bgGrad.setColorAt(1.0, QColor(35, 35, 45));
        p.fillRect(rect(), bgGrad);
    }

    int topMargin    = 50;               // under title
    int bottomMargin = height() * 0.35;  // keep lower 35% for controls

    QRect stoveRect(25,
                    topMargin,
                    width() - 50,
                    height() - topMargin - bottomMargin);

    // shaodow
    QRect shadowRect = stoveRect.translated(4, 6);
    QRadialGradient shadowGrad(shadowRect.center(), shadowRect.width() * 0.6);
    shadowGrad.setColorAt(0.0, QColor(0, 0, 0, 120));
    shadowGrad.setColorAt(1.0, Qt::transparent);
    p.setBrush(shadowGrad);
    p.setPen(Qt::NoPen);
    p.drawRoundedRect(shadowRect, 24, 24);

    // make stove glass kinda
    QLinearGradient bodyGrad(stoveRect.topLeft(), stoveRect.bottomLeft());
    bodyGrad.setColorAt(0.0, QColor(45, 45, 55));
    bodyGrad.setColorAt(0.5, QColor(25, 25, 30));
    bodyGrad.setColorAt(1.0, QColor(10, 10, 12));

    p.setBrush(bodyGrad);
    p.setPen(QPen(QColor(120, 120, 130), 2));
    p.drawRoundedRect(stoveRect, 20, 20);

    // Top highlight strip
    QRect highlightRect = stoveRect.adjusted(3, 3, -3, -stoveRect.height() / 2);
    QLinearGradient hiGrad(highlightRect.topLeft(), highlightRect.bottomLeft());
    hiGrad.setColorAt(0.0, QColor(255, 255, 255, 50));
    hiGrad.setColorAt(1.0, QColor(255, 255, 255, 5));
    p.setBrush(hiGrad);
    p.setPen(Qt::NoPen);
    p.drawRoundedRect(highlightRect, 18, 18);

    //burner location
    int burnerRadius = qMin(stoveRect.width(), stoveRect.height()) / 6;

    int cxLeft   = stoveRect.left() + stoveRect.width() * 1 / 3;
    int cxRight  = stoveRect.left() + stoveRect.width() * 2 / 3;
    int cyTop    = stoveRect.top()  + stoveRect.height() * 1 / 3;
    int cyBottom = stoveRect.top()  + stoveRect.height() * 2 / 3;

    QPoint burners[4] = {
        QPoint(cxLeft,  cyTop),
        QPoint(cxRight, cyTop),
        QPoint(cxLeft,  cyBottom),
        QPoint(cxRight, cyBottom)
    };

    // flamas
    const double minTemp = 70.0;
    const double maxTemp = 150.0;
    double t = (m_currentTempF - minTemp) / (maxTemp - minTemp);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    // Base color for coils: from cool gray → hot orange/red
    int r = static_cast<int>(80  + t * 175); // 80 → 255
    int g = static_cast<int>(80  + t *  60); // 80 → 140
    int b = static_cast<int>(80  - t *  80); // 80 → 0

    QColor innerGlowColor(r, g, b);

    // Dangerous threshold for flames
    const double dangerThresholdF = 100.0;
    bool showFlames = m_on && (m_currentTempF >= dangerThresholdF);

    // Pen for the outer burner ring
    QPen outerPen(QColor(170, 170, 180), 3);

    for (int i = 0; i < 4; ++i) {
        QRect circleRect(burners[i].x() - burnerRadius,
                         burners[i].y() - burnerRadius,
                         2 * burnerRadius,
                         2 * burnerRadius);

        // Outer metallic ring
        p.setBrush(Qt::NoBrush);
        p.setPen(outerPen);
        p.drawEllipse(circleRect);

        // Inner glass surface (dark)
        QRect innerGlass = circleRect.adjusted(5, 5, -5, -5);
        QRadialGradient glassGrad(innerGlass.center(), innerGlass.width() / 2);
        glassGrad.setColorAt(0.0, QColor(10, 10, 12));
        glassGrad.setColorAt(1.0, QColor(25, 25, 30));
        p.setBrush(glassGrad);
        p.setPen(Qt::NoPen);
        p.drawEllipse(innerGlass);

        if (m_on) {
            // Glowing burner
            QRadialGradient coilGrad(innerGlass.center(), innerGlass.width() / 2);
            coilGrad.setColorAt(0.0, innerGlowColor.lighter(140));
            coilGrad.setColorAt(0.4, innerGlowColor);
            coilGrad.setColorAt(1.0, QColor(0, 0, 0, 0));
            p.setBrush(coilGrad);
            p.drawEllipse(innerGlass);

            // rings
            p.setPen(QPen(QColor(255, 180, 120, static_cast<int>(120 * t)), 2));
            int rings = 3;
            for (int k = 1; k <= rings; ++k) {
                int shrink = 5 + k * (innerGlass.width() / (2 * rings + 2));
                QRect coilRect = innerGlass.adjusted(shrink, shrink, -shrink, -shrink);
                p.drawEllipse(coilRect);
            }
        }

        // Flame emoji for HOT temps
        if (showFlames) {
            int flameSize = static_cast<int>(burnerRadius * (0.7 + 0.6 * t));
            QFont f = p.font();
            f.setPointSize(flameSize);
            p.setFont(f);
            p.setPen(Qt::yellow);
            p.drawText(circleRect, Qt::AlignCenter, QStringLiteral("🔥"));
        }
    }


    QRect statusBand = stoveRect.adjusted(5,
                                          stoveRect.height() - 45,
                                          -5,
                                          -5);
    QLinearGradient bandGrad(statusBand.topLeft(), statusBand.bottomLeft());
    bandGrad.setColorAt(0.0, QColor(0, 0, 0, 180));
    bandGrad.setColorAt(1.0, QColor(0, 0, 0, 100));
    p.setBrush(bandGrad);
    p.setPen(QPen(QColor(200, 200, 210, 120), 1));
    p.drawRoundedRect(statusBand, 10, 10);

    QString status;
    if (!m_on) {
        status = tr("Stove is OFF (Safe)");
    } else if (m_currentTempF >= dangerThresholdF) {
        status = tr("WARNING: HOT! %1°F")
                     .arg(m_currentTempF, 0, 'f', 1);
    } else {
        status = tr("Heating... %1°F (Target: %2°F)")
                     .arg(m_currentTempF, 0, 'f', 1)
                     .arg(m_targetTempF);
    }

    if (m_on && m_currentTempF >= dangerThresholdF) {
        p.setPen(QColor(255, 120, 120));
    } else {
        p.setPen(QColor(220, 220, 230));
    }

    QFont statusFont = p.font();
    statusFont.setPointSize(10);
    p.setFont(statusFont);
    p.drawText(statusBand, Qt::AlignCenter, status);
}
