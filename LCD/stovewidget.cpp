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
    // adafruit
    m_aioUser = "ldsj";
    m_aioKey  = "aio_Fmad035K4dRzhUgbvKHHZfjT9QF7";

    // initial tempertaure
    m_tempHistory.reserve(120);
    m_tempHistory.append(m_currentTempF);

    //power button
    m_powerButton = new QPushButton(QStringLiteral("⏻"), this);
    m_powerButton->setCheckable(true);
    m_powerButton->setFixedSize(48, 48);
    m_powerButton->setToolTip(tr("Stove OFF"));

    m_powerButton->setStyleSheet(
        "QPushButton { "
        "  border-radius: 24px;"
        "  border: 3px solid white;"
        "  background-color: #222222;"
        "  color: white;"
        "  font-size: 24px;"
        "}"
        "QPushButton:checked { "
        "  background-color: #44aa44;"
        "}"
        );

    // slider for temperature
    m_tempSlider = new QSlider(Qt::Horizontal, this);
    m_tempSlider->setRange(0, 100);        // farenheight
    m_tempSlider->setValue(m_targetTempF);

    // Text label for current/target temperature
    m_tempLabel = new QLabel(this);
    m_tempLabel->setAlignment(Qt::AlignCenter);
    QFont tempFont = m_tempLabel->font();
    tempFont.setPointSize(10);              
    m_tempLabel->setFont(tempFont);
    m_tempLabel->setStyleSheet("color: white;");
    //  title and power button on top,

    // slider row at the bottom.
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    QHBoxLayout *topRow = new QHBoxLayout;
    QLabel *title = new QLabel(tr("EC535 Smart Stove"), this);
    title->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    QFont titleFont = title->font();
    titleFont.setPointSize(14);
    titleFont.setBold(true);
    title->setFont(titleFont);
    title->setStyleSheet("color: white;");
    topRow->addWidget(title);
    topRow->addStretch();
    topRow->addWidget(m_powerButton, 0, Qt::AlignRight);

    mainLayout->addLayout(topRow);
    mainLayout->addWidget(m_tempLabel);


    mainLayout->addStretch();

    // Bottom row with slider
    QHBoxLayout *bottomRow = new QHBoxLayout;
    bottomRow->addWidget(new QLabel(tr("Target Temp"), this));
    bottomRow->addWidget(m_tempSlider);
    mainLayout->addLayout(bottomRow);

    setLayout(mainLayout);

    connect(m_powerButton, &QPushButton::clicked,
            this, &StoveWidget::toggleStove);
    connect(m_tempSlider, &QSlider::valueChanged,
            this, &StoveWidget::setTargetTemp);

    //LED

    initLedDevice();
    syncLedWithSimState();

   //simulation

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

        // Log this temp in the history for the graph
        m_tempHistory.append(m_currentTempF);
        if (m_tempHistory.size() > 120) {
            m_tempHistory.remove(0);   // keep most recent samples
        }

        updateTempLabel();
        update(); // repaint

        syncLedWithSimState();
        //sendTempToAdafruit();  //
    });

    updateTempLabel();
    resize(480, 272); // for LCD to fit
}

void StoveWidget::toggleStove()
{
    m_on = !m_on;

    if (m_on) {
        m_powerButton->setToolTip(tr("Stove ON"));
        m_simTimer->start();
    } else {
        m_powerButton->setToolTip(tr("Stove OFF"));
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

    // adc
    m_tempHistory.append(m_currentTempF);
    if (m_tempHistory.size() > 120) {
        m_tempHistory.remove(0);
    }

    updateTempLabel();
    update();

    syncLedWithSimState();
    //sendTempToAdafruit();
}

void StoveWidget::updateTempLabel()
{
    m_tempLabel->setText(
        tr("Current: %1°F    Target: %2°F")
            .arg(m_currentTempF, 0, 'f', 0)
            .arg(m_targetTempF)
        );
}


// LED device helpers

void StoveWidget::initLedDevice()
{
    m_ledFd = ::open("/dev/mysignal", O_WRONLY | O_NONBLOCK);
    if (m_ledFd < 0) {
        qWarning() << "Failed to open /dev/mysignal:" << strerror(errno);
        return;
    }

    // Connected so green LED comes on
    writeLedCommand("server_connected");
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

    // Blue is on off type
    if (m_on) {
        writeLedCommand("stove_on");
    } else {
        writeLedCommand("stove_off");
    }

    // Red bad
    static constexpr double kThresholdF = 60.0;

    bool nowAbove = (m_currentTempF >= kThresholdF);

    if (nowAbove != m_lastTempAbove) {
        if (nowAbove) {
            writeLedCommand("temp_above");
        } else {
            writeLedCommand("temp_below");
        }
        m_lastTempAbove = nowAbove;
    }
}

//adafruit

void StoveWidget::sendTempToAdafruit()
{
    if (m_aioUser.isEmpty() || m_aioKey.isEmpty()) {
        qWarning() << "Adafruit IO user/key not set";
        return;
    }

    // When Wi-Fi is present, this posts the current temperature to:
    // https://io.adafruit.com/api/v2/<user>/feeds/stove-temperature/data
    QString urlStr = QString("https://io.adafruit.com/api/v2/%1/feeds/stove-temperature/data")
                         .arg(m_aioUser);
    QUrl url(urlStr);
    QNetworkRequest req(url);

    QJsonObject obj;
    obj["value"] = m_currentTempF;
    QJsonDocument doc(obj);
    QByteArray body = doc.toJson(QJsonDocument::Compact);

    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    req.setRawHeader("X-AIO-Key", m_aioKey.toUtf8());

    QNetworkReply *reply = m_net.post(req, body);

    // Debug
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

   //background

    {
        QLinearGradient bgGrad(rect().topLeft(), rect().bottomLeft());
        bgGrad.setColorAt(0.0, QColor(15, 15, 20));
        bgGrad.setColorAt(1.0, QColor(35, 35, 45));
        p.fillRect(rect(), bgGrad);
    }

    // Define a content area between top widgets and bottom slider row
    int topMargin    = 80;               // under title and power button
    int bottomMargin = height() * 0.28;

    QRect contentRect(20,
                      topMargin,
                      width() - 40,
                      height() - topMargin - bottomMargin);

    if (contentRect.height() <= 0 || contentRect.width() <= 0)
        return;

   // left and righ equally
    int halfWidth = contentRect.width() / 2;
    QRect leftRect(contentRect.left(),
                   contentRect.top(),
                   halfWidth - 5,
                   contentRect.height());

    QRect rightRect(leftRect.right() + 10,
                    contentRect.top(),
                    contentRect.right() - (leftRect.right() + 10),
                    contentRect.height());


    const double minTemp = 0.0;
    const double maxTemp = 100.0;
    double t = (m_currentTempF - minTemp) / (maxTemp - minTemp);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    //graph
    int r = static_cast<int>(80  + t * 175); // 80 → 255
    int g = static_cast<int>(80  + t *  60); // 80 → 140
    int b = static_cast<int>(80  - t *  80); // 80 → 0
    QColor innerGlowColor(r, g, b);

    //harware limit underneath
    int hwHeight = 50;           // height of HW limit box
    int graphBottomGap = 8;      // spacing between graph and HW box

    QRect hwRect(leftRect.left() + 10,
                 leftRect.bottom() - hwHeight,
                 leftRect.width() - 20,
                 hwHeight);

    QRect graphRect(leftRect.left() + 8,
                    leftRect.top() + 8,
                    leftRect.width() - 16,
                    leftRect.height() - hwHeight - graphBottomGap - 16);

    // Graph background
    p.setPen(QPen(QColor(150, 150, 160), 1));
    p.setBrush(QColor(25, 25, 35));
    p.drawRoundedRect(graphRect, 8, 8);

    QRect graphInner = graphRect.adjusted(6, 6, -6, -6);

    // Draw axes
    p.setPen(QColor(120, 120, 130));
    p.drawLine(graphInner.bottomLeft(), graphInner.bottomRight()); // x-axis
    p.drawLine(graphInner.bottomLeft(), graphInner.topLeft());     // y-axis

    //reference line

    double refTemp = 60.0;  // comparator reference

    // normalize 60F to graph Y coordinate:
    double normRef = (refTemp - minTemp) / (maxTemp - minTemp);
    if (normRef < 0.0) normRef = 0.0;
    if (normRef > 1.0) normRef = 1.0;

    qreal yRef = graphInner.bottom() - normRef * (graphInner.height() - 1);

    // Draw thick dashed reference line
    QPen refPen(QColor(200, 200, 100), 2);
    refPen.setStyle(Qt::DashLine);
    p.setPen(refPen);
    p.drawLine(QPointF(graphInner.left(),  yRef),
               QPointF(graphInner.right(), yRef));

    // // label it
    // p.setPen(Qt::yellow);
    // p.drawText(graphInner.left(),
    //            yRef - 8,
    //            tr("60°F reference"));



    // Y-axis tick labels (min / max / mid)
    p.setPen(Qt::white);
    QFont graphFont = p.font();
    graphFont.setPointSize(7);
    p.setFont(graphFont);

    double midTemp = 0.5 * (minTemp + maxTemp);
    p.drawText(graphInner.left() - 30, graphInner.bottom(), 28, 10,
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(minTemp, 'f', 0));
    p.drawText(graphInner.left() - 30, graphInner.center().y() - 5, 28, 10,
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(midTemp, 'f', 0));
    p.drawText(graphInner.left() - 30, graphInner.top() - 5, 28, 10,
               Qt::AlignRight | Qt::AlignBottom,
               QString::number(maxTemp, 'f', 0));

    // Graph label
    p.drawText(graphInner.adjusted(0, -18, 0, 0),
               Qt::AlignTop | Qt::AlignHCenter,
               tr("Temperature History (°F)"));

    // Plot temperature history as a line
    if (m_tempHistory.size() >= 2) {
        int n = m_tempHistory.size();
        QPolygonF poly;
        poly.reserve(n);

        for (int i = 0; i < n; ++i) {
            double val = m_tempHistory[i];
            double norm = (val - minTemp) / (maxTemp - minTemp);
            if (norm < 0.0) norm = 0.0;
            if (norm > 1.0) norm = 1.0;

            qreal x = graphInner.left() +
                      (graphInner.width() - 1) * ( (n == 1) ? 0.0 : double(i) / (n - 1) );
            qreal y = graphInner.bottom() - norm * (graphInner.height() - 1);
            poly << QPointF(x, y);
        }

        // Line color depends on temperature (same innerGlowColor)
        QPen linePen(innerGlowColor, 2);
        p.setPen(linePen);
        p.setBrush(Qt::NoBrush);
        p.drawPolyline(poly);
    }


    const double hwLimitF = 60.0;
    bool overLimit = (m_currentTempF >= hwLimitF);

    QColor stateColor = overLimit
                            ? QColor(220, 60, 60)    // RED when over limit
                            : QColor(80, 200, 80);   // GREEN when safe

    // Background box
    p.setPen(QColor(200, 200, 210, 160));
    p.setBrush(QColor(0, 0, 0, 180));
    p.drawRoundedRect(hwRect, 12, 12);

    // Circle showing comparator state
    int circleSize = hwRect.height() - 16;
    QRect ledCircle(hwRect.left() + 10,
                    hwRect.top() + 8,
                    circleSize,
                    circleSize);
    p.setBrush(stateColor);
    p.setPen(Qt::NoPen);
    p.drawEllipse(ledCircle);

    // Text label
    p.setPen(Qt::white);
    QFont hwFont = p.font();
    hwFont.setPointSize(9);
    p.setFont(hwFont);

    QString hwText = overLimit
                         ? tr("HW LIMIT: HOT")
                         : tr("HW LIMIT: SAFE");

    QRect textRect = hwRect.adjusted(circleSize + 22, 0, -8, 0);
    p.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, hwText);
    QFont f = p.font();
    f.setPointSize(4);
    //
    // stove graphic
    //
    QRect stoveRect = rightRect;

    // Shadow
    QRect shadowRect = stoveRect.translated(4, 6);
    QRadialGradient shadowGrad(shadowRect.center(), shadowRect.width() * 0.6);
    shadowGrad.setColorAt(0.0, QColor(0, 0, 0, 120));
    shadowGrad.setColorAt(1.0, Qt::transparent);
    p.setBrush(shadowGrad);
    p.setPen(Qt::NoPen);
    p.drawRoundedRect(shadowRect, 24, 24);

    // Glass / body
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

    // Burner locations
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

    const double dangerThresholdF = 100.0;   // visual "hot" threshold for flames
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

        // lit emoji for lit temps
        if (showFlames) {
            int flameSize = static_cast<int>(burnerRadius * (0.7 + 0.6 * t));
            QFont f = p.font();
            f.setPointSize(flameSize);
            p.setFont(f);
            p.setPen(Qt::yellow);
            p.drawText(circleRect, Qt::AlignCenter, QStringLiteral("🔥"));
        }
    }

    // Status band at bottom of stove
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
        status = tr("Heating... %1°F")
                .arg(m_currentTempF, 0, 'f', 1);
        // status = tr("Heating... %1°F (Target: %2°F)")
        //              .arg(m_currentTempF, 0, 'f', 1)
        //              .arg(m_targetTempF);
    }

    if (m_on && m_currentTempF >= dangerThresholdF) {
        p.setPen(QColor(255, 120, 120));
    } else {
        p.setPen(QColor(220, 220, 230));
    }

    QFont statusFont = p.font();
    statusFont.setPointSize(9);
    p.setFont(statusFont);
    p.drawText(statusBand, Qt::AlignCenter, status);
}
