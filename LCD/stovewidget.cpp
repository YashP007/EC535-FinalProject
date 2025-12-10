#include "stovewidget.h"

#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QFont>
#include <QTimer>
#include <QtMath>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <QDebug>
// Adafruit IO disabled - no web connectivity
// #include <QNetworkRequest>
// #include <QNetworkReply>
// #include <QJsonDocument>
// #include <QJsonObject>

StoveWidget::StoveWidget(QWidget *parent)
    : QWidget(parent),
    m_on(false),
    m_currentTempF(70.0),
    m_targetTempF(200),
    m_softwareLimitF(300),
    m_powerButton(nullptr),
    m_tempSlider(nullptr),
    m_softwareLimitSlider(nullptr),
    m_tempLabel(nullptr),
    m_statusLabel(nullptr),
    m_emergencyLabel(nullptr),
    m_simTimer(nullptr),
    m_ledFd(-1),
    m_lastTempAbove(false),
    m_compFd(-1),
    m_comparatorTriggered(false),
    m_compTimer(nullptr)
{
    // Adafruit IO disabled - no web connectivity
    // m_aioUser = "ldsj";
    // m_aioKey  = "aio_Fmad035K4dRzhUgbvKHHZfjT9QF7";

    // initial tempertaure
    m_tempHistory.reserve(0);
    m_tempHistory.append(m_currentTempF);

    //power button - smaller and positioned top right
    m_powerButton = new QPushButton(QStringLiteral("⏻"), this);
    m_powerButton->setCheckable(true);
    m_powerButton->setFixedSize(27, 27);  // 0.75x smaller (36 * 0.75 = 27)
    m_powerButton->setToolTip(tr("Stove OFF"));

    m_powerButton->setStyleSheet(
        "QPushButton { "
        "  border-radius: 13px;"  // Adjusted for smaller size (27/2)
        "  border: 2px solid #4a90e2;"  // Professional blue
        "  background-color: #2c3e50;"  // Professional dark blue-gray
        "  color: #ecf0f1;"  // Light gray text
        "  font-size: 14px;"  // Adjusted for smaller button
        "}"
        "QPushButton:checked { "
        "  background-color: #27ae60;"  // Professional green
        "  border-color: #2ecc71;"
        "}"
        );

    // slider for temperature (realistic stove range: 70-500°F)
    // Will be positioned under stove in paintEvent
    m_tempSlider = new QSlider(Qt::Horizontal, this);
    m_tempSlider->setRange(70, 500);       // realistic stove temperature range
    m_tempSlider->setValue(m_targetTempF);
    m_tempSlider->hide();  // Hide initially, will be positioned in paintEvent
    
    // slider for software limit (70-500°F) - hidden, drawn manually in paintEvent
    m_softwareLimitSlider = new QSlider(Qt::Horizontal, this);
    m_softwareLimitSlider->setRange(70, 500);
    m_softwareLimitSlider->setValue(m_softwareLimitF);
    m_softwareLimitSlider->hide();  // Hide widget, we draw it manually in paintEvent

    // Text label for current/target temperature - stacked format, positioned next to button
    m_tempLabel = new QLabel(this);
    m_tempLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_tempLabel->setStyleSheet("color: #bdc3c7; font-weight: bold; font-size: 9px;");  // Professional light gray
    
    // Status label (above slider, full width) - smaller
    m_statusLabel = new QLabel(this);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    m_statusLabel->setStyleSheet("color: white; font-weight: bold; font-size: 10px; padding: 3px;");
    m_statusLabel->setWordWrap(true);

    //  title and power button on top,

    // slider row at the bottom.
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Emergency label (shown when HW limit is met) - overlay on top, doesn't push content
    m_emergencyLabel = new QLabel(this);
    m_emergencyLabel->setAlignment(Qt::AlignCenter);
    m_emergencyLabel->setStyleSheet("color: #ff0000; font-weight: bold; font-size: 18px; padding: 6px; background-color: rgba(0,0,0,220);");
    m_emergencyLabel->setText(tr("🚨 CALLING 911 AND THE BFD 🚨"));
    m_emergencyLabel->hide();  // Hidden by default
    m_emergencyLabel->raise();  // Ensure it's on top
    m_emergencyLabel->setAttribute(Qt::WA_TransparentForMouseEvents, true);  // Don't block mouse events
    
    QHBoxLayout *topRow = new QHBoxLayout;
    QLabel *title = new QLabel(tr("SafeStoves"), this);
    title->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    title->setStyleSheet("color: #ecf0f1;");  // Professional light gray
    QFont titleFont = title->font();
    titleFont.setPointSize(11);  // Smaller to avoid overlapping with graph
    titleFont.setBold(true);
    title->setFont(titleFont);

    // Add title, then stretch, then temp label (closer to button), then power button
    topRow->addWidget(title);
    topRow->addStretch();
    topRow->addWidget(m_tempLabel);
    topRow->addSpacing(5);  // Small spacing between temp label and button
    topRow->addWidget(m_powerButton, 0, Qt::AlignRight | Qt::AlignTop);  // Align to top right

    // Emergency label is positioned absolutely, not in layout
    mainLayout->addLayout(topRow);
    mainLayout->setSpacing(0);  // No spacing between elements
    mainLayout->setContentsMargins(5, 5, 5, 5);  // Small padding for professional look
    mainLayout->addSpacing(1);  // Minimal spacing before graph/stove area
    mainLayout->addStretch();

    // Status label moved to bottom (where slider was)
    // Target temp slider will be drawn under stove in paintEvent
    mainLayout->addWidget(m_statusLabel);

    setLayout(mainLayout);

    connect(m_powerButton, &QPushButton::clicked,
            this, &StoveWidget::toggleStove);
    connect(m_tempSlider, &QSlider::valueChanged,
            this, &StoveWidget::setTargetTemp);
    connect(m_softwareLimitSlider, &QSlider::valueChanged,
            this, &StoveWidget::setSoftwareLimit);

    //LED
    initLedDevice();
    
    // Hardware Comparator
    initComparatorDevice();
    if (m_compFd >= 0) {
        // Timer to periodically read comparator status
        m_compTimer = new QTimer(this);
        m_compTimer->setInterval(500); // Check every 500ms
        connect(m_compTimer, &QTimer::timeout, this, &StoveWidget::readComparatorStatus);
        m_compTimer->start();
        
        // Read initial status
        readComparatorStatus();
    }
    
    syncLedWithSimState();

   //simulation

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(500); // 0.5s
    m_simTimer->start(); // Start immediately so temp can cool when off
    connect(m_simTimer, &QTimer::timeout, [this]() {
        // If HW limit is reached, keep temperature at HW limit level
        if (m_comparatorTriggered) {
            m_currentTempF = m_hardwareLimitF;  // Lock at 450°F
        } else if (m_on) {
            // Stove is ON: move current temp toward target temp
            if (m_currentTempF < m_targetTempF)
                m_currentTempF += 8.0;  // Heat up faster
            else if (m_currentTempF > m_targetTempF)
                m_currentTempF -= 2.0;  // Cool slightly if above target
        } else {
            // Stove is OFF: temperature decreases toward room temp (70°F)
            if (m_currentTempF > 70.0)
                m_currentTempF -= 3.0;  // Cool down
            else if (m_currentTempF < 70.0)
                m_currentTempF = 70.0;  // Don't go below room temp
        }

        // Log this temp in the history for the graph
        m_tempHistory.append(m_currentTempF);
        if (m_tempHistory.size() > 120) {
            m_tempHistory.remove(0);   // keep most recent samples
        }

        updateTempLabel();
        updateStatusLabel();
        update(); // repaint

        syncLedWithSimState();
        // sendTempToAdafruit();  // Adafruit IO disabled
    });

    updateTempLabel();
    updateStatusLabel();
    resize(480, 272); // for LCD to fit
    
    // Position emergency label initially (will be updated in updateStatusLabel and resizeEvent)
    m_emergencyLabel->setGeometry(0, 0, 480, 32);  // Bigger height for larger text
}

void StoveWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    
    // Update emergency label position when widget is resized
    if (m_emergencyLabel && m_emergencyLabel->isVisible()) {
        m_emergencyLabel->setGeometry(0, 0, width(), 32);  // Bigger height for larger text
        m_emergencyLabel->raise();
    }
}

void StoveWidget::toggleStove()
{
    m_on = !m_on;

    if (m_on) {
        m_powerButton->setToolTip(tr("Stove ON"));
    } else {
        m_powerButton->setToolTip(tr("Stove OFF"));
    }
    
    // Always run simulation timer (needed for cooling when off)
    if (!m_simTimer->isActive()) {
        m_simTimer->start();
    }

    updateTempLabel();
    updateStatusLabel();
    update(); // repaint

    syncLedWithSimState();
}

void StoveWidget::setTargetTemp(int temp)
{
    m_targetTempF = temp;
    updateTempLabel();
    updateStatusLabel();
    update();

    syncLedWithSimState();
}

void StoveWidget::setSoftwareLimit(int temp)
{
    m_softwareLimitF = temp;
    updateStatusLabel();
    update();  // Repaint to update the drawn slider
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
    updateStatusLabel();
    update();

    syncLedWithSimState();
    // sendTempToAdafruit();  // Adafruit IO disabled
}

void StoveWidget::updateTempLabel()
{
    // Stack the text vertically with newline
    m_tempLabel->setText(
        tr("Current Temp: %1°F\nTarget Temp: %2°F")
            .arg(m_currentTempF, 0, 'f', 1)
            .arg(m_targetTempF)
        );
}

void StoveWidget::updateStatusLabel()
{
    QString status;
    QColor statusColor = QColor(220, 220, 230);  // Default white
    
    // Check if software limit (300°F) is exceeded
    bool swLimitExceeded = m_on && m_currentTempF >= m_softwareLimitF;
    // Hardware limit is m_comparatorTriggered (450°F)
    
    // Show/hide and position emergency label as overlay at top
    if (m_comparatorTriggered) {
        m_emergencyLabel->show();
        // Position absolutely at top center, doesn't affect layout - bigger height for larger text
        m_emergencyLabel->setGeometry(0, 0, width(), 32);
        m_emergencyLabel->raise();  // Ensure it's on top
    } else {
        m_emergencyLabel->hide();
    }
    
    if (!m_on) {
        // Dynamic message based on temperature
        QString tempStatus;
        if (m_currentTempF >= 150.0) {
            tempStatus = tr("Stove is OFF (Hot to Touch)");
            statusColor = QColor(241, 196, 15);  // Amber for hot
        } else if (m_currentTempF >= 100.0) {
            tempStatus = tr("Stove is OFF (Warm to Touch)");
            statusColor = QColor(52, 152, 219);  // Blue for warm
        } else {
            tempStatus = tr("Stove is OFF (Cool to Touch)");
            statusColor = QColor(149, 165, 166);  // Gray for cool
        }
        status = tempStatus;
    } else if (m_comparatorTriggered) {
        // Hardware limit exceeded (most critical - 450°F)
        status = tr("⚠️ CRITICAL: HARDWARE LIMIT EXCEEDED! Temperature: %1°F (Limit: %2°F)")
                     .arg(m_currentTempF, 0, 'f', 1)
                     .arg(m_hardwareLimitF);
        statusColor = QColor(231, 76, 60);  // Professional red
    } else if (swLimitExceeded) {
        // Software limit exceeded (300°F reached)
        status = tr("⚠️ Software Limit Reached: %1°F (Limit: %2°F)")
                     .arg(m_currentTempF, 0, 'f', 1)
                     .arg(m_softwareLimitF);
        statusColor = QColor(241, 196, 15);  // Professional amber
    } else {
        status = tr("Heating... Current: %1°F → Target: %2°F")
                     .arg(m_currentTempF, 0, 'f', 1)
                     .arg(m_targetTempF);
        statusColor = QColor(52, 152, 219);  // Professional blue
    }
    
    m_statusLabel->setText(status);
    m_statusLabel->setStyleSheet(QString("color: %1; font-weight: bold; font-size: 10px; padding: 3px;")
                                  .arg(statusColor.name()));
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

    // Blue is on off type (stove state)
    if (m_on) {
        writeLedCommand("stove_on\n");
    } else {
        writeLedCommand("stove_off\n");
    }

    // Red LED controlled by hardware comparator
    // Note: With active low comparator, triggered = LOW = temp_above
    bool nowAbove = m_comparatorTriggered;

    if (nowAbove != m_lastTempAbove) {
        if (nowAbove) {
            writeLedCommand("temp_above\n");
        } else {
            writeLedCommand("temp_below\n");
        }
        m_lastTempAbove = nowAbove;
    }
}

// Adafruit IO disabled - no web connectivity
/*
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
*/

// Hardware Comparator helpers

void StoveWidget::initComparatorDevice()
{
    m_compFd = ::open("/dev/mytempsensor_comp", O_RDONLY | O_NONBLOCK);
    if (m_compFd < 0) {
        qWarning() << "Failed to open /dev/mytempsensor_comp:" << strerror(errno);
        return;
    }
    qDebug() << "Comparator device opened successfully";
}

void StoveWidget::readComparatorStatus()
{
    if (m_compFd < 0)
        return;

    char buffer[512];
    ssize_t n = ::read(m_compFd, buffer, sizeof(buffer) - 1);
    
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available, try again
            return;
        }
        if (errno != EINTR) {
            qWarning() << "read(/dev/mytempsensor_comp) failed:" << strerror(errno);
        }
        return;
    }

    if (n == 0) {
        // EOF - kernel reset position, try reading again
        n = ::read(m_compFd, buffer, sizeof(buffer) - 1);
        if (n <= 0)
            return;
    }

    buffer[n] = '\0';
    
    // Parse status string: "comparator=0 comparator_gpio=26 period=15000 ms triggered=1"
    // Note: With active low comparator:
    //   - comparator=0 (LOW) means threshold exceeded (currently triggered)
    //   - comparator=1 (HIGH) means safe (below threshold)
    //   - triggered=1 means falling edge occurred (IQR fired: high->low transition)
    QString statusStr = QString::fromUtf8(buffer);
    
    bool oldTriggered = m_comparatorTriggered;
    bool newTriggered = false;
    
    // Priority: Current comparator state is most reliable indicator
    // The triggered flag indicates an edge event occurred, but current state shows actual condition
    if (statusStr.contains("comparator=0")) {
        newTriggered = true;  // LOW = threshold currently exceeded (danger)
    } else if (statusStr.contains("comparator=1")) {
        newTriggered = false; // HIGH = safe (below threshold)
    } else if (statusStr.contains("triggered=1")) {
        // Fallback: if triggered=1 but no comparator state, assume triggered
        // This handles the case where IQR just fired
        newTriggered = true;
    }
    
    m_comparatorTriggered = newTriggered;
    
    // If state changed, update LED and repaint
    if (oldTriggered != m_comparatorTriggered) {
        // When comparator fires (HW limit exceeded), instantly set temp to 450°F
        if (m_comparatorTriggered && !oldTriggered) {
            m_currentTempF = m_hardwareLimitF;  // Instant jump to 450°F
            // Add this point to history to show the jump
            m_tempHistory.append(m_currentTempF);
            if (m_tempHistory.size() > 120) {
                m_tempHistory.remove(0);
            }
            updateTempLabel();
        }
        syncLedWithSimState();
        updateStatusLabel(); // Update status text
        update(); // Repaint to update HW limit display and flames
        qDebug() << "Comparator state changed:" << (m_comparatorTriggered ? "DANGER (triggered)" : "SAFE");
    }
}


void StoveWidget::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

   //background - professional color scheme

    {
        QLinearGradient bgGrad(rect().topLeft(), rect().bottomLeft());
        bgGrad.setColorAt(0.0, QColor(26, 32, 38));  // Professional dark blue-gray
        bgGrad.setColorAt(1.0, QColor(34, 40, 48));  // Slightly lighter dark blue-gray
        p.fillRect(rect(), bgGrad);
    }

    // Define a content area between top widgets and bottom slider row
    // Increased top margin to move content down and reduce crowding at top
    int topMargin    = 35;               // Increased to move content down (was 25)
    int bottomMargin = 35; // Fixed margin for status label at bottom

    QRect contentRect(20,
                      topMargin,
                      width() - 40,
                      height() - topMargin - bottomMargin);

    if (contentRect.height() <= 0 || contentRect.width() <= 0)
        return;

    // Split content area: left for graph, right for stove
    // Make stove smaller to accommodate slider underneath
    int graphWidth = static_cast<int>(contentRect.width() * 0.58);  // Graph gets 58% (more space)
    int stoveWidth = contentRect.width() - graphWidth - 10;  // Stove gets remaining minus gap
    
    QRect leftRect(contentRect.left(),
                   contentRect.top(),
                   graphWidth,
                   contentRect.height());

    // Stove area - need to leave room for slider underneath
    int stoveHeight = contentRect.height() - 30;  // Leave 30px for slider below stove
    QRect rightRect(leftRect.right() + 10,
                    contentRect.top(),
                    stoveWidth,
                    stoveHeight);


    const double minTemp = 70.0;   // Realistic stove minimum (room temp)
    const double maxTemp = 500.0;  // Realistic stove maximum
    double t = (m_currentTempF - minTemp) / (maxTemp - minTemp);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    //graph
    int r = static_cast<int>(80  + t * 175); // 80 → 255
    int g = static_cast<int>(80  + t *  60); // 80 → 140
    int b = static_cast<int>(80  - t *  80); // 80 → 0
    QColor innerGlowColor(r, g, b);

    // Limit indicators box underneath graph (two rows: HW and SW limits + slider)
    int limitBoxHeight = 85;      // Reduced height (was 100)
    int graphBottomGap = 2;       // Minimal spacing between graph and limit box (was 3)

    QRect limitRect(leftRect.left() + 10,
                    leftRect.bottom() - limitBoxHeight,
                    leftRect.width() - 20,
                    limitBoxHeight);
    
    // Position the SW limit slider widget over the drawn slider area
    if (m_softwareLimitSlider) {
        int rowHeight = (limitBoxHeight - 8) / 3;
        int sliderRowY = limitRect.top() + 2 * rowHeight + 4;
        int sliderX = limitRect.left() + 68;  // Match drawn position
        int sliderWidth = limitRect.width() - 120;  // Reduced width to match drawn slider
        // Align slider widget exactly with drawn slider (sliderY is top + 8, so widget should be at sliderRowY + 8)
        m_softwareLimitSlider->setGeometry(sliderX, sliderRowY + 8, sliderWidth, 12);
        m_softwareLimitSlider->setStyleSheet(
            "QSlider {"
            "  background: transparent;"
            "}"
            "QSlider::groove:horizontal {"
            "  border: none;"
            "  background: transparent;"
            "  height: 12px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: rgb(127, 140, 141);"
            "  border: 1px solid rgb(149, 165, 166);"
            "  width: 16px;"
            "  margin: 0px;"
            "  border-radius: 6px;"
            "}"
            "QSlider::handle:horizontal:hover {"
            "  background: rgb(127, 140, 141);"
            "}"
            "QSlider::sub-page:horizontal {"
            "  background: transparent;"
            "}"
            "QSlider::add-page:horizontal {"
            "  background: transparent;"
            "}"
            "QSlider::tick-mark:horizontal {"
            "  background: transparent;"
            "}"
        );
        m_softwareLimitSlider->show();  // Show it so it's interactive
    }

    // Make graph bigger by using more of the available space
    // Make graph bigger by moving it down and using more vertical space
    // Reduced top padding to move graph down
    QRect graphRect(leftRect.left() + 8,
                    leftRect.top() + 4,  // Reduced from 8 to move graph down
                    leftRect.width() - 16,
                    leftRect.height() - limitBoxHeight - graphBottomGap - 12);  // Reduced bottom padding

    // Graph background
    p.setPen(QPen(QColor(150, 150, 160), 1));
    p.setBrush(QColor(25, 25, 35));
    p.drawRoundedRect(graphRect, 8, 8);

    QRect graphInner = graphRect.adjusted(6, 6, -6, -6);

    // Draw axes - professional colors
    p.setPen(QColor(90, 100, 110));  // Professional medium gray
    p.drawLine(graphInner.bottomLeft(), graphInner.bottomRight()); // x-axis
    p.drawLine(graphInner.bottomLeft(), graphInner.topLeft());     // y-axis

    // Reference lines for limits
    double hwLimitTemp = 450.0;  // Hardware limit (comparator threshold)
    double swLimitTemp = static_cast<double>(m_softwareLimitF);  // Software limit

    // Draw hardware limit line (450°F)
    double normHwLimit = (hwLimitTemp - minTemp) / (maxTemp - minTemp);
    if (normHwLimit < 0.0) normHwLimit = 0.0;
    if (normHwLimit > 1.0) normHwLimit = 1.0;
    qreal yHwLimit = graphInner.bottom() - normHwLimit * (graphInner.height() - 1);
    
    // Draw software limit line (300°F)
    double normSwLimit = (swLimitTemp - minTemp) / (maxTemp - minTemp);
    if (normSwLimit < 0.0) normSwLimit = 0.0;
    if (normSwLimit > 1.0) normSwLimit = 1.0;
    qreal ySwLimit = graphInner.bottom() - normSwLimit * (graphInner.height() - 1);

    // Draw hardware limit line - professional red
    QPen hwLimitPen(QColor(231, 76, 60), 2);  // Professional red
    hwLimitPen.setStyle(Qt::DashLine);
    p.setPen(hwLimitPen);
    p.drawLine(QPointF(graphInner.left(),  yHwLimit),
               QPointF(graphInner.right(), yHwLimit));
    
    // Draw software limit line - professional orange
    QPen swLimitPen(QColor(241, 196, 15), 2);  // Professional amber/orange
    swLimitPen.setStyle(Qt::DashLine);
    p.setPen(swLimitPen);
    p.drawLine(QPointF(graphInner.left(),  ySwLimit),
               QPointF(graphInner.right(), ySwLimit));

    // Limit line labels removed - values shown in limit widget below



    // Y-axis tick labels (min / max / mid) - professional colors
    p.setPen(QColor(200, 210, 220));  // Professional light gray
    QFont graphFont = p.font();
    graphFont.setPointSize(6);  // Reduced from 7
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


    // Two-row limit indicator widget - professional colors
    // Background box
    p.setPen(QColor(70, 80, 90, 180));  // Professional dark gray border
    p.setBrush(QColor(30, 36, 42, 220));  // Professional dark background
    p.drawRoundedRect(limitRect, 12, 12);

    // Split into three sections: HW row, SW row, and slider row
    int rowHeight = (limitRect.height() - 8) / 3;  // Three equal sections
    QRect hwRowRect(limitRect.left() + 8, limitRect.top() + 4, 
                    limitRect.width() - 16, rowHeight - 2);
    QRect swRowRect(limitRect.left() + 8, limitRect.top() + rowHeight + 2,
                    limitRect.width() - 16, rowHeight - 2);
    QRect sliderRowRect(limitRect.left() + 8, limitRect.top() + 2 * rowHeight + 4,
                        limitRect.width() - 16, rowHeight - 2);

    // Hardware limit row (top row)
    // Show HW limit state regardless of stove on/off (hardware safety device)
    bool hwLimitExceeded = m_comparatorTriggered;
    QColor hwColor = hwLimitExceeded 
                        ? QColor(231, 76, 60)    // Professional red when exceeded
                        : QColor(39, 174, 96);  // Professional green when safe
    
    // Icon/circle for HW limit - smaller to match reduced text
    int iconSize = hwRowRect.height() - 6;  // Slightly smaller icon
    QRect hwIconRect(hwRowRect.left() + 4, hwRowRect.top() + 4, iconSize, iconSize);
    p.setBrush(hwColor);
    p.setPen(QPen(hwColor.darker(150), 2));
    p.drawEllipse(hwIconRect);
    
    // Add warning icon symbol when exceeded
    if (hwLimitExceeded) {
        p.setPen(QPen(Qt::white, 2));
        p.setFont(QFont("Arial", iconSize/2, QFont::Bold));
        p.drawText(hwIconRect, Qt::AlignCenter, "!");
    }

    // HW limit text - smaller font
    p.setPen(Qt::white);
    QFont limitFont = p.font();
    limitFont.setPointSize(7);  // Reduced from 8 (1-2 points smaller)
    limitFont.setBold(true);
    p.setFont(limitFont);
    QString hwText = tr("Hardware Limit: %1°F").arg(m_hardwareLimitF);
    QRect hwTextRect = hwRowRect.adjusted(iconSize + 12, 0, -4, 0);
    p.drawText(hwTextRect, Qt::AlignVCenter | Qt::AlignLeft, hwText);

    // Software limit row (bottom row)
    bool swLimitExceeded = m_on && m_currentTempF >= m_softwareLimitF;
    QColor swColor = swLimitExceeded
                        ? QColor(241, 196, 15)  // Professional amber when exceeded
                        : QColor(52, 152, 219); // Professional blue when safe
    
    // Circle for SW limit
    QRect swIconRect(swRowRect.left() + 4, swRowRect.top() + 4, iconSize, iconSize);
    p.setBrush(swColor);
    p.setPen(QPen(swColor.darker(150), 2));
    p.drawEllipse(swIconRect);

    // SW limit text - professional colors
    p.setPen(QColor(236, 240, 241));  // Professional light gray
    limitFont.setPointSize(7);  // Same size as HW limit (reduced from 8)
    p.setFont(limitFont);
    QString swText = tr("Software Limit: %1°F").arg(m_softwareLimitF);
    QRect swTextRect = swRowRect.adjusted(iconSize + 12, 0, -4, 0);
    p.drawText(swTextRect, Qt::AlignVCenter | Qt::AlignLeft, swText);
    
    // Draw SW limit slider inside the widget - professional colors
    p.setPen(QColor(236, 240, 241));  // Professional light gray
    QFont sliderFont = p.font();
    sliderFont.setPointSize(6);  // Reduced from 7
    p.setFont(sliderFont);
    p.drawText(sliderRowRect.left(), sliderRowRect.top() + 10, tr("SW Limit:"));
    
    // Draw slider track - smaller width, professional colors
    int sliderX = sliderRowRect.left() + 60;
    int sliderY = sliderRowRect.top() + 8;
    int sliderWidth = sliderRowRect.width() - 120;  // Reduced width (was 65, now 120 to make smaller)
    int sliderHeight = 12;
    
    QRect sliderTrack(sliderX, sliderY, sliderWidth, sliderHeight);
    p.setPen(QPen(QColor(70, 80, 90), 1));  // Professional dark gray
    p.setBrush(QColor(40, 46, 52));  // Professional dark background
    p.drawRoundedRect(sliderTrack, 6, 6);
    
    // Draw slider thumb - professional colors
    int sliderRange = m_softwareLimitSlider->maximum() - m_softwareLimitSlider->minimum();
    int sliderValue = m_softwareLimitSlider->value() - m_softwareLimitSlider->minimum();
    int thumbPos = sliderX + (sliderWidth - 16) * sliderValue / sliderRange;
    
    QRect sliderThumb(thumbPos, sliderY, 16, sliderHeight);
    p.setPen(QPen(QColor(149, 165, 166), 1));  // Professional medium gray
    p.setBrush(QColor(127, 140, 141));  // Professional gray
    p.drawRoundedRect(sliderThumb, 6, 6);
    
    // Value text removed - already shown in "Software Limit: XXX°F" above

    //
    // stove graphic (smaller to fit slider underneath)
    //
    QRect stoveRect = rightRect;
    
    // Draw target temp slider under the stove (same width as stove)
    int targetSliderY = rightRect.bottom() + 5;
    int targetSliderHeight = 20;
    QRect targetSliderArea(rightRect.left(), targetSliderY, rightRect.width(), targetSliderHeight);
    
    // Draw slider label
    p.setPen(QColor(236, 240, 241));  // Professional light gray
    QFont sliderLabelFont = p.font();
    sliderLabelFont.setPointSize(8);
    sliderLabelFont.setBold(true);
    p.setFont(sliderLabelFont);
    p.drawText(targetSliderArea.left(), targetSliderArea.top() + 12, tr("Target Temp:"));
    
    // Draw slider track
    int targetSliderTrackX = targetSliderArea.left() + 80;
    int targetSliderTrackY = targetSliderArea.top() + 6;
    int targetSliderTrackWidth = targetSliderArea.width() - 85;
    int targetSliderTrackHeight = 8;
    
    QRect targetSliderTrack(targetSliderTrackX, targetSliderTrackY, targetSliderTrackWidth, targetSliderTrackHeight);
    p.setPen(QPen(QColor(70, 80, 90), 1));  // Professional dark gray
    p.setBrush(QColor(40, 46, 52));  // Professional dark background
    p.drawRoundedRect(targetSliderTrack, 4, 4);
    
    // Draw slider thumb
    int targetSliderRange = m_tempSlider->maximum() - m_tempSlider->minimum();
    int targetSliderValue = m_tempSlider->value() - m_tempSlider->minimum();
    int targetThumbPos = targetSliderTrackX + (targetSliderTrackWidth - 12) * targetSliderValue / targetSliderRange;
    
    QRect targetSliderThumb(targetThumbPos, targetSliderTrackY, 12, targetSliderTrackHeight);
    p.setPen(QPen(QColor(149, 165, 166), 1));  // Professional medium gray
    p.setBrush(QColor(127, 140, 141));  // Professional gray
    p.drawRoundedRect(targetSliderThumb, 4, 4);
    
    // Position the actual slider widget over the drawn slider for interaction
    if (m_tempSlider) {
        m_tempSlider->setGeometry(targetSliderTrackX, targetSliderTrackY, targetSliderTrackWidth, targetSliderTrackHeight);
        m_tempSlider->setStyleSheet(
            "QSlider::groove:horizontal {"
            "  border: none;"
            "  background: transparent;"
            "  height: 8px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: rgb(127, 140, 141);"
            "  border: 1px solid rgb(149, 165, 166);"
            "  width: 12px;"
            "  margin: 0px;"
            "  border-radius: 4px;"
            "}"
            "QSlider::sub-page:horizontal {"
            "  background: transparent;"
            "}"
            "QSlider::add-page:horizontal {"
            "  background: transparent;"
            "}"
        );
        m_tempSlider->show();  // Show the slider so it's interactive
    }

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

    // Show flames when hardware limit (450°F) OR software limit (300°F) is exceeded
    // Flames only show when stove is on AND a limit is exceeded
    bool hwLimitForFlames = m_on && m_comparatorTriggered;
    bool showFlames = hwLimitForFlames || swLimitExceeded;

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

        // Show flames when HW limit (450°F) OR SW limit (300°F) is exceeded
        if (showFlames) {
            // Make flames more visible with larger size and glow effect
            int flameSize = static_cast<int>(burnerRadius * 1.2);
            QFont f = p.font();
            f.setPointSize(flameSize);
            f.setBold(true);
            p.setFont(f);
            
            // Different intensity based on which limit is exceeded
            if (hwLimitForFlames) {
                // HW limit: brighter, more intense flames
                p.setPen(QPen(QColor(255, 200, 0, 150), 1));
                p.drawText(circleRect.adjusted(-2, -2, 2, 2), Qt::AlignCenter, QStringLiteral("🔥"));
                p.setPen(QPen(QColor(255, 80, 0), 3));
            } else {
                // SW limit: slightly milder flames
                p.setPen(QPen(QColor(255, 220, 100, 120), 1));
                p.drawText(circleRect.adjusted(-2, -2, 2, 2), Qt::AlignCenter, QStringLiteral("🔥"));
                p.setPen(QPen(QColor(255, 150, 50), 2));
            }
            
            // Draw main flame
            p.drawText(circleRect, Qt::AlignCenter, QStringLiteral("🔥"));
        }
    }

    // Status band removed - now using status label widget above slider
}
