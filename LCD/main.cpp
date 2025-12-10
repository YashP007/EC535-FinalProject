#include <QApplication>
#include "stovewidget.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    StoveWidget w;
    w.setWindowTitle("EC535 Stove");
    
    w.showFullScreen();
    // either that or w.show();
    //^^^
    return app.exec();
}
