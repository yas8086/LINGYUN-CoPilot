#include <QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setOrganizationName("LoRaTemperature");
    app.setApplicationName("LoRaTemperature");

    MainWindow w;
    w.show();
    return app.exec();
}
