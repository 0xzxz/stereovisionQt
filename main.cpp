#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    // This is a where the app starts
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
