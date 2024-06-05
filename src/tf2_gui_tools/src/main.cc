#include "mainwindow.h"
#include <QApplication>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: ros2 run tf2_gui_tools main <0, 1, 2>" << std::endl;
    return 1;
  }
  QApplication app(argc, argv);
  MainWindow mainWindow(std::stoi(argv[1]));
  mainWindow.show();
  return app.exec();
}
