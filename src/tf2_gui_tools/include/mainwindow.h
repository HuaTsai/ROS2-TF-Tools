#pragma once

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
  Ui::MainWindow *ui;

  // public slots:
  //   void updateTopicInfo(QString);
};
