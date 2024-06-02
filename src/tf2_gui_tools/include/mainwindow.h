#pragma once

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class CommNode;

class MainWindow : public QMainWindow {

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private:
  std::shared_ptr<Ui::MainWindow> ui;
  std::shared_ptr<CommNode> node;

// public slots:
//   void updateTopicInfo(QString);
};
