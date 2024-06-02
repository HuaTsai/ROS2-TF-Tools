#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "comm_node.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  rclcpp::init(0, nullptr);
  node = std::make_shared<CommNode>();
  node->start();
}

MainWindow::~MainWindow() {
  rclcpp::shutdown();
}

