#pragma once

#include <QMainWindow>
#include <QLineEdit>
#include <QSlider>
#include <Eigen/Dense>

namespace Ui {
class MainWindow;
}

class CommNode;

class MainWindow : public QMainWindow {

public:
  explicit MainWindow(int id = 0, QWidget *parent = nullptr);
  ~MainWindow();

private:
  std::shared_ptr<Ui::MainWindow> ui;
  std::shared_ptr<CommNode> node;
  Eigen::Isometry3d tf;
  void ComputeAndUpdateTF();
  void SendStaticTF();
  void SendTF();
  void UpdateQuaternionAndMatrix(const Eigen::Isometry3d &tf);

public slots:
  void UpdateLineEditXYZ(QLineEdit *le, QSlider *slider);
  void UpdateSliderXYZ(QSlider *slider, QLineEdit *le);

  void UpdateLineEditRPY(QLineEdit *le, QSlider *slider);
  void UpdateSliderRPY(QSlider *slider, QLineEdit *le);
};
