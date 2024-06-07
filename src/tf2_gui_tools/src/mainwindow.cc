#include "mainwindow.h"
#include "comm_node.h"
#include "ui_mainwindow.h"
#include <Eigen/Dense>

#if (defined(ROS_DISTRO_ROLLING) || defined(ROS_DISTRO_HUMBLE))
#include <tf2_eigen/tf2_eigen.hpp>
#elif defined(ROS_DISTRO_GALACTIC)
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace {

constexpr int kScale = 1000;
constexpr double kXYZMax = 2.0;
constexpr double kXYZMin = -2.0;
constexpr double kRPYMax = 100.0;
constexpr double kRPYMin = -100.0;

void InitXYZ(QSlider *slider, QLineEdit *le) {
  slider->setMinimum(kXYZMin * kScale);
  slider->setMaximum(kXYZMax * kScale);
  slider->setValue(0);
  le->setText("0");
  le->setValidator(new QDoubleValidator(kRPYMin, kRPYMax, 3));
}

std::optional<double> SliderToValueXYZ(int tick) {
  if (tick > kXYZMax * kScale || tick < kXYZMin * kScale)
    return std::nullopt;
  return static_cast<double>(tick) / kScale;
}

std::optional<int> ValueToSliderXYZ(double value) {
  if (value > kXYZMax || value < kXYZMin)
    return std::nullopt;
  return value * kScale;
}

void InitRPY(QSlider *slider, QLineEdit *le) {
  slider->setMinimum(kRPYMin * kScale);
  slider->setMaximum(kRPYMax * kScale);
  slider->setValue(0);
  le->setText("0");
  le->setValidator(new QDoubleValidator(kRPYMin, kRPYMax, 3));
}

std::optional<double> SliderToValueRPY(int tick) {
  if (tick > kRPYMax * kScale || tick < kRPYMin * kScale)
    return std::nullopt;
  return static_cast<double>(tick) / kScale;
}

std::optional<int> ValueToSliderRPY(double value) {
  if (value > kRPYMax || value < kRPYMin)
    return std::nullopt;
  return value * kScale;
}
} // namespace

MainWindow::MainWindow(int id, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->lb_img->setPixmap(QPixmap(":/conan.png")
                            .scaled(ui->lb_img->size(), Qt::KeepAspectRatio,
                                    Qt::SmoothTransformation));
  setTabOrder(ui->le_x, ui->le_y);
  setTabOrder(ui->le_y, ui->le_z);
  setTabOrder(ui->le_z, ui->le_rx);
  setTabOrder(ui->le_rx, ui->le_ry);
  setTabOrder(ui->le_ry, ui->le_rz);

  InitXYZ(ui->slider_x, ui->le_x);
  InitXYZ(ui->slider_y, ui->le_y);
  InitXYZ(ui->slider_z, ui->le_z);
  InitRPY(ui->slider_rx, ui->le_rx);
  InitRPY(ui->slider_ry, ui->le_ry);
  InitRPY(ui->slider_rz, ui->le_rz);

  // clang-format off
  connect(ui->slider_x, &QSlider::valueChanged, this, [this]() { UpdateSliderXYZ(ui->slider_x, ui->le_x); });
  connect(ui->slider_y, &QSlider::valueChanged, this, [this]() { UpdateSliderXYZ(ui->slider_y, ui->le_y); });
  connect(ui->slider_z, &QSlider::valueChanged, this, [this]() { UpdateSliderXYZ(ui->slider_z, ui->le_z); });
  connect(ui->le_x, &QLineEdit::textChanged, this, [this]() { UpdateLineEditXYZ(ui->le_x, ui->slider_x); });
  connect(ui->le_y, &QLineEdit::textChanged, this, [this]() { UpdateLineEditXYZ(ui->le_y, ui->slider_y); });
  connect(ui->le_z, &QLineEdit::textChanged, this, [this]() { UpdateLineEditXYZ(ui->le_z, ui->slider_z); });

  connect(ui->slider_rx, &QSlider::valueChanged, this, [this]() { UpdateSliderRPY(ui->slider_rx, ui->le_rx); });
  connect(ui->slider_ry, &QSlider::valueChanged, this, [this]() { UpdateSliderRPY(ui->slider_ry, ui->le_ry); });
  connect(ui->slider_rz, &QSlider::valueChanged, this, [this]() { UpdateSliderRPY(ui->slider_rz, ui->le_rz); });
  connect(ui->le_rx, &QLineEdit::textChanged, this, [this]() { UpdateLineEditRPY(ui->le_rx, ui->slider_rx); });
  connect(ui->le_ry, &QLineEdit::textChanged, this, [this]() { UpdateLineEditRPY(ui->le_ry, ui->slider_ry); });
  connect(ui->le_rz, &QLineEdit::textChanged, this, [this]() { UpdateLineEditRPY(ui->le_rz, ui->slider_rz); });

  connect(ui->btn_sendtf, &QPushButton::clicked, this, &MainWindow::SendTF);
  connect(ui->btn_sendstf, &QPushButton::clicked, this, &MainWindow::SendStaticTF);
  // clang-format on

  rclcpp::init(0, nullptr);
  node = std::make_shared<CommNode>(id);
  node->start();
  ComputeAndUpdateTF();
}

MainWindow::~MainWindow() { rclcpp::shutdown(); }

void MainWindow::ComputeAndUpdateTF() {
  // Rotation: Rz * Ry * Rx
  // clang-format off
  tf = Eigen::Translation3d(ui->le_x->text().toDouble(), ui->le_y->text().toDouble(), ui->le_z->text().toDouble()) *
       Eigen::AngleAxisd(ui->le_rz->text().toDouble() * M_PI / 180., Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(ui->le_ry->text().toDouble() * M_PI / 180., Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(ui->le_rx->text().toDouble() * M_PI / 180., Eigen::Vector3d::UnitX());
  // clang-format on

  tf2_msgs::msg::TFMessage tf_msg;
  tf_msg.transforms.resize(1);

  if (ui->ckb_inv->isChecked()) {
    UpdateQuaternionAndMatrix(tf.inverse());
    tf_msg.transforms[0] = tf2::eigenToTransform(tf.inverse());
  } else {
    UpdateQuaternionAndMatrix(tf);
    tf_msg.transforms[0] = tf2::eigenToTransform(tf);
  }

  // Stamp is handled by the node so not set here
  tf_msg.transforms[0].header.frame_id = ui->cb_parent->currentText().toStdString();
  tf_msg.transforms[0].child_frame_id = ui->cb_child->currentText().toStdString();
  node->set_tf_msg(tf_msg);
}

void MainWindow::SendStaticTF() {
  ui->btn_sendtf->setEnabled(false);
  ui->btn_sendstf->setEnabled(false);
  ui->cb_parent->setEnabled(false);
  ui->cb_child->setEnabled(false);
  auto font = ui->btn_sendstf->font();
  font.setBold(true);
  ui->btn_sendstf->setFont(font);
  node->PublishStaticTF();
}

void MainWindow::SendTF() {
  ui->btn_sendstf->setEnabled(false);
  ui->btn_sendtf->setEnabled(false);
  ui->cb_parent->setEnabled(false);
  ui->cb_child->setEnabled(false);
  auto font = ui->btn_sendtf->font();
  font.setBold(true);
  ui->btn_sendtf->setFont(font);
  node->PublishTF();
}

void MainWindow::UpdateQuaternionAndMatrix(const Eigen::Isometry3d &tf) {
  Eigen::Quaterniond q(tf.rotation());
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << "[ " << q.w() << ", " << q.x()
     << ", " << q.y() << ", " << q.z() << " ]";
  ui->lb_wxyz->setText(ss.str().c_str());

  auto mtx = tf.matrix();
  ui->lb_mtx00->setText(QString::number(mtx(0, 0), 'f', 3));
  ui->lb_mtx01->setText(QString::number(mtx(0, 1), 'f', 3));
  ui->lb_mtx02->setText(QString::number(mtx(0, 2), 'f', 3));
  ui->lb_mtx03->setText(QString::number(mtx(0, 3), 'f', 3));
  ui->lb_mtx10->setText(QString::number(mtx(1, 0), 'f', 3));
  ui->lb_mtx11->setText(QString::number(mtx(1, 1), 'f', 3));
  ui->lb_mtx12->setText(QString::number(mtx(1, 2), 'f', 3));
  ui->lb_mtx13->setText(QString::number(mtx(1, 3), 'f', 3));
  ui->lb_mtx20->setText(QString::number(mtx(2, 0), 'f', 3));
  ui->lb_mtx21->setText(QString::number(mtx(2, 1), 'f', 3));
  ui->lb_mtx22->setText(QString::number(mtx(2, 2), 'f', 3));
  ui->lb_mtx23->setText(QString::number(mtx(2, 3), 'f', 3));
}

void MainWindow::UpdateSliderXYZ(QSlider *slider, QLineEdit *le) {
  auto value = SliderToValueXYZ(slider->value());
  if (value) {
    le->setText(QString::number(value.value()));
    ComputeAndUpdateTF();
  }
}

void MainWindow::UpdateLineEditXYZ(QLineEdit *le, QSlider *slider) {
  auto tick = ValueToSliderXYZ(le->text().toDouble());
  if (tick) {
    slider->setValue(tick.value());
    ComputeAndUpdateTF();
  }
}

void MainWindow::UpdateSliderRPY(QSlider *slider, QLineEdit *le) {
  auto value = SliderToValueRPY(slider->value());
  if (value) {
    le->setText(QString::number(value.value()));
    ComputeAndUpdateTF();
  }
}

void MainWindow::UpdateLineEditRPY(QLineEdit *le, QSlider *slider) {
  auto tick = ValueToSliderRPY(le->text().toDouble());
  if (tick) {
    slider->setValue(tick.value());
    ComputeAndUpdateTF();
  }
}
