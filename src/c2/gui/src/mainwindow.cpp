#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include "roslink/roslink.hpp"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), m_link(argc, argv) {
  ui->setupUi(this);

  qRegisterMetaType<std::vector<unsigned char>>("std::vector<unsigned char>");
  QObject::connect(
      &m_link,
      SIGNAL(livefeedFrame(std::vector<unsigned char>, unsigned, unsigned)),
      this,
      SLOT(livefeedFrame(std::vector<unsigned char>, unsigned, unsigned)));
  QObject::connect(
      &m_link,
      SIGNAL(detectorfeedFrame(std::vector<unsigned char>, unsigned, unsigned)),
      this,
      SLOT(detectorfeedFrame(std::vector<unsigned char>, unsigned, unsigned)));
  QObject::connect(&m_link,
                   SIGNAL(objectDetected(std::vector<unsigned char>, QString)),
                   &m_detectedObjectsWindow,
                   SLOT(newObject(std::vector<unsigned char>, QString)));
  QObject::connect(&m_link, SIGNAL(gearStateChanged(bool)), this,
                   SLOT(updateGearState(bool)));
  QObject::connect(&m_link, SIGNAL(shouldClose()), this, SLOT(close()));

  m_link.start();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::resizeLivefeed() {
  if (!m_livefeedPixmap.isNull())
    ui->livefeedLabel->setPixmap(m_livefeedPixmap.scaled(
        ui->livefeedLabel->size(), Qt::AspectRatioMode::KeepAspectRatio));

  if (!m_detectorfeedPixmap.isNull())
    ui->detectorfeedLabel->setPixmap(m_detectorfeedPixmap.scaled(
        ui->detectorfeedLabel->size(), Qt::AspectRatioMode::KeepAspectRatio));
}

void MainWindow::livefeedFrame(std::vector<unsigned char> data, unsigned width,
                               unsigned height) {
  QImage image(data.data(), static_cast<int>(width), static_cast<int>(height),
               QImage::Format_RGB888);

  if (image.isNull())
    ROS_ERROR("Received livefeed image is null");
  else {
    m_livefeedPixmap = QPixmap::fromImage(image.rgbSwapped());
    resizeLivefeed();
  }
}

void MainWindow::detectorfeedFrame(std::vector<unsigned char> data,
                                   unsigned width, unsigned height) {
  QImage image(data.data(), static_cast<int>(width), static_cast<int>(height),
               QImage::Format_RGB888);

  if (image.isNull())
    ROS_ERROR("Received detector feed image is null");
  else {
    m_detectorfeedPixmap = QPixmap::fromImage(image);
    resizeLivefeed();
  }
}

void MainWindow::on_viewDetectedObjectsButton_clicked() {
  m_detectedObjectsWindow.show();
}

void MainWindow::on_raiseGearButton_clicked() { m_link.setLandingGear(false); }

void MainWindow::on_lowerGearButton_clicked() { m_link.setLandingGear(true); }

void MainWindow::updateGearState(bool isDown) {
  if (isDown) {
    ui->gearStatusLabel->setStyleSheet(
        "background-color: green; color: white;");
    ui->gearStatusLabel->setText("Gear down");
  } else {
    ui->gearStatusLabel->setStyleSheet("background-color: red; color: white;");
    ui->gearStatusLabel->setText("Gear up");
  }
}

void MainWindow::closeEvent(QCloseEvent *) { m_detectedObjectsWindow.close(); }
