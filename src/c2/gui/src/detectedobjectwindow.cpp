#include "include/detectedobjectwindow.hpp"
#include "ui_detectedobjectwindow.h"

#include <ros/console.h>

DetectedObjectWindow::DetectedObjectWindow(QWidget *parent)
    : QWidget(parent), ui(new Ui::DetectedObjectWindow),
      m_currentPixmap(nullptr) {
  ui->setupUi(this);

  ui->imageBrowser->setModel(&m_listModel);
  QObject::connect(ui->imageBrowser, SIGNAL(clicked(const QModelIndex &)), this,
                   SLOT(imageSelected(const QModelIndex &)));
}

DetectedObjectWindow::~DetectedObjectWindow() { delete ui; }

void DetectedObjectWindow::newObject(std::vector<unsigned char> pngData,
                                     QString name) {
  QStandardItem *item = new QStandardItem;
  QImage image;
  if (!image.loadFromData(pngData.data(), pngData.size(), "PNG")) {
    ROS_ERROR("Failed to decode object image");
    return;
  }

  // Set up the item
  item->setText(name);
  QPixmap pixmap = QPixmap::fromImage(std::move(image).rgbSwapped());
  item->setIcon(QIcon(pixmap));
  QVariant index = QVariant::fromValue(m_images.size());
  m_images.emplace_back(std::move(pixmap));
  item->setData(index);
  m_listModel.appendRow(item);
}

void DetectedObjectWindow::resizeImage() {
  if (m_currentPixmap != nullptr) {
    ui->imageLabel->setPixmap(
        m_currentPixmap->scaled(ui->imageLabel->size(), Qt::KeepAspectRatio));
  }
}

void DetectedObjectWindow::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  resizeImage();
}

void DetectedObjectWindow::imageSelected(const QModelIndex &index) {
  m_currentPixmap = &m_images[static_cast<size_t>(index.row())];
  resizeImage();
}

void DetectedObjectWindow::reset() {
  m_listModel.clear();
  m_images.clear();
  m_currentPixmap = nullptr;
}
