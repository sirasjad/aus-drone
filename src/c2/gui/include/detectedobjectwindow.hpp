#pragma once

#include <QStandardItemModel>
#include <QWidget>

#include <asd_msg/objectImage.h>

namespace Ui {
class DetectedObjectWindow;
}

class DetectedObjectWindow : public QWidget {
  Q_OBJECT

public:
  explicit DetectedObjectWindow(QWidget *parent = nullptr);
  ~DetectedObjectWindow();

public slots:
  void newObject(std::vector<unsigned char> pngData, QString name);

private slots:
  void imageSelected(const QModelIndex &index);
  void reset();

protected:
  void resizeEvent(QResizeEvent *event) override;

private:
  void resizeImage();
  Ui::DetectedObjectWindow *ui;

  QStandardItemModel m_listModel;
  std::vector<QPixmap> m_images;
  QPixmap *m_currentPixmap;
};
