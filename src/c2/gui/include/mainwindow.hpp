#pragma once

#include "detectedobjectwindow.hpp"
#include "roslink/roslink.hpp"

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);
  ~MainWindow() override;

protected:
  void closeEvent(QCloseEvent *) override;

private slots:
  void livefeedFrame(std::vector<unsigned char>, unsigned width,
                     unsigned height);
  void detectorfeedFrame(std::vector<unsigned char>, unsigned width,
                         unsigned height);
  void updateGearState(bool isDown);

  void on_viewDetectedObjectsButton_clicked();
  void on_raiseGearButton_clicked();
  void on_lowerGearButton_clicked();

private:
  Ui::MainWindow *ui;
  QPixmap m_livefeedPixmap, m_detectorfeedPixmap;
  RosLink m_link;
  DetectedObjectWindow m_detectedObjectsWindow;

  void resizeLivefeed();
};
