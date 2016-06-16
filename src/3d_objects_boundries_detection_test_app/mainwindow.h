#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "normal_estimation_form.h"
#include "cloud_crop_form.h"
#include "cloud_homog_form.h"
#include "cloud_to_image_form.h"

#include <QMainWindow>
#include <QMessageBox>
#include <QErrorMessage>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_test_btn_clicked();

    void on_actionQuit_triggered();

    void on_test_fct_cb_currentIndexChanged(int index);

    void on_actionAbout_triggered();

    void on_quit_btn_clicked();

    void on_next_btn_1_clicked();

    void on_next_btn_2_clicked();

    void how_to();

    void on_how_to_btn_clicked();

    void on_actionInfo_triggered();

private:
    Ui::MainWindow *ui;
    normal_estimation_form *ne_form;
    cloud_crop_form *cc_form;
    cloud_homog_form *ch_form;
    cloud_to_image_form *cti_form;
};

#endif // MAINWINDOW_H
