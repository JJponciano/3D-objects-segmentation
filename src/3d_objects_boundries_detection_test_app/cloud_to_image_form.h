#ifndef CLOUD_TO_IMAGE_FORM_H
#define CLOUD_TO_IMAGE_FORM_H

#include "test_lib.h"

#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>

namespace Ui {
class cloud_to_image_form;
}

class cloud_to_image_form : public QDialog
{
    Q_OBJECT

public:
    explicit cloud_to_image_form(QWidget *parent = 0, int img_type = 0);
    ~cloud_to_image_form();

private slots:
    void on_cloud_in_browse_btn_clicked();

    void on_cloud_out_browse_btn_clicked();

    void on_launch_test_btn_clicked();

    void on_cancel_btn_clicked();

    void on_help_btn_clicked();

    void on_image_out_browse_btn_clicked();

private:
    Ui::cloud_to_image_form *ui;
    test::io_data *_iod;
    int _img_type;
};

#endif // CLOUD_TO_IMAGE_FORM_H
