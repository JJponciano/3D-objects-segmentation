#ifndef CONT_DET_FORM_H
#define CONT_DET_FORM_H

#include "test_lib.h"

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>

namespace Ui {
class cont_det_form;
}

class cont_det_form : public QDialog
{
    Q_OBJECT

public:
    explicit cont_det_form(QWidget *parent = 0);
    ~cont_det_form();

private slots:
    void on_image_in_browse_btn_clicked();

    void on_image_out_browse_btn_clicked();

    void on_launch_test_btn_clicked();

    void on_cancel_btn_clicked();

    void on_help_btn_clicked();

private:
    Ui::cont_det_form *ui;
};

#endif // CONT_DET_FORM_H
