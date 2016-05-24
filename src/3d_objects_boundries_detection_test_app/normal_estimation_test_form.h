#ifndef NORMAL_ESTIMATION_TEST_FORM_H
#define NORMAL_ESTIMATION_TEST_FORM_H

#include "data.h"
#include "test_lib.h"
#include "../except/invalid_path.h"
#include "../except/invalid_cloud_pointer.h"

#include <string>
#include <exception>

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QErrorMessage>

namespace Ui {
class normal_estimation_test_form;
}

class normal_estimation_test_form : public QDialog
{
    Q_OBJECT

public:
    explicit normal_estimation_test_form(QWidget *parent = 0);
    ~normal_estimation_test_form();

private slots:
    void on_cloud_in_browse_btn_clicked();

    void on_cloud_out_browse_btn_clicked();

    void on_launch_test_btn_clicked();

    void on_cancel_btn_clicked();

private:
    Ui::normal_estimation_test_form *ui;
    data::normal_estimation_data *_ned;
    std::exception_ptr _except_ptr = nullptr;
};

#endif // NORMAL_ESTIMATION_TEST_FORM_H
