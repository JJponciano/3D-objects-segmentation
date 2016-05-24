#include "normal_estimation_test_form.h"
#include "ui_normal_estimation_test_form.h"

normal_estimation_test_form::normal_estimation_test_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::normal_estimation_test_form)
{
    this->setWindowTitle("Cloud normal estimation test form");
    this->setFixedSize(508, 393);
    ui->setupUi(this);
    _ned = new data::normal_estimation_data;

    // xyz coords input
    ui->x_scale_dsb->setSingleStep(50.0);
    ui->x_scale_dsb->setMinimum(1.0);
    ui->x_scale_dsb->setMaximum(1000.0);
    ui->y_scale_dsb->setSingleStep(50.0);
    ui->y_scale_dsb->setMinimum(1.0);
    ui->y_scale_dsb->setMaximum(1000.0);
    ui->z_scale_dsb->setSingleStep(50.0);
    ui->z_scale_dsb->setMinimum(1.0);
    ui->z_scale_dsb->setMaximum(1000.0);

    // kd-tree input
    ui->radius_dsb->setDecimals(3);
    ui->radius_dsb->setSingleStep(0.01);
    ui->radius_dsb->setMaximum(5);
    ui->max_neighbs_sb->setSingleStep(25);
    ui->max_neighbs_sb->setMaximum(1000.0);

    // max. fragm. depth
    ui->max_fragm_depth_sb->setMaximum(5000);
    ui->max_fragm_depth_sb->setSingleStep(100);

    ui->cloud_in_ledit->setEnabled(false);
    ui->cloud_out_ledit->setEnabled(false);

    ui->launch_test_btn->setEnabled(false);
}

normal_estimation_test_form::~normal_estimation_test_form()
{
    delete ui;
}

void normal_estimation_test_form::on_cloud_in_browse_btn_clicked()
{
    QString cloud_in_path;
    cloud_in_path =  QFileDialog::getOpenFileName(this,tr("Open Document"), QDir::currentPath(),
                                                  tr("text files (*.txt)"));

    ui->launch_test_btn->setEnabled(false);

    _ned->cloud_in_path = cloud_in_path.toStdString();
    ui->cloud_in_ledit->setEnabled(true);
    ui->cloud_in_ledit->setText(cloud_in_path);
    ui->cloud_in_ledit->setEnabled(false);

    if (_ned->cloud_in_path.compare("") && _ned->cloud_out_path.compare(""))
    {
        ui->launch_test_btn->setEnabled(true);
    }
}

void normal_estimation_test_form::on_cloud_out_browse_btn_clicked()
{
    QString cloud_out_path;
    cloud_out_path =  QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly
                                                                      | QFileDialog::DontResolveSymlinks);

    ui->launch_test_btn->setEnabled(false);

    _ned->cloud_out_path = cloud_out_path.toStdString();
    ui->cloud_out_ledit->setEnabled(true);
    ui->cloud_out_ledit->setText(cloud_out_path);
    ui->cloud_out_ledit->setEnabled(false);

    if (_ned->cloud_in_path.compare("") && _ned->cloud_out_path.compare(""))
    {
        ui->launch_test_btn->setEnabled(true);
    }
}

void normal_estimation_test_form::on_launch_test_btn_clicked()
{
    // for when the test is done
    QMessageBox done;

    this->setEnabled(false);

    _ned->radius = ui->radius_dsb->value();
    _ned->max_neighbs = ui->max_neighbs_sb->value();
    _ned->x_scale = ui->x_scale_dsb->value();
    _ned->y_scale = ui->y_scale_dsb->value();
    _ned->z_scale = ui->z_scale_dsb->value();
    _ned->max_fragment_depth = ui->max_fragm_depth_sb->value();

    try
    {
        test_normal_estimation(_ned->cloud_in_path, _ned->cloud_out_path, _ned->radius,
                               _ned->max_neighbs, _ned->x_scale, _ned->y_scale,
                               _ned->z_scale, _ned->max_fragment_depth);

        if (_except_ptr) std::rethrow_exception(_except_ptr);

        done.setText("Cloud normal estimation test completed.");
        done.exec();
    }

    catch (const std::exception& e)
    {
        QErrorMessage q_err_msg;
        QString err_msg;

        err_msg.append(QString::fromUtf8(e.what()));
        q_err_msg.showMessage(err_msg);
    }
}

void normal_estimation_test_form::on_cancel_btn_clicked()
{
    this->close();
}
