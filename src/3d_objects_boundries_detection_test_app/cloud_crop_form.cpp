#include "cloud_crop_form.h"
#include "ui_cloud_crop_form.h"

cloud_crop_form::cloud_crop_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::cloud_crop_test_form)
{
    this->setFixedSize(477, 332);
    ui->setupUi(this);

    // xyz thresh input
    ui->x_thresh_dsb->setMinimum(0.0);
    ui->x_thresh_dsb->setMaximum(FLT_MAX);
    ui->y_thresh_dsb->setMinimum(0.0);
    ui->y_thresh_dsb->setMaximum(FLT_MAX);
    ui->z_thresh_dsb->setMinimum(0.0);
    ui->z_thresh_dsb->setMaximum(FLT_MAX);

    ui->cloud_in_ledit->setEnabled(false);
    ui->cloud_out_ledit->setEnabled(false);

    ui->launch_test_btn->setEnabled(false);
}

cloud_crop_form::~cloud_crop_form()
{
    delete ui;
}

void cloud_crop_form::on_cloud_in_browse_btn_clicked()
{
    QString file_in_path;
    file_in_path = QFileDialog::getOpenFileName(this,tr("Open Document"), QDir::currentPath(),
                                                  tr("text files (*.txt)"));

    ui->launch_test_btn->setEnabled(false);
    ui->cloud_in_ledit->setEnabled(true);
    ui->cloud_in_ledit->setText(file_in_path);
    ui->cloud_in_ledit->setEnabled(false);

    if (file_in_path.toStdString().compare("")
            && ui->cloud_out_ledit->text().toStdString().compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cloud_crop_form::on_cloud_out_browse_btn_clicked()
{
    QString file_out_path;
    file_out_path = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly
                                                                      | QFileDialog::DontResolveSymlinks);

    ui->launch_test_btn->setEnabled(false);
    ui->cloud_out_ledit->setEnabled(true);
    ui->cloud_out_ledit->setText(file_out_path);
    ui->cloud_out_ledit->setEnabled(false);

    if (ui->cloud_in_ledit->text().toStdString().compare("")
            && file_out_path.toStdString().compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cloud_crop_form::on_launch_test_btn_clicked()
{
    this->setEnabled(false);

    // for when the test is done
    int test_function_return_code;
    QMessageBox info_box;

    test_function_return_code = test::crop_cloud(ui->cloud_in_ledit->text().toStdString(),
                                                 ui->cloud_out_ledit->text().toStdString(),
                                                 ui->x_thresh_dsb->value(), ui->y_thresh_dsb->value(),
                                                 ui->z_thresh_dsb->value());

    if (test_function_return_code)
        info_box.setText("Invalid input.");

    else
        info_box.setText("Operation complete.");

    info_box.exec();
    this->setEnabled(true);
}

void cloud_crop_form::on_cancel_btn_clicked()
{
    this->close();
}

void cloud_crop_form::on_help_btn_clicked()
{
    QMessageBox help_box;
    std::string help_msg;

    help_msg = (std::string("This is a test for the cloud crop function.\n\n") +
                std::string("> cloud in path - path of the input point cloud\n") +
                std::string("> cloud out path - path of directory where the result cloud will be stored\n") +
                std::string("> x, y, z thresh - points that have the absolute value bigger than") +
                std::string(" these floats will be removed; input 0 corresponds to an infinite value\n"));

    help_box.setText(QString::fromUtf8(&help_msg[0], help_msg.size()));
    help_box.exec();
}
