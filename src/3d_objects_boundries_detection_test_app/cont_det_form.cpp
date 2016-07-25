#include "cont_det_form.h"
#include "ui_cont_det_form.h"

cont_det_form::cont_det_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::cont_det_form)
{
    this->setFixedSize(479, 218);
    ui->setupUi(this);
    ui->image_in_ledit->setEnabled(false);
    ui->image_out_ledit->setEnabled(false);
    ui->launch_test_btn->setEnabled(false);
}

cont_det_form::~cont_det_form()
{
    delete ui;
}

void cont_det_form::on_image_in_browse_btn_clicked()
{
    QString file_in_path;
    file_in_path = QFileDialog::getOpenFileName(this,tr("Open Document"), QDir::currentPath(),
                                                  tr("text files (*.pgm)"));

    ui->launch_test_btn->setEnabled(false);

    ui->image_in_ledit->setEnabled(true);
    ui->image_in_ledit->setText(file_in_path);
    ui->image_in_ledit->setEnabled(false);

    if (file_in_path.toStdString().compare("")
            && ui->image_out_ledit->text().toStdString().compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cont_det_form::on_image_out_browse_btn_clicked()
{
    QString file_out_path;
    file_out_path = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly
                                                                      | QFileDialog::DontResolveSymlinks);

    ui->launch_test_btn->setEnabled(false);

    ui->image_out_ledit->setEnabled(true);
    ui->image_out_ledit->setText(file_out_path);
    ui->image_out_ledit->setEnabled(false);

    if (ui->image_in_ledit->text().toStdString().compare("")
            && file_out_path.toStdString().compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cont_det_form::on_launch_test_btn_clicked()
{
    this->setEnabled(false);

    int test_function_return_code = test::detect_contours(ui->image_in_ledit->text().toStdString(),
                                                          ui->image_out_ledit->text().toStdString());
    QMessageBox info_box;

    if (test_function_return_code)
        info_box.setText("Invalid input.");

    else
        info_box.setText("Operation complete.");

    info_box.exec();
    this->setEnabled(true);
}

void cont_det_form::on_cancel_btn_clicked()
{
    this->close();
}

void cont_det_form::on_help_btn_clicked()
{
    QMessageBox help_box;
    std::string help_msg;

    help_msg = (std::string("This is a test for the cloud crop function.\n\n") +
                std::string("> image in path - path of the input grey scale image\n") +
                std::string("> image out path - path of directory where the result image will be stored\n"));

    help_box.setText(QString::fromUtf8(&help_msg[0], help_msg.size()));
    help_box.exec();
}
