#include "cloud_to_image_form.h"
#include "ui_cloud_to_image_form.h"

cloud_to_image_form::cloud_to_image_form(QWidget *parent, int img_type) :
    QDialog(parent),
    ui(new Ui::cloud_to_image_form)
{
    this->setFixedSize(472, 287);
    ui->setupUi(this);
    ui->cloud_in_ledit->setEnabled(false);
    ui->image_out_ledit->setEnabled(false);
    ui->launch_test_btn->setEnabled(false);
    ui->epsilon_dsb->setMinimum(0);
    ui->epsilon_dsb->setMaximum(100.0);
    ui->epsilon_dsb->setSingleStep(0.5);
    ui->epsilon_dsb->setDecimals(1);
    _img_type = img_type;
}

cloud_to_image_form::~cloud_to_image_form()
{
    delete ui;
}

void cloud_to_image_form::on_cloud_in_browse_btn_clicked()
{
    QString file_in_path;
    file_in_path = QFileDialog::getOpenFileName(this,tr("Open Document"), QDir::currentPath(),
                                                  tr("text files (*.txt)"));

    ui->launch_test_btn->setEnabled(false);
    ui->cloud_in_ledit->setEnabled(true);
    ui->cloud_in_ledit->setText(file_in_path);
    ui->cloud_in_ledit->setEnabled(false);

    if (file_in_path.toStdString().compare("")
            && ui->image_out_ledit->text().toStdString().compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cloud_to_image_form::on_image_out_browse_btn_clicked()
{
    QString file_out_path;
    file_out_path = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly
                                                                      | QFileDialog::DontResolveSymlinks);

    ui->launch_test_btn->setEnabled(false);
    ui->image_out_ledit->setEnabled(true);
    ui->image_out_ledit->setText(file_out_path);
    ui->image_out_ledit->setEnabled(false);

    if (ui->cloud_in_ledit->text().toStdString().compare("")
            && file_out_path.toStdString().compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cloud_to_image_form::on_launch_test_btn_clicked()
{
    this->setEnabled(false);

    int test_function_return_code;
    QMessageBox info_box;

    test_function_return_code = test::cloud_to_image(_img_type, ui->cloud_in_ledit->text().toStdString(),
                                                     ui->image_out_ledit->text().toStdString(),
                                                           ui->epsilon_dsb->value());

    if (test_function_return_code)
        info_box.setText("Invalid input.");

    else
        info_box.setText("Operation complete.");

    info_box.exec();
    this->setEnabled(true);
}

void cloud_to_image_form::on_cancel_btn_clicked()
{
    this->close();
}

void cloud_to_image_form::on_help_btn_clicked()
{
    QMessageBox help_box;
    std::string help_msg;

    help_msg = (std::string("This is a test for the cloud crop function.\n\n") +
                std::string("> cloud in path - path of the input point cloud\n") +
                std::string("> image out path - path of directory where the result image will be stored\n") +
                std::string("> epsilon - used for delimiting the x cases of the image") +
                std::string(" this parameter is, the smaller the number of colors in the end cloud\n"));

    help_box.setText(QString::fromUtf8(&help_msg[0], help_msg.size()));
    help_box.exec();
}
