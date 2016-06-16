#include "cloud_homog_form.h"
#include "ui_cloud_homog_form.h"

cloud_homog_form::cloud_homog_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::cloud_homogenization_test_form)
{
    this->setFixedSize(476, 295);
    ui->setupUi(this);
    ui->cloud_in_ledit->setEnabled(false);
    ui->cloud_out_ledit->setEnabled(false);
    ui->epsilon_sb->setMinimum(1);
    ui->epsilon_sb->setMaximum(255);
    ui->epsilon_sb->setSingleStep(25);
    _ed = new test::epsilon_data;
}

cloud_homog_form::~cloud_homog_form()
{
    delete ui;
}

void cloud_homog_form::on_cloud_in_browse_btn_clicked()
{
    QString file_in_path;
    file_in_path = QFileDialog::getOpenFileName(this,tr("Open Document"), QDir::currentPath(),
                                                  tr("text files (*.txt)"));

    ui->launch_test_btn->setEnabled(false);

    _ed->file_in_path = file_in_path.toStdString();
    ui->cloud_in_ledit->setEnabled(true);
    ui->cloud_in_ledit->setText(file_in_path);
    ui->cloud_in_ledit->setEnabled(false);

    if (_ed->file_in_path.compare("") && _ed->file_out_path.compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cloud_homog_form::on_cloud_out_browse_btn_clicked()
{
    QString file_out_path;
    file_out_path = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly
                                                                      | QFileDialog::DontResolveSymlinks);

    ui->launch_test_btn->setEnabled(false);

    _ed->file_out_path = file_out_path.toStdString();
    ui->cloud_out_ledit->setEnabled(true);
    ui->cloud_out_ledit->setText(file_out_path);
    ui->cloud_out_ledit->setEnabled(false);

    if (_ed->file_in_path.compare("") && _ed->file_out_path.compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void cloud_homog_form::on_launch_test_btn_clicked()
{
    // for when the test is done
    int test_function_return_code;
    QMessageBox info_box;

    this->setEnabled(false);
    _ed->epsilon = (float)(ui->epsilon_sb->value());
    test_function_return_code = test::homogenize_cloud(_ed->file_in_path, _ed->file_out_path,
                                                           (int)_ed->epsilon);

    if (test_function_return_code)
        info_box.setText("Invalid input.");

    else
        info_box.setText("Operation complete.");

    info_box.exec();
    this->setEnabled(true);
}

void cloud_homog_form::on_cancel_btn_clicked()
{
    this->close();
}

void cloud_homog_form::on_help_btn_clicked()
{
    QMessageBox help_box;
    std::string help_msg;

    help_msg = (std::string("This is a test for the cloud crop function.\n\n") +
                std::string("> cloud in path - path of the input point cloud\n") +
                std::string("> cloud out path - path of directory where the result cloud will be stored\n") +
                std::string("> epsilon - determines the number of color categories for your cloud; the bigger") +
                std::string(" this parameter is, the smaller the number of colors in the end cloud\n"));

    help_box.setText(QString::fromUtf8(&help_msg[0], help_msg.size()));
    help_box.exec();
}