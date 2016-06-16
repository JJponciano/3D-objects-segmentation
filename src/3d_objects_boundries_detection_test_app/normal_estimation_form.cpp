#include "normal_estimation_form.h"
#include "ui_normal_estimation_form.h"

normal_estimation_form::normal_estimation_form(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::normal_estimation_test_form)
{
    this->setWindowTitle("Cloud normal estimation test form");
    this->setFixedSize(508, 393);
    ui->setupUi(this);
    _ned = new test::normal_estimation_data;

    // xyz coords input
    ui->x_scale_dsb->setSingleStep(0.01);
    ui->x_scale_dsb->setValue(0.001);
    ui->x_scale_dsb->setMinimum(0.001);
    ui->x_scale_dsb->setMaximum(1000.0);
    ui->y_scale_dsb->setSingleStep(0.01);
    ui->y_scale_dsb->setValue(0.001);
    ui->y_scale_dsb->setMinimum(0.001);
    ui->y_scale_dsb->setMaximum(1000.0);
    ui->z_scale_dsb->setSingleStep(0.01);
    ui->z_scale_dsb->setValue(0.001);
    ui->z_scale_dsb->setMinimum(0.001);
    ui->z_scale_dsb->setMaximum(1000.0);

    // kd-tree input
    ui->radius_dsb->setDecimals(3);
    ui->radius_dsb->setSingleStep(0.01);
    ui->radius_dsb->setMaximum(5);
    ui->max_neighbs_sb->setSingleStep(25);
    ui->max_neighbs_sb->setMinimum(1.0);
    ui->max_neighbs_sb->setMaximum(1000.0);

    // max. fragm. depth
    ui->max_fragm_depth_sb->setMinimum(1);
    ui->max_fragm_depth_sb->setMaximum(5000);
    ui->max_fragm_depth_sb->setSingleStep(100);

    ui->cloud_in_ledit->setEnabled(false);
    ui->cloud_out_ledit->setEnabled(false);
    ui->launch_test_btn->setEnabled(false);
}

normal_estimation_form::~normal_estimation_form()
{
    delete ui;
}

void normal_estimation_form::on_cloud_in_browse_btn_clicked()
{
    QString file_in_path;
    file_in_path = QFileDialog::getOpenFileName(this,tr("Open Document"), QDir::currentPath(),
                                                  tr("text files (*.txt)"));

    ui->launch_test_btn->setEnabled(false);

    _ned->file_in_path = file_in_path.toStdString();
    ui->cloud_in_ledit->setEnabled(true);
    ui->cloud_in_ledit->setText(file_in_path);
    ui->cloud_in_ledit->setEnabled(false);
    ui->x_scale_dsb->setValue(0.001);ui->x_scale_dsb->setValue(0.001);

    if (_ned->file_in_path.compare("") && _ned->file_out_path.compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void normal_estimation_form::on_cloud_out_browse_btn_clicked()
{
    QString file_out_path;
    file_out_path = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "/home", QFileDialog::ShowDirsOnly
                                                                      | QFileDialog::DontResolveSymlinks);

    ui->launch_test_btn->setEnabled(false);

    _ned->file_out_path = file_out_path.toStdString();
    ui->cloud_out_ledit->setEnabled(true);
    ui->cloud_out_ledit->setText(file_out_path);
    ui->cloud_out_ledit->setEnabled(false);

    if (_ned->file_in_path.compare("") && _ned->file_out_path.compare(""))
        ui->launch_test_btn->setEnabled(true);
}

void normal_estimation_form::on_launch_test_btn_clicked()
{
    // for when the test is done
    int test_function_return_code;
    QMessageBox info_box;

    this->setEnabled(false);
    _ned->radius = ui->radius_dsb->value();
    _ned->max_neighbs = ui->max_neighbs_sb->value();
    _ned->x_scale = ui->x_scale_dsb->value();
    _ned->y_scale = ui->y_scale_dsb->value();
    _ned->z_scale = ui->z_scale_dsb->value();
    _ned->max_fragment_depth = ui->max_fragm_depth_sb->value();
    test_function_return_code = test::estimate_normals(_ned->file_in_path, _ned->file_out_path,
                                                        _ned->radius, _ned->max_neighbs, _ned->x_scale,
                                                        _ned->y_scale,_ned->z_scale,
                                                        _ned->max_fragment_depth);

    if (test_function_return_code)
        info_box.setText("Invalid input.");

    else
        info_box.setText("Operation complete.");

    info_box.exec();
    this->setEnabled(true);
}

void normal_estimation_form::on_cancel_btn_clicked()
{
    this->close();
}

void normal_estimation_form::on_help_btn_clicked()
{
    QMessageBox help_box;
    std::string help_msg;

    help_msg = std::string("This is a test form for cloud normal estimation.\n\n") +
            std::string("> cloud in path - path of the input point cloud\n") +
            std::string("> cloud out path - path of directory where the result cloud will be stored\n") +
            std::string("> radius - parameter for the kd-tree radius search function (see report for details)\n") +
            std::string("> max_neighbs - parameter for defining the maximum number of neighbours returned by the") +
            std::string("kd-tree radius search function\n") +
            std::string("> x/y/z scale - float number by which every coordinate of each point in the cloud will be multiplied;") +
            std::string(" example: with widop clouds the y coordinate is many times bigger than x and z and it needs to be scaled down\n");

    help_box.setText(QString::fromUtf8(&help_msg[0], help_msg.size()));
    help_box.exec();

}
