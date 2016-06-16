#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    this->setWindowTitle("3D Objects Boundries Detection");
    this->setFixedSize(426, 341);
    ui->setupUi(this);
    ne_form = new normal_estimation_form(this);
    ne_form->setModal(true);
    cc_form = new cloud_crop_form(this);
    cc_form->setModal(true);
    ch_form = new cloud_homog_form(this);
    ch_form->setModal(true);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_next_btn_1_clicked()
{
    QMessageBox info_box;

    if (ui->test_fct_cb_1->currentIndex() == -1)
    {
       info_box.setText("Select one function.");
    }

    else
    {
        info_box.setText("Not yet implemented.");

        int current_index = ui->test_fct_cb_1->currentIndex();

        switch (current_index)
        {
        case 0:
            cc_form->exec();
            break;
        case 1:
            ne_form->exec();
            break;
        case 2:
            ch_form->exec();
            break;
        default:
            info_box.exec();
        }
    }
}

void MainWindow::on_next_btn_2_clicked()
{
    QMessageBox info_box;

    if (ui->test_fct_cb_2->currentIndex() == -1)
    {
       info_box.setText("Select one function.");
    }

    else
    {
        cti_form = new cloud_to_image_form(this, ui->test_fct_cb_2->currentIndex());
        cti_form->exec();
    }
}

void MainWindow::on_actionQuit_triggered()
{
    QApplication::quit();
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox about_box;
    std::string about_msg;

    about_msg = std::string("Cloud Object Segmentation\n\n") +
                std::string("This is a program that allows testing different cloud object segmentation") +
                std::string(" functions such as cloud normal estimation or clustering.\n\n") +
                std::string("Project Manager: Jean-Jacques Ponciano\n") +
                std::string("Developers: Vlad-Adrian Moglan, Kevin Naudin, Hugo Hebert\n\n") +
                std::string("i3mainz Institute for Spatial Information and Surveying Technology");
    about_box.setText(QString::fromUtf8(&about_msg[0], about_msg.size()));
    about_box.exec();
}

void MainWindow::on_quit_btn_clicked()
{
    QApplication::quit();
}

void MainWindow::how_to()
{
    QMessageBox about_box;
    std::string about_msg;

    about_msg = std::string("To use this app, just select a function and click next.\n\n") +
                std::string("> Primary Cloud Operations are strictly linked with Cloud Object Segmentation.\n") +
                std::string("> Secondary Cloud Operations are auxilliary functions that may be needed in future projects.\n");
    about_box.setText(QString::fromUtf8(&about_msg[0], about_msg.size()));
    about_box.exec();
}

void MainWindow::on_how_to_btn_clicked()
{
    MainWindow::how_to();
}

void MainWindow::on_actionInfo_triggered()
{
    MainWindow::how_to();
}
