#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    this->setWindowTitle("3D Objects Boundries Detection Test App");
    this->setFixedSize(409, 156);
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_test_btn_clicked()
{
    QMessageBox info_box;

    if (ui->test_fct_cb->currentIndex() == -1)
    {
       info_box.setText("Select one function.");
    }

    else
    {
        info_box.setText("Not yet implemented.");

        // windows
        normal_estimation_test_form ne_test_form(this);

        int current_index = ui->test_fct_cb->currentIndex();

        switch (current_index)
        {
        case 0:
            ne_test_form.setModal(true);
            ne_test_form.exec();
            break;
        default:
            info_box.exec();
        }
    }
}

void MainWindow::on_actionQuit_triggered()
{
    QApplication::quit();
}
