#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    instance = QJoysticks::getInstance();

    /* Enable the virtual joystick */
    instance->setVirtualJoystickRange(1);
    instance->setVirtualJoystickEnabled(true);
    instance->setVirtualJoystickAxisSensibility(0.7);
    //instance->
    connect(
        instance, &QJoysticks::axisChanged,
        [=]( const int js, const int axis, const qreal value) { std::cout<<"doslo "<<js<<" "<<axis<<" "<<value<<std::endl; }
    );

}

MainWindow::~MainWindow()
{
    delete ui;
}

