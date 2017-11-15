/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-11 11:28:52#$ $Rev:: 63048   $
**********************************************************************/


#include "widget.h"
#include "ui_widget.h"


//#include <SFML/Window/Joystick.hpp>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    // Rearrange buttons id 0...5 for lights
    ui->buttonGroup_lights->setId(ui->buttonGroup_lights->buttons().at(1), 0); // Head
    ui->buttonGroup_lights->setId(ui->buttonGroup_lights->buttons().at(2), 1); // Brake
    ui->buttonGroup_lights->setId(ui->buttonGroup_lights->buttons().at(4), 2); // Reverse
    ui->buttonGroup_lights->setId(ui->buttonGroup_lights->buttons().at(5), 3); // Hazard
    ui->buttonGroup_lights->setId(ui->buttonGroup_lights->buttons().at(3), 4); // TurnLeft
    ui->buttonGroup_lights->setId(ui->buttonGroup_lights->buttons().at(0), 5); // TurnRight

    // Rearrange buttons id 0...3 for us
    ui->buttonGroup_control_us->setId(ui->buttonGroup_control_us->buttons().at(0), 0); // enable front
    ui->buttonGroup_control_us->setId(ui->buttonGroup_control_us->buttons().at(2), 1); // disable front
    ui->buttonGroup_control_us->setId(ui->buttonGroup_control_us->buttons().at(1), 2); // enable rear
    ui->buttonGroup_control_us->setId(ui->buttonGroup_control_us->buttons().at(3), 3); // disable rear

//    for(int i = 0; ui->buttonGroup_control_us->buttons().size(); i++)
//    {
//        qDebug() << i << ui->buttonGroup_control_us->buttons().at(i)->text();
//    }

    ui->lineEdit_controller->setDisabled(true);
    ui->lineEdit_controller_xaxis->setDisabled(true);
    ui->lineEdit_controller_zaxis->setDisabled(true);
    ui->lineEdit_controller_a->setDisabled(true);
    ui->lineEdit_controller_b->setDisabled(true);
    ui->lineEdit_controller_x->setDisabled(true);
    ui->lineEdit_controller_y->setDisabled(true);
    ui->lineEdit_controller_rb->setDisabled(true);
    ui->lineEdit_controller_lb->setDisabled(true);

    ui->pushButton_calibrate->setDisabled(true);
    ui->radioButton_enableController->setDisabled(true);

    m_timer.setInterval(50);

    connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));

    m_timer.start();
}

Widget::~Widget()
{
    delete ui;
}

QButtonGroup* Widget::getLightButtonGroup()
{
    return ui->buttonGroup_lights;
}

QButtonGroup* Widget::getUsButtonGroup()
{
    return ui->buttonGroup_control_us;
}

void Widget::setSpeed(int value)
{
    ui->lcdNumber_throttle->display(value);
    emit throttleReceived((float)value);
}

void Widget::setSteering(int value)
{
    ui->lcdNumber_steering->display(value);
    emit steeringReceived((float)value);
}

void Widget::on_verticalSlider_throttle_valueChanged(int value)
{

}

void Widget::on_horizontalSlider_steering_valueChanged(int value)
{

}

void Widget::update()
{
    if(ui->radioButton_enable_resetting_slider->isChecked())
    {
        int steering = ui->horizontalSlider_steering->value();
        int speed = ui->verticalSlider_throttle->value();

        if(steering > 0)
            steering--;
        if (steering < 0)
            steering++;

        if(speed > 0)
            speed--;
        if(speed < 0)
            speed++;

        ui->horizontalSlider_steering->setValue(steering);
        ui->verticalSlider_throttle->setValue(speed);
    }

    setSteering(ui->horizontalSlider_steering->value());
    setSpeed(ui->verticalSlider_throttle->value());

    /*
    sf::Joystick::update();
    if (sf::Joystick::isConnected(0) && ui->radioButton_enableController->isChecked())
    {
        // joystick number 0 is connected
        float x = sf::Joystick::getAxisPosition(0, sf::Joystick::X);

        // the linux driver actaully handles the z-axis (both triggers left and right)
        // in 2 different axis
        float z_minus = sf::Joystick::getAxisPosition(0, sf::Joystick::Z);
        z_minus = -(z_minus + 100) / 2;
        float z_plus = sf::Joystick::getAxisPosition(0, sf::Joystick::R);
        z_plus = (z_plus + 100) / 2;
        float actual_z = z_minus + z_plus;

        ui->lcdNumber_steering->display(x);
        ui->lcdNumber_throttle->display(actual_z);

        emit steeringReceived(x);
        emit throttleReceived(actual_z);

        // Head
        if (sf::Joystick::isButtonPressed(0, 0))
        {
            static bool button_state = true;
            ui->lineEdit_controller_a->setEnabled(button_state);
            button_state = !button_state;
            emit buttonClicked(0); // this is actually the light id not the button id
        }

        // Brake
        if (sf::Joystick::isButtonPressed(0, 1))
        {
            static bool button_state = true;
            ui->lineEdit_controller_b->setEnabled(button_state);
            button_state = !button_state;
            emit buttonClicked(1);
        }

        // reverse
        if (sf::Joystick::isButtonPressed(0, 2))
        {
            static bool button_state = true;
            ui->lineEdit_controller_x->setEnabled(button_state);
            button_state = !button_state;
            emit buttonClicked(2);
        }

        // hazard
        if (sf::Joystick::isButtonPressed(0, 3))
        {
            static bool button_state = true;
            ui->lineEdit_controller_y->setEnabled(button_state);
            button_state = !button_state;
            emit buttonClicked(3);
        }

        // turn left
        if (sf::Joystick::isButtonPressed(0, 4))
        {
            static bool button_state = true;
            ui->lineEdit_controller_lb->setEnabled(button_state);
            button_state = !button_state;
            emit buttonClicked(4);
        }

        // turn right
        if (sf::Joystick::isButtonPressed(0, 5))
        {
            static bool button_state = true;
            ui->lineEdit_controller_rb->setEnabled(button_state);
            button_state = !button_state;
            emit buttonClicked(5);
        }
    }*/
}

void Widget::on_pushButton_calibrate_clicked()
{
//    QMessageBox msgBox;
//    msgBox.setWindowTitle( tr("Attention!") );
//    msgBox.setText("Now Calibration begins.");
//    msgBox.setInformativeText("Do not touch anything before advice.\n\nIf once started it can not be canceled so make sure you can not harm anybody.\n\nFirst set the states of your potis.");
//    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
//    msgBox.setDefaultButton(QMessageBox::Ok);
//    int ret = msgBox.exec();

//    if(ret == QMessageBox::Ok)
}
