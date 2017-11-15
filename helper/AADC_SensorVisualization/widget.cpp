/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-18 16:51:13#$ $Rev:: 63512   $
**********************************************************************/
#include "stdafx.h"

#include "widget.h"
#include "ui_widget.h"


QString format_number(int distance)
{
    std::stringstream ss;
    if (distance >= 0)
        ss << std::setw(4) << std::setfill(' ') << std::right << distance;
    else
        ss << "-" << std::setw(3) << std::setfill(' ') << abs(distance);

    return QString::fromStdString(ss.str());
}

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    memset(&m_usData, 0, sizeof(tUltrasonicStructSimple));
    memset(&m_imuData, 0, sizeof(tInerMeasUnitData));
    memset(&m_wheelData, 0, sizeof(tWheelDataSimple));
    memset(&m_batteryData, 0, sizeof(tVoltageStructSimple));
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update_gui()));
    timer->start(50);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::setUsFL(int distance)
{
    m_frameRates[SensorDefinition::US_FRONTLEFT].calc();
    m_usData.us_front_left = distance;
}

void Widget::setUsFCL(int distance)
{
    m_frameRates[SensorDefinition::US_FRONTCENTERLEFT].calc();
    m_usData.us_front_center_left = distance;
}

void Widget::setUsFC(int distance)
{
    m_frameRates[SensorDefinition::US_FRONTCENTER].calc();
    m_usData.us_front_center = distance;
}

void Widget::setUsFCR(int distance)
{
    m_frameRates[SensorDefinition::US_FRONTCENTERRIGHT].calc();
    m_usData.us_front_center_right = distance;
}

void Widget::setUsFR(int distance)
{
    m_frameRates[SensorDefinition::US_FRONTRIGHT].calc();
    m_usData.us_front_right = distance;
}

void Widget::setUsSL(int distance)
{
    m_frameRates[SensorDefinition::US_SIDELEFT].calc();
    m_usData.us_side_left = distance;
}

void Widget::setUsRCL(int distance)
{
    m_frameRates[SensorDefinition::US_REARCENTERLEFT].calc();
    m_usData.us_rear_center_left = distance;
}

void Widget::setUsRC(int distance)
{
    m_frameRates[SensorDefinition::US_REARCENTER].calc();
    m_usData.us_rear_center = distance;
}

void Widget::setUsRCR(int distance)
{
    m_frameRates[SensorDefinition::US_REARCENTERRIGHT].calc();
    m_usData.us_rear_center_right = distance;
}

void Widget::setUsSR(int distance)
{
    m_frameRates[SensorDefinition::US_SIDERIGHT].calc();
    m_usData.us_side_right = distance;
}

void Widget::setImu(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double roll, double pitch, double yaw)
{
    m_frameRates[SensorDefinition::IMU].calc();
    m_imuData.f32A_x = ax;
    m_imuData.f32A_y = ay;
    m_imuData.f32A_z = az;
    m_imuData.f32G_x = gx;
    m_imuData.f32G_y = gy;
    m_imuData.f32G_z = gz;
    m_imuData.f32M_x = mx;
    m_imuData.f32M_y = my;
    m_imuData.f32M_z = mz;

    m_imuData.f32Roll = roll;
    m_imuData.f32Pitch = pitch;
    m_imuData.f32Yaw = yaw;
}

void Widget::setWheelLeft(int count, int direction)
{
    m_frameRates[SensorDefinition::WHEEL_LEFT].calc();
    m_wheelData.count_left = count;
    m_wheelData.dir_left = direction;
}

void Widget::setWheelRight(int count, int direction)
{
    m_frameRates[SensorDefinition::WHEEL_RIGHT].calc();
    m_wheelData.count_right = count;
    m_wheelData.dir_right = direction;
}

void Widget::setVoltageActuator(float value)
{
    m_frameRates[SensorDefinition::VOLTAGE_ACTUATORS].calc();
    m_batteryData.actuator_overall = value;
}

void Widget::setVoltageActuatorCell1(float value)
{
    m_batteryData.actuator_cell1 = value;
}

void Widget::setVoltageActuatorCell2(float value)
{
    m_batteryData.actuator_cell2 = value;
}

void Widget::setVoltageSensors(float value)
{
    m_batteryData.sensors_overall = value;
}

void Widget::setVoltageSensorsCell1(float value)
{
    m_batteryData.sensors_cell1 = value;
}

void Widget::setVoltageSensorsCell2(float value)
{
    m_batteryData.sensors_cell2 = value;
}

void Widget::setVoltageSensorsCell3(float value)
{
    m_batteryData.sensors_cell3 = value;
}

void Widget::setVoltageSensorsCell4(float value)
{
    m_batteryData.sensors_cell4 = value;
}

void Widget::setVoltageSensorsCell5(float value)
{
    m_batteryData.sensors_cell5 = value;
}

void Widget::setVoltageSensorsCell6(float value)
{
    m_batteryData.sensors_cell6 = value;
}

void Widget::update_gui()
{
    if (ui->tabWidget->currentIndex() == 0 || ui->tabWidget->currentIndex() == 1)
    {

        ui->us_widget->setDistances(m_usData.us_front_left,
                                    m_usData.us_front_center_left,
                                    m_usData.us_front_center,
                                    m_usData.us_front_center_right,
                                    m_usData.us_front_right,
                                    m_usData.us_side_left,
                                    m_usData.us_side_right,
                                    m_usData.us_rear_center_left,
                                    m_usData.us_rear_center,
                                    m_usData.us_rear_center_right);


        // US
        ui->lcdNumber_us_update_fl->display(m_frameRates[SensorDefinition::US_FRONTLEFT].get_frame_rate().c_str());
        ui->lcdNumber_us_fl->display(format_number(m_usData.us_front_left));

        ui->lcdNumber_us_update_fcl->display(m_frameRates[SensorDefinition::US_FRONTCENTERLEFT].get_frame_rate().c_str());
        ui->lcdNumber_us_fcl->display(format_number(m_usData.us_front_center_left));

        ui->lcdNumber_us_update_fc->display(m_frameRates[SensorDefinition::US_FRONTCENTER].get_frame_rate().c_str());
        ui->lcdNumber_us_fc->display(format_number(m_usData.us_front_center));

        ui->lcdNumber_us_update_fcr->display(m_frameRates[SensorDefinition::US_FRONTCENTERRIGHT].get_frame_rate().c_str());
        ui->lcdNumber_us_fcr->display(format_number(m_usData.us_front_center_right));

        ui->lcdNumber_us_update_fr->display(m_frameRates[SensorDefinition::US_FRONTRIGHT].get_frame_rate().c_str());
        ui->lcdNumber_us_fr->display(format_number(m_usData.us_front_right));

        ui->lcdNumber_us_update_sl->display(m_frameRates[SensorDefinition::US_SIDELEFT].get_frame_rate().c_str());
        ui->lcdNumber_us_sl->display(format_number(m_usData.us_side_left));

        ui->lcdNumber_us_update_rcl->display(m_frameRates[SensorDefinition::US_REARCENTERLEFT].get_frame_rate().c_str());
        ui->lcdNumber_us_rcl->display(format_number(m_usData.us_rear_center_left));

        ui->lcdNumber_us_update_rc->display(m_frameRates[SensorDefinition::US_REARCENTER].get_frame_rate().c_str());
        ui->lcdNumber_us_rc->display(format_number(m_usData.us_rear_center));

        ui->lcdNumber_us_update_rcr->display(m_frameRates[SensorDefinition::US_REARCENTERRIGHT].get_frame_rate().c_str());
        ui->lcdNumber_us_rcr->display(format_number(m_usData.us_rear_center_right));

        ui->lcdNumber_us_update_sr->display(m_frameRates[SensorDefinition::US_SIDERIGHT].get_frame_rate().c_str());
        ui->lcdNumber_us_sr->display(format_number(m_usData.us_side_right));
    }


    if (ui->tabWidget->currentIndex() == 2)
    {
        // WHEEL
        ui->lcdNumber_wheel_update_left->display(m_frameRates[SensorDefinition::WHEEL_LEFT].get_frame_rate().c_str());

        ui->lcdNumber_wheel_left_count->display(m_wheelData.count_left);
        ui->lcdNumber_wheel_left_direction->display(m_wheelData.dir_left);

        ui->lcdNumber_wheel_update_right->display(m_frameRates[SensorDefinition::WHEEL_RIGHT].get_frame_rate().c_str());

        ui->lcdNumber_wheel_right_count->display(m_wheelData.count_right);
        ui->lcdNumber_wheel_right_direction->display(m_wheelData.dir_right);
    }

    if (ui->tabWidget->currentIndex() == 3 || ui->tabWidget->currentIndex() == 4)
    {
        // IMU
        ui->lcdNumber_imu_update->display(m_frameRates[SensorDefinition::IMU].get_frame_rate().c_str());

        ui->lcdNumber_imu_acc_x->display(m_imuData.f32A_x);
        ui->lcdNumber_imu_acc_y->display(m_imuData.f32A_y);
        ui->lcdNumber_imu_acc_z->display(m_imuData.f32A_z);

        ui->lcdNumber_imu_angular_x->display(m_imuData.f32G_x);
        ui->lcdNumber_imu_angular_y->display(m_imuData.f32G_y);
        ui->lcdNumber_imu_angular_z->display(m_imuData.f32G_z);

        ui->lcdNumber_imu_mag_x->display(m_imuData.f32M_x);
        ui->lcdNumber_imu_mag_y->display(m_imuData.f32M_y);
        ui->lcdNumber_imu_mag_z->display(m_imuData.f32M_z);

        ui->lcdNumber_imu_roll->display(m_imuData.f32Roll);
        ui->lcdNumber_imu_pitch->display(m_imuData.f32Pitch);
        ui->lcdNumber_imu_yaw->display(m_imuData.f32Yaw);
    }

    if (ui->tabWidget->currentIndex() == 5)
    {
        //BATTERY
        ui->lcdNumber_volt_update->display(m_frameRates[SensorDefinition::VOLTAGE].get_frame_rate().c_str());

        ui->lcdNumber_volt_actuator_overall->display(m_batteryData.actuator_overall);
        ui->lcdNumber_volt_sensors_overall->display(m_batteryData.sensors_overall);

        ui->lcdNumber_actuator_cell1->display(m_batteryData.actuator_cell1);
        ui->lcdNumber_actuator_cell2->display(m_batteryData.actuator_cell2);
        ui->lcdNumber_sensors_cell1->display(m_batteryData.sensors_cell1);
        ui->lcdNumber_sensors_cell2->display(m_batteryData.sensors_cell2);
        ui->lcdNumber_sensors_cell3->display(m_batteryData.sensors_cell3);
        ui->lcdNumber_sensors_cell4->display(m_batteryData.sensors_cell4);
        ui->lcdNumber_sensors_cell5->display(m_batteryData.sensors_cell5);
        ui->lcdNumber_sensors_cell6->display(m_batteryData.sensors_cell6);
    }

}
