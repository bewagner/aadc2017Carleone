/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/
#include "stdafx.h"
#include "displaywidget.h"



DisplayWidget::DisplayWidget(QWidget* parent) : QWidget(parent)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_btSendValueZero = new QPushButton(this);
    m_btSendValueZero->setText("Brake");
    m_btSendValueZero->setFixedSize(200,50);

    m_btSendValueOne = new QPushButton(this);
    m_btSendValueOne->setText("Follow the road");
    m_btSendValueOne->setFixedSize(200,50);

    m_btSendValueTwo = new QPushButton(this);
    m_btSendValueTwo->setText("Go straight for 1m");
    m_btSendValueTwo->setFixedSize(200,50);

    m_btSendValueThree = new QPushButton(this);
    m_btSendValueThree->setText("Right turn");
    m_btSendValueThree->setFixedSize(200,50);

    m_btSendValueFour = new QPushButton(this);
    m_btSendValueFour->setText("Left turn");
    m_btSendValueFour->setFixedSize(200,50);

    m_btSendValueFive = new QPushButton(this);
    m_btSendValueFive->setText("TurnOutRight");
    m_btSendValueFive->setFixedSize(200,50);

    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(m_btSendValueZero,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueOne, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueTwo, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueThree, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueFour, 0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValueFive, 0,Qt::AlignCenter);

    setLayout(m_mainLayout);

    connect(m_btSendValueZero,  SIGNAL(clicked()), this, SLOT(sendValueZero()));
    connect(m_btSendValueOne,  SIGNAL(clicked()), this, SLOT(sendValueOne()));
    connect(m_btSendValueTwo,  SIGNAL(clicked()), this, SLOT(sendValueTwo()));
    connect(m_btSendValueThree,  SIGNAL(clicked()), this, SLOT(sendValueThree()));
    connect(m_btSendValueFour,  SIGNAL(clicked()), this, SLOT(sendValueFour()));
    connect(m_btSendValueFive,  SIGNAL(clicked()), this, SLOT(sendValueFive()));

}








