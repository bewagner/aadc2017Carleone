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
   * $Author:: spiesra $  $Date:: 2017-05-05 16:42:49#$ $Rev:: 62880   $
   **********************************************************************/
#include "stdafx.h"
#include "displaywidget.h"



DisplayWidget::DisplayWidget(QWidget* parent) : QWidget(parent)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_tableView = new QTableView(this);

    m_modelTableView = new QStandardItemModel(5, 3, this); //2 Rows and 3 Columns
    m_modelTableView->setHorizontalHeaderItem(0, new QStandardItem(QString("Type")));
    m_modelTableView->setHorizontalHeaderItem(1, new QStandardItem(QString("Prob.")));
	m_modelTableView->setHorizontalHeaderItem(2, new QStandardItem(QString("Orient.")));
	m_modelTableView->setHorizontalHeaderItem(3, new QStandardItem(QString("Kid?")));
    m_tableView->setModel(m_modelTableView);
    m_tableView->setMinimumWidth(800);
    m_tableView->setMinimumHeight(260);
    m_tableView->setColumnWidth(150, 300);

    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(m_tableView, 0, Qt::AlignCenter);
    setLayout(m_mainLayout);

}


void DisplayWidget::OnNewClassificationResult(QString results, double probability,  QString orientation,  QString kid)
{
    //check sizes of list
    //clear old values

    m_modelTableView->setItem(m_currentTableViewLine, 0, new QStandardItem(results));
    m_modelTableView->setItem(m_currentTableViewLine, 1, new QStandardItem(QString::number(probability)));
    m_modelTableView->setItem(m_currentTableViewLine, 2, new QStandardItem(orientation));
	m_modelTableView->setItem(m_currentTableViewLine, 3, new QStandardItem(kid));
	m_currentTableViewLine++;

}

void DisplayWidget::OnResetResults()
{
	//m_modelTableView->clear();
	m_currentTableViewLine = 0;
}








