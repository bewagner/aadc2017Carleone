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
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/
#include "stdafx.h"
#include "displaywidget.h"



DisplayWidget::DisplayWidget(QString filename, QWidget* parent) : QWidget(parent), m_defaultFileName(filename)
{
    m_pWidget = new QWidget(this);
    m_pWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    m_btSendValue = new QPushButton(this);
    m_btSendValue->setText("Start");
    m_btSendValue->setFixedSize(200,50);

    m_btLoadDefaultFile = new QPushButton(this);
    m_btLoadDefaultFile->setText("Load Defaults");
    m_btLoadDefaultFile->setFixedSize(200,50);

    m_btSaveToFile = new QPushButton(this);
    m_btSaveToFile->setText("Save");
    m_btSaveToFile->setFixedSize(200,50);

    // create the table view and the item model for the gyrocscope
    m_TableView = new QTableView(this);
    createTableModel();

    m_mainLayout = new QVBoxLayout();
    m_mainLayout->addWidget(m_TableView,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btLoadDefaultFile,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSendValue,0,Qt::AlignCenter);
    m_mainLayout->addWidget(m_btSaveToFile,0,Qt::AlignCenter);
    setLayout(m_mainLayout);

    connect(m_btSendValue,  SIGNAL(clicked()), this, SLOT(OnButtonClicked()));
    connect(m_btSaveToFile,  SIGNAL(clicked()), this, SLOT(OnButtonSaveClicked()));
    connect(m_btLoadDefaultFile,  SIGNAL(clicked()), this, SLOT(OnButtonLoadDefaultClicked()));
}
void DisplayWidget::createTableModel()
{
    // create the table view and the item model for the gyrocscope
    m_ItemModel = new QStandardItemModel(10,3,this);
    m_TableView->setModel(m_ItemModel);
    m_TableView->resizeRowsToContents();
    m_TableView->setColumnWidth(0,60);
    m_TableView->setColumnWidth(1,60);
    m_TableView->setColumnWidth(2,60);
    m_TableView->setFixedWidth(m_TableView->columnWidth(0)+m_TableView->columnWidth(1)+m_TableView->columnWidth(2)+m_TableView->verticalHeader()->width());
    m_TableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_TableView->setFixedHeight(m_TableView->rowHeight(1)*m_ItemModel->rowCount() +m_TableView->horizontalHeader()->height());
    m_TableView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QStringList header;
    header << "Time (msec)" << "Value 1"<< "Value 2";;
    m_ItemModel->setHorizontalHeaderLabels(header);
}
void DisplayWidget::OnButtonClicked()
{
    int i8StateFlag=0;
    for (int i = 0; i < m_ItemModel->rowCount(); i++)
    {
        if (i==0)
            i8StateFlag = 0;  //clears the vector
        else if (i==m_ItemModel->rowCount()-1)
            i8StateFlag = 2; //starts the list after receiving the last tuple
        else
            i8StateFlag = 1; // just adding the line to vector
        if (m_ItemModel->data(m_ItemModel->index(i,0)).isValid() && m_ItemModel->data(m_ItemModel->index(i,1)).isValid() && m_ItemModel->data(m_ItemModel->index(i,2)).isValid())
        {
            if (m_ItemModel->data(m_ItemModel->index(i,0)).toFloat()>0)
            {
                // the next entry is not valid so start after this tuple
                if (!m_ItemModel->data(m_ItemModel->index(i+1,0)).isValid()) i8StateFlag = 2;
                sendTuple(i8StateFlag,m_ItemModel->data(m_ItemModel->index(i,0)).toFloat(),m_ItemModel->data(m_ItemModel->index(i,1)).toFloat(),m_ItemModel->data(m_ItemModel->index(i,2)).toFloat());
            }
        }
    }
}

void DisplayWidget::OnButtonSaveClicked()
{
    // get the file dialog and the file name
    QString filename = QFileDialog::getSaveFileName(this,tr("Save Calibration As.."),m_defaultFileName);
    QFile file(filename);
    // open and read the file
    if (file.open(QIODevice::WriteOnly))
    {
        QTextStream stream(&file);
        qint32 n(m_ItemModel->rowCount()), m(m_ItemModel->columnCount());
        // iterating through the model
        for (int i=0; i<n; ++i)
        {
            if (m_ItemModel->data(m_ItemModel->index(i,0)).isValid() && m_ItemModel->data(m_ItemModel->index(i,1)).isValid() && m_ItemModel->data(m_ItemModel->index(i,2)).isValid())
            {
                for (int j=0; j<m; j++)
                {
                    // save the data to the file
                    if (m_ItemModel->data(m_ItemModel->index(i,j)).isValid())
                        stream << m_ItemModel->data(m_ItemModel->index(i,j)).toString() << " ";
                }
                // create a new line
                stream << "\n";
            }
        }
        file.close();
    }
}

void DisplayWidget::OnButtonLoadDefaultClicked()
{
    // clears the data in the model
    for (int i=0; i<m_ItemModel->rowCount(); ++i)
    {
        {
            for (int j=0; j<m_ItemModel->columnCount(); j++)
            {
                m_ItemModel->setData(m_ItemModel->index(i,j),QVariant::Invalid);
            }
        }
    }

    QFile file(m_defaultFileName);
    // open the file for read
    if (file.open(QIODevice::ReadOnly))
    {
        QTextStream stream(&file);
        // indicating the row in the file
        int n = 0;
        while (!stream.atEnd())
        {
            // seperates one line in the elements
            QStringList list = stream.readLine().split(" ");
            if (list.size() == 4)
            {
                if (list[0].toFloat()==0.f) break;
                //write data to model
                m_ItemModel->setData(m_ItemModel->index(n,0), list[0].toFloat());
                m_ItemModel->setData(m_ItemModel->index(n,1), list[1].toFloat());
                m_ItemModel->setData(m_ItemModel->index(n,2), list[2].toFloat());
                n++;
                if (n>m_ItemModel->rowCount()) break;
            }
        }



        file.close();
    }
}