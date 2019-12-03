#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QMessageBox>
#include <QString>
#include <QFileDialog>
#include <QTextStream>
#include <QFile>
#include <logger/Logger.h>
//#include <QtXml>
//#include <QDomDocument>
#include <QTime>
#include <LocalStorageRepositoryFactoryInterface.h>
#if 0
using namespace LION_LSR_NS_API;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),_count(1),_row(0),_colum(0),_timer(nullptr)
{
    ui->setupUi(this);
    setWindowTitle("Record Grep Points");
    setAutoFillBackground(true);//必须有这条语句
    setPalette(QPalette(QColor(85, 87, 83)));
    int width = this->geometry().width();
    int height = this->geometry().height();
    this->setFixedSize(width,height); //设置窗体固定大小
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->pushButton_connect->setStyleSheet("background-color:lightblue");
    ui->pushButton_disconnect->setStyleSheet("background-color:lightblue");
    ui->pushButton->setStyleSheet("background-color:lightblue");
    ui->pushButton_save->setStyleSheet("background-color:lightblue");
    ui->pushButton_modify->setStyleSheet("background-color:lightblue");
    ui->pushButton_openfile->setStyleSheet("background-color:lightblue");
    ui->pushButton_delete->setStyleSheet("background-color:lightblue");
    ui->spinBox_index->setValue(1);

    _jointDataManager= boost::shared_ptr<JointsDataManager> (new JointsDataManager());

    boost::shared_ptr<LocalStorageRepositoryFactoryInterface> lsrFactory = LocalStorageRepositoryFactoryInterface::create();
    _lsr = lsrFactory->createLocalStorageRepository();
    //_lsr->getItem()

    boost::shared_ptr<RobotDriverFactoryInterface> robotfactory = RobotDriverFactoryInterface::create();
    _robotDriver = robotfactory->createRobotDriver("ur3");

}

MainWindow::~MainWindow()
{
    delete ui;

}

void MainWindow::on_tableWidget_activated(const QModelIndex &index)
{
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
}

void MainWindow::refresh() {
    _joins=_robotDriver->getRobotJoints();
    if(_joints.size()==6)
    {
        QString jointStr;
        jointStr = QString("%1,%2,%3,%4,%5,%6").arg(joints[0]*180/3.1415927).arg(joints[1]*180/3.1415927).arg(joints[2]*180/3.1415927).arg(joints[3]*180/3.1415927).arg(joints[4]*180/3.1415927).arg(joints[5]*180/3.1415927);
        ui->textEdit->setText(jointStr);
    }
}

int MainWindow::findRow(int index){
    int row;
    bool isFindRow=false;
    _row=ui->tableWidget->rowCount();
    for(int i=0 ; i<_row ; i++)
    {
        QString rowName=ui->tableWidget->verticalHeaderItem(i)->text();
        int rowdata = rowName.toInt();
        if(rowdata==index){
            row=i;
            isFindRow=true;
            break;
        }
    }
    if(isFindRow){
        return row;
    }
    else{
        return -1;
    }
}

//改名
void MainWindow::addData(){
    int index=ui->spinBox_index->value();
//    if(findRow(index)==-1){//先判断是否重复,然后提示确认是否添加该序号点
        ui->tableWidget->setRowCount(_row+1);
        QTableWidgetItem *item = new QTableWidgetItem();
        QString qStrIndex=QString::number(index);
        QString currentPoint=ui->textEdit->toPlainText();
        item->setText(qStrIndex);
        ui->tableWidget->setVerticalHeaderItem(_row,item);
        for(int i=0;i<_colum;i++){
            ui->tableWidget->setItem(_row,i,new QTableWidgetItem(_joins[i]));
        }
        _count++;
        ui->textEdit->clear();
//    }
//    else{
//        QMessageBox::information(this,"ADD","Repeated ADD");
//        ui->textEdit->clear();
//    }
}


void MainWindow::on_pushButton_clicked()    //ADD按钮点击事件
{
    int index=ui->spinBox_index->value();
    if (findRow(index)!=-1)
    {
        QMessageBox::information(this,"ADD","Repeated ADD");
        return;
    }
    _row=ui->tableWidget->rowCount();
    _colum=ui->tableWidget->columnCount();

//    int index=ui->spinBox_index->value();
    QString str="Add "+QString::number(index);
    if(QMessageBox::Yes == QMessageBox::question(this,"Add",str,QMessageBox::Yes|QMessageBox::No))
    {
        //逻辑不对,需要先修改map,再在界面显示,而且应该在确认添加后再进行这部
        if(_jointDataManager->addJoints(index,_joins))
        {
            addData();
            ui->spinBox_index->setValue(_count);
            LOG_INFO<<"添加成功";

        }
        else
        {
            LOG_ERROR<<"添加失败";
        }
    }
}


void MainWindow::on_pushButton_connect_clicked()
{
    if(RobotStatus::Idle == _robotDriver->connect(/*_robotIP->text().toStdString())*/)
    {
        QMessageBox::information(this,"connect","connected!");
        if(!_timer)
        {
            _timer = new QTimer(this);
            connect(_timer, SIGNAL(timeout()), this, SLOT(refresh()));
            _timer->start(500);
            LOG_INFO<<"time start!";
        }
        ui->pushButton_connect->setStyleSheet("background-color:rgb(85, 87, 83)");
        ui->pushButton_connect->setEnabled(false);
        ui->pushButton_disconnect->setEnabled(true);
        ui->pushButton_disconnect->setStyleSheet("background-color:lightblue");
    }
}

void MainWindow::on_pushButton_disconnect_clicked()
{
      if(RobotStatus::Disconnect == _robotDriver->connect(/*_robotIP->text().toStdString())*/))//这个是什么意思?为什么还是connect
      {
        QMessageBox::information(this,"connect","disconnected!");
        ui->pushButton_disconnect->setStyleSheet("background-color:rgb(85, 87, 83)");
        ui->pushButton_disconnect->setEnabled(false);
        ui->pushButton_connect->setEnabled(true);
        ui->pushButton_connect->setStyleSheet("background-color:lightblue");
       }
}


void MainWindow::on_pushButton_openfile_clicked()
{
    //当表格有数据的时候,不确认打开新文件,则返回
    if(ui->tableWidget->rowCount()>0)
    {
        if(QMessageBox::No == QMessageBox::question(this,"打开","表格有数据，确定打开？",QMessageBox::Yes|QMessageBox::No))
        {
            return;
        }
    }

    //当表格没数据,或者即使有数据,也想打开新文件,则执行以下打开操作
    ui->tableWidget->clear();
    QString fileName=QFileDialog::getOpenFileName(this,tr("打开文件"),".",tr("xml文件(*.xml)"));

    //当打开文件为空,则提示返回
    if(fileName==NULL)
    {
        QMessageBox::information(this,"Open","Filename is NULL!");
        return;
    }

    //不确认打开,则返回
    if(QMessageBox::No == QMessageBox::question(this,"打开",fileName,QMessageBox::Yes|QMessageBox::No))
    {
        return;
    }

    //确认打开,则执行以下操作
    std::map<std::string,std::vector<std::string>> getJointsMap;
    if(_jointDataManager->xml2JointsMap(fileName.toStdString(),getJointsMap))
    {
        int iRowCount=getJointsMap.size();
        ui->tableWidget->setRowCount(iRowCount);
        ui->spinBox_index->setValue(iRowCount);
        for(auto iter = getJointsMap.begin(),_count=0;iter != getJointsMap.end();iter++,_count++)
        {
            QTableWidgetItem *item = new QTableWidgetItem();
            QString qStrIndex=QString::fromStdString(iter->first);
            item->setText(qStrIndex);
            ui->tableWidget->setVerticalHeaderItem(_count,item);
            for(int i=0;i<iter->second.size();i++)
            {
                QString joint=QString::fromStdString(iter->second[i]);
                ui->tableWidget->setItem(_count,i,new QTableWidgetItem(joint));
            }
        }
        QMessageBox::about(this,tr("打开"),tr("打开文件成功"));
    }
    else
    {
        QMessageBox::about(this,tr("打开"),tr("打开文件失败"));
    }



/*    if(ui->tableWidget->rowCount()>0)
    {
        if(QMessageBox::Yes == QMessageBox::question(this,"打开","表格有数据，确定打开？",QMessageBox::Yes|QMessageBox::No))
        {
            ui->tableWidget->clear();
            QString fileName=QFileDialog::getOpenFileName(this,tr("打开文件"),".",tr("xml文件(*.xml)"));
            if(fileName!=NULL)
            {
                if(QMessageBox::Yes == QMessageBox::question(this,"打开",fileName,QMessageBox::Yes|QMessageBox::No))
                {
                    std::map<std::string,std::vector<std::string>> getJointsMap;

                    if(_jointDataManager->xml2JointsMap(fileName.toStdString(),getJointsMap))
                    {
                        int iRowCount=getJointsMap.size();
                        ui->tableWidget->setRowCount(iRowCount);
                        ui->spinBox_index->setValue(iRowCount);
                        for(auto iter = getJointsMap.begin(),_count=0;iter != getJointsMap.end();iter++,_count++)
                        {
                            QTableWidgetItem *item = new QTableWidgetItem();
                            QString qStrIndex=QString::fromStdString(iter->first);
                            item->setText(qStrIndex);
                            ui->tableWidget->setVerticalHeaderItem(_count,item);
                            for(int i=0;i<iter->second.size();i++)
                            {
                                QString joint=QString::fromStdString(iter->second[i]);
                                ui->tableWidget->setItem(_count,i,new QTableWidgetItem(joint));
                            }
                        }
                        QMessageBox::about(this,tr("打开"),tr("打开文件成功"));
                    }
                    else
                    {
                        QMessageBox::about(this,tr("打开"),tr("打开文件失败"));
                    }
                }

            }
        }
    }
*/
}

void MainWindow::on_pushButton_save_clicked()
{
    QString curPath=QCoreApplication::applicationDirPath();
    QString fileName=QFileDialog::getSaveFileName(this,tr("保存文件"),curPath,tr("xml文件(*.xml)"));
    if(fileName!=NULL)
    {

        if(QMessageBox::Yes == QMessageBox::question(this,"保存",fileName,QMessageBox::Yes|QMessageBox::No))
        {
            if(_jointDataManager->jointsMap2Xml(fileName.toStdString()))
            {
                QMessageBox::about(this,tr("保存"),tr("保存文件成功"));
            }
            else
            {
                QMessageBox::about(this,tr("保存"),tr("保存文件失败"));
            }
        }

    }

}

void MainWindow::on_pushButton_delete_clicked()
{
    int index=ui->spinBox_index->value();
    int findrow = findRow(index);
    if(findrow!=-1){
        QString str="delete "+QString::number(index);
        if(QMessageBox::Yes == QMessageBox::question(this,"delete",str,QMessageBox::Yes|QMessageBox::No))
        {
            _jointDataManager->deleteJoints(index);
            ui->tableWidget->removeRow(findrow);
            _count--;
            ui->spinBox_index->setValue(_count);
        }
    }
    else
        {
            QMessageBox::information(this,"delete","make sure your index!");
        }
}

void MainWindow::on_pushButton_modify_clicked()
{
    int index=ui->spinBox_index->value();
    QString str="modify "+QString::number(index);
    int findrow = findRow(index);
    if(findrow!=-1)
    {
        if(QMessageBox::Yes == QMessageBox::question(this,"modify",str,QMessageBox::Yes|QMessageBox::No))
        {
            _jointDataManager->modifyJoints(index,_joins);
            for(int j=0;j<_colum;j++)
            {
                ui->tableWidget->setItem(findrow,j,new QTableWidgetItem(QString::number(_joins[i])));
            }
        }
    }
    else
        {
            QMessageBox::information(this,"modify","make sure your index!");
        }

}
#endif