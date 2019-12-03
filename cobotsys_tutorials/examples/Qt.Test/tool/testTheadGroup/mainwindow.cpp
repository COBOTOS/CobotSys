#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QThread>
#include "Task.h"
#include "TaskListenerAdapter.h"
#include <iostream>
#include <QMessageBox>
#include <QThreadPool>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
#include <logger/Logger.h>

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow) {
    ui->setupUi(this);
    _taskMonitor = TaskMonitor::createTaskMonitor();
    _taskMonitor->_startFuncion = [this](Task *task) {
        ui->plainTextEdit->setPlainText(task->getTaskName() + " is running");
        ui->progressBar->setValue(0);

    };
    _taskMonitor->_processFunction = [this](Task *task, float p, QString message) {
        ui->progressBar->setToolTip(message);
        ui->progressBar->setValue(p);
    };
    _taskMonitor->_finishFunction = [this](Task *task, Result result) {
        ui->plainTextEdit->setPlainText(task->getTaskName() + " is finish");
        ui->progressBar->setValue(0);
    };
    _taskMonitor->_errorFunction = [this](Task *task, std::exception ex) {
        ui->plainTextEdit->setPlainText(task->getTaskName() + " is fail");
    };

}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButton_clicked() {
    QMessageBox *msgBox = new QMessageBox(this);
    msgBox->setAttribute(Qt::WA_DeleteOnClose);
    msgBox->setStandardButtons(QMessageBox::Ok);
    msgBox->setWindowTitle(tr("Status"));
    msgBox->setText(tr("正在加载文件请稍等!"));
    msgBox->setModal(false);

    Task *task = new Task("testTask",
                          [](Task *task) -> Result {
                              LOG_INFO << "testTask running" << std::endl;
                              Result result;
                              for (int i = 0; i <= 100; i++) {
                                  LOG_INFO << "xxxx";
                                  task->notifyProgress(i, QString::number(i));
                                  if (i % 10 == 0)
                                      sleep(1);
                              }
                              result._result = 1;
                              return result;
                          });
    boost::function<void(Task *task, Result)> finish = [](Task *task, Result result) {};
    task->addTaskListener(boost::make_shared<TaskListenerAdapter>(finish));
    _taskMonitor->postStask(task);
//    task->addTaskListener(boost::make_shared<TaskListener>([](Task *task, Result result) {}));
//    _taskMonitor->postStask(task);
//    QThreadPool::globalInstance()->start(task);
//    LOG_INFO << "QThreadPool start" << std::endl;
}
