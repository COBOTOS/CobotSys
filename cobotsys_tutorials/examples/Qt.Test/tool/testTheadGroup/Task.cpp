//
// Created by xuzhenhai on 19-7-18.
//

#include "Task.h"
#include <QProcess>
#include <iostream>
#include <logger/Logger.h>
#include <QtCore/QThreadPool>
#include "boost/make_shared.hpp"

Task::Task(QString taskName) : _taskName(taskName) {
    engage();
    _taskListeners = boost::shared_ptr<std::vector<TaskListenerPtr>>();
}

Task::~Task() {
    LOG_INFO<<"~Task";
}

QString Task::getTaskName() {
    return _taskName;
}

Task::Task(QString taskName, boost::function<Result(Task*)> taskFunction) : _taskName(taskName),
                                                                           _taskFuncion(taskFunction) {
    engage();
    _taskListeners = boost::make_shared<std::vector<TaskListenerPtr>>();
}

void Task::engage() {
    connect(this, &Task::startTaskSignal, this, &Task::taskBegin, Qt::BlockingQueuedConnection);
    connect(this, &Task::finishTaskSignal, this, &Task::taskFinish, Qt::BlockingQueuedConnection);
    connect(this, &Task::taskExceptionSignal, this, &Task::taskFail, Qt::BlockingQueuedConnection);
    connect(this, &Task::taskProgressSignal, this, &Task::taskProgress, Qt::BlockingQueuedConnection);
}

void Task::disengage() {
    disconnect();
}


void Task::run() {
    Q_EMIT startTaskSignal();
    Result result;
    if (_taskFuncion != 0) {
        LOG_INFO << "taskFuncion start" << std::endl;
        try {
            result = _taskFuncion(this);
        } catch (std::exception ex) {
            LOG_ERROR << _taskName.toStdString() << " run fail !!!" << ex.what();
            Q_EMIT taskExceptionSignal(ex);
        }
    }
    LOG_INFO << "finishTaskSignal start" << std::endl;
    Q_EMIT finishTaskSignal(result);
    disengage();
}

Result Task::getResult() {
    return _result;
}

void Task::notifyProgress(float percent, QString message) {
    Q_EMIT taskProgressSignal(percent, message);
}

void Task::addTaskListener(TaskListenerPtr taskListenerPtr) {
    _taskListeners->push_back(taskListenerPtr);
}

void Task::taskBegin() {
    for(int i =0;i<_taskListeners->size();i++){
        auto l = _taskListeners->at(i);
        l->taskBegin(this);
    }
}

void Task::taskFinish(Result result) {
    for(int i =0;i<_taskListeners->size();i++){
        auto l = _taskListeners->at(i);
        l->taskFinish(this, result);
    }
}

void Task::taskProgress(float percent, QString message) {
    for(int i =0;i<_taskListeners->size();i++){
        auto l = _taskListeners->at(i);
        l->taskProgress(this, percent, message);
    }
}

void Task::taskFail(std::exception exection) {
    for(int i =0;i<_taskListeners->size();i++){
        auto l = _taskListeners->at(i);
        l->taskFail(this, exection);
    }
}
//
//void Task::start() {
//    QThreadPool::globalInstance()->start(this);
//}