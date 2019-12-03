//
// Created by xuzhenhai on 19-7-18.
//

#ifndef COBOTCORE_QT_TEST_TASK_H
#define COBOTCORE_QT_TEST_TASK_H

#include <QString>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
#include <QRunnable>
#include <QObject>
#include <QVariant>
#include "TaskListener.h"
#include <vector>
#include "Result.h"

class TaskListener;

typedef boost::shared_ptr<TaskListener> TaskListenerPtr;

class Task : public QObject, public QRunnable {
Q_OBJECT
public:
    Task(QString taskName = "");

    Task(QString taskName = "", boost::function<Result(Task*)> taskFunction = nullptr);

    virtual ~Task();

    QString getTaskName();

    Result getResult();


    boost::function<Result(Task*)> _taskFuncion;
//    boost::function<void()> _startFuncion;
//    boost::function<void(Result)> _finishFunction;
//    boost::function<void(float, QString)> _processFunction;
//    boost::function<void(QString)> _errorFunction;

private:
    QString _taskName;
    Result _result;
    boost::shared_ptr<std::vector<TaskListenerPtr>> _taskListeners;
public:
    virtual void run();

    void addTaskListener(TaskListenerPtr taskListenerPtr);

    void notifyProgress(float percent, QString message);

protected:
    void taskBegin();

    void taskFinish(Result result);

    void taskProgress(float percent, QString message);

    void taskFail(std::exception exection);

private:
    void engage();

    void disengage();


Q_SIGNALS:

    void startTaskSignal();

    void finishTaskSignal(Result result);

    void taskExceptionSignal(std::exception exection);

    void taskProgressSignal(float percent, QString message);
};


#endif //COBOTCORE_QT_TEST_TASK_H
