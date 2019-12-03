//
// Created by xuzhenhai on 19-7-20.
//

#ifndef SPARROW_QT_TEST_TASKLISTENER_H
#define SPARROW_QT_TEST_TASKLISTENER_H

#include <boost/shared_ptr.hpp>
#include "Result.h"
#include "Task.h"

class Task;

class TaskListener  {
public:
    TaskListener() {};

    virtual ~TaskListener() {};

    virtual void taskBegin(Task *task) = 0;

    virtual void taskFinish(Task *task, Result result) = 0;

    virtual void taskProgress(Task *task, float percent, QString message) = 0;

    virtual void taskFail(Task *task, std::exception exection) = 0;

    typedef boost::function<void(Task *)> StartFuncion;
    typedef boost::function<void(Task *, Result)> FinishFunction;
    typedef boost::function<void(Task *, float, QString)> ProcessFunction;
    typedef boost::function<void(Task *, std::exception)> ErrorFunction;
};

#endif //SPARROW_QT_TEST_TASKLISTENER_H
