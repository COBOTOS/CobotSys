//
// Created by xuzhenhai on 19-7-20.
//

#ifndef SPARROW_QT_TEST_TASKLISTENERADAPTER_H
#define SPARROW_QT_TEST_TASKLISTENERADAPTER_H

#include <boost/shared_ptr.hpp>
#include "Task.h"
#include "TaskListener.h"

class Result;

class TaskListenerAdapter : public TaskListener {
public:
    TaskListenerAdapter() {};

    TaskListenerAdapter(StartFuncion startFuncion,
                        FinishFunction finishFunction,
                        ProcessFunction processFunction,
                        ErrorFunction errorFunction) {
        _startFuncion = startFuncion;
        _finishFunction = finishFunction;
        _processFunction = processFunction;
        _errorFunction = errorFunction;
    };

//    TaskListenerAdapter(StartFuncion startFuncion) {
//        _startFuncion = startFuncion;
//    };

    TaskListenerAdapter(FinishFunction finishFunction):_finishFunction(finishFunction) { };

//    TaskListenerAdapter(ProcessFunction processFunction) {
//        _processFunction = processFunction;
//    };

//    TaskListenerAdapter(ErrorFunction errorFunction) {
//        _errorFunction = errorFunction;
//    };

    virtual ~TaskListenerAdapter() {};

    void taskBegin(Task * task) {
        if (_startFuncion) {
            _startFuncion(task);
        }
    };

    void taskFinish(Task * task,Result result) {
        if (_finishFunction) {
            _finishFunction( task,result);
        }
    };

    void taskProgress(Task * task,float percent, QString message) {
        if (_processFunction) {
            _processFunction(task,percent, message);
        }
    };

    virtual void taskFail(Task * task,std::exception exection) {
        if (_errorFunction) {
            _errorFunction(task,exection);
        }
    };

    StartFuncion _startFuncion;
    FinishFunction _finishFunction;
    ProcessFunction _processFunction;
    ErrorFunction _errorFunction;
};


#endif //SPARROW_QT_TEST_TASKLISTENERADAPTER_H
