//
// Created by xuzhenhai on 19-7-19.
//

#ifndef PEACOCKGUI_TASKMONITOR_H
#define PEACOCKGUI_TASKMONITOR_H


#include <QtCore/QThreadPool>
#include "Task.h"
#include "TaskListenerAdapter.h"
#include <boost/shared_ptr.hpp>
#include "boost/enable_shared_from_this.hpp"

class TaskMonitor : public TaskListenerAdapter, public boost::enable_shared_from_this<TaskMonitor> {

public:

    TaskMonitor();

    static boost::shared_ptr<TaskMonitor> createTaskMonitor();

    void postStask(Task *task);

private:
    QThreadPool *threadPool;

};


#endif //PEACOCKGUI_TASKMONITOR_H
