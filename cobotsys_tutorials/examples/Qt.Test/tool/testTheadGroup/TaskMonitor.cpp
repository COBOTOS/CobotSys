//
// Created by xuzhenhai on 19-7-19.
//

#include <boost/boost/make_shared.hpp>
#include "TaskMonitor.h"

TaskMonitor::TaskMonitor() {
    threadPool = new QThreadPool();
    threadPool->setMaxThreadCount(3);
}

boost::shared_ptr<TaskMonitor> TaskMonitor::createTaskMonitor() {
    static boost::shared_ptr<TaskMonitor>  instance = boost::make_shared<TaskMonitor>();
    return instance;
}

void TaskMonitor::postStask(Task *task) {
    task->addTaskListener(shared_from_this());
    threadPool->start(task);
}