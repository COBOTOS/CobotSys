//
// Created by zhoupeng on 19-6-13.
//
#include <logger/Logger.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <vector>

void boostThreadCallback(int counter) {
    LOG_INFO << "invoke boostThreadCallback:" << counter;
    sleep(1);
}

void testCreateThread(std::vector<boost::shared_ptr<boost::thread>>& threads, int count) {
    for (int i = 0; i < count ; i++) {
        threads.push_back(boost::make_shared<boost::thread>(boost::bind(boostThreadCallback, i)));
        threads[i]->detach();
        //threads[i]->join();
    }
}

void createManyThreads() {
    std::vector<boost::shared_ptr<boost::thread>> threads;
    int count = 30;
    //threads.reserve(count);
    testCreateThread(threads, count);

    LOG_INFO << "begin sleep 3s";
    sleep(3);
    LOG_INFO << "finish sleep 3s";
}

void createOneThread() {
    boost::shared_ptr<boost::thread> oneThread =
            boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(boostThreadCallback, 0)));
    oneThread->detach();
    LOG_INFO << "begin sleep 2s";
    sleep(2);
    LOG_INFO << "finish sleep 2s";
}

int main(int argc, char** argv) {
    //createOneThread();
    createManyThreads();
}
