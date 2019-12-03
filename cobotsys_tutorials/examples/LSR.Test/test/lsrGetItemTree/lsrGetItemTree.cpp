//
// Created by qishimeng on 19-6-17.
//
#include <LSRStatic.h>
#include <logger/Logger.h>
#include <logger/GlogConfig.h>
#include <LocalStorageRepositoryAPIKey.h>
#include <LocalStorageRepositoryInterface.h>
#include <boost/shared_ptr.hpp>
#include <dlfcn.h>
#include <systemproperty/SystemProperty.h>
#include <boosttime/BoostTime.h>
#include <sys/types.h>
#include <signal.h>
#include <boosttimer/BoostTimer.h>
#include <boosttimer/BoostTimerFactory.h>

using namespace LION_LSR_NS_API;
using namespace boost;
int g_getItemTree_Times = 0;
void processGetItemTreeCostTooTime(boost::shared_ptr<BoostTimer> timer, long pid, long beginTimeMs)
{
    if(timer->taskCanceled())
    {
        return;
    }
    else
    {
        long curTimeMs2 = BoostTime::getCurTimeMs();
        if (pid > 0) {//发送信号给进程
            LOG_ERROR << "send sigstop to LSR service with Pid:" << pid;
            kill(pid, SIGSTOP);

        }
        kill(getpid(), SIGSTOP);
        LOG_ERROR << "too slow for getItem, has send SIGSTOP to self, debug me please; costMs=" << (curTimeMs2 - beginTimeMs)
                  << " g_getItemTree_Times=" << g_getItemTree_Times;
    }
}

void getAllItems(boost::shared_ptr<LocalStorageRepositoryInterface> lsr, long *pCount) {
#if 0
    g_getItemTree_Times++;
    LOG_INFO << "begin app getItemTree[" << g_getItemTree_Times << "]";
    std::map<std::string,std::string> maptree  = lsr->getItemTree("app","");
    LOG_INFO << "finish app getItemTree[" << g_getItemTree_Times << "]"<< "　count="<<*pCount;

    g_getItemTree_Times++;
    LOG_INFO << "begin sys getItemTree[" << g_getItemTree_Times << "]";
    std::map<std::string,std::string> maptree1  = lsr->getItemTree("sys","");
    LOG_INFO << "finish sys getItemTree[" << g_getItemTree_Times << "]"<< "　count="<<*pCount;
#endif
    g_getItemTree_Times++;
    long curTimeMs1 = BoostTime::getCurTimeMs();
    LOG_INFO << "begin hw getItemTree[" << g_getItemTree_Times << "]";
    std::map<std::string,std::string> maptree2  = lsr->getItemTree("hw","");
    long curTimeMs2 = BoostTime::getCurTimeMs();
    LOG_INFO << "finish hw getItemTree[" << g_getItemTree_Times << "]" << "　count="<<*pCount;
#if 0
    if (*pCount % 10 ==0)
    {
        LOG_INFO <<"["<<*pCount<< "] app.cnt=" << maptree.size() << " sys.cnt=" << maptree1.size() << " hw.cnt=" << maptree2.size()
        << " g_getItemTree_Times=" << g_getItemTree_Times << " costMs= " << (curTimeMs2-curTimeMs1);
    }
#else
    LOG_INFO <<"["<<*pCount<< "] "<< " hw.cnt=" << maptree2.size()  << " g_getItemTree_Times=" << g_getItemTree_Times
             << " costMs=" << (curTimeMs2-curTimeMs1);
#endif
    (*pCount)++;
}
void getItem(boost::shared_ptr<LocalStorageRepositoryInterface>  lsr, long *pCount)
{
    std::string value = LSRStatic::getItem(sys_cgrasp_deeplearning_box_verticalgrasp_enabled);


    if (*pCount % 1000 ==0) {
        LOG_INFO <<"["<<*pCount<< "] key = " << sys_cgrasp_deeplearning_box_verticalgrasp_enabled;
    }
    (*pCount)++;
}
int main(int argc, char** argv)
{
    long i = 0;
    int lsrPid = 0;
    bool isGetItem = false;
    if (argc > 1) {
        lsrPid = atoi(argv[1]);
    }
    int delayMs = 1000;
    long count = 0;
    if (argc > 2) {
        delayMs = atoi(argv[2]);
    }
    if(argc >3)
    {
        isGetItem = true;
    }
    boost::shared_ptr<LocalStorageRepositoryFactoryInterface> factory = LocalStorageRepositoryFactoryInterface::create();
    boost::shared_ptr<LocalStorageRepositoryInterface>  lsr = factory->createLocalStorageRepository();;
    boost::shared_ptr<TIGER_COMMON_NS_API::BoostTimer> Timer = BoostTimerFactory::createTimer();

    while(1)
    {
        long curTimeMs1 = BoostTime::getCurTimeMs();
        Timer->setTask(boost::bind(processGetItemTreeCostTooTime,Timer, lsrPid, curTimeMs1), 22000*1000);
        if(isGetItem)
        {
            getItem(lsr, &count);
        }
        else
        {
            getAllItems(lsr, &count);
        }

        Timer->cancelTask();

        if (delayMs> 0) {
            usleep(delayMs * 1000);
        }
        if (100000==i)
        {
            LOG_ERROR << "Living!!!!!!!!!!!!!!";
            i= 0;
        }
        i++;
    }
    return 0;

}

