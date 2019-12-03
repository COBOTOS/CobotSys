//
// Created by qishimeng on 19-6-14.
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

using namespace LION_LSR_NS_API;
using namespace boost;
int main(int argc, char** argv)
{
    long i = 0;
    int lsrPid = 0;
    if (argc > 1) {
        lsrPid = atoi(argv[1]);
    }
    int delayMs = 0;
    if (argc > 2) {
        delayMs = atoi(argv[2]);
    }
    while(1)
    {
        long curTimeMs1 = BoostTime::getCurTimeMs();
        std::string value = LSRStatic::getItem(sys_cgrasp_deeplearning_box_verticalgrasp_enabled);
        long curTimeMs2 = BoostTime::getCurTimeMs();
        if ((curTimeMs2 - curTimeMs1) > 5000) {
            if (lsrPid > 0) {
                LOG_ERROR << "send sigstop to lsrPid:" << lsrPid;
                kill(lsrPid, SIGSTOP);
            }
            LOG_FATAL << "too slow for getItem  costMs=" << (curTimeMs2-curTimeMs1);
        }
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