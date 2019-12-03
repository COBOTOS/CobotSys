//
// Created by zhoupeng on 19-6-12.
//
#include <LSRStatic.h>
#include <logger/Logger.h>
#include <logger/GlogConfig.h>
#include <LocalStorageRepositoryAPIKey.h>
#include <LocalStorageRepositoryInterface.h>
#include <boost/shared_ptr.hpp>
#include <dlfcn.h>
#include <systemproperty/SystemProperty.h>

using namespace LION_LSR_NS_API;
using namespace boost;

void testUseLSRStatic() {
    std::string value = LSRStatic::getItem(app_cgrasp_production_name);
    LOG_INFO << "value=" << value << " key=" << app_cgrasp_production_name;
}

void testUseFactory() {
    shared_ptr<LocalStorageRepositoryFactoryInterface> factory
    = LocalStorageRepositoryFactoryInterface::create("libLionLocalStorageRepositoryClientSystem.so","libLionLocalStorageRepositoryDBusClient.so");
    shared_ptr <LocalStorageRepositoryInterface> lsr = factory->createLocalStorageRepository();
    std::string value = lsr->getItem(app_cgrasp_production_name);
    LOG_INFO << "value=" << value << " key=" << app_cgrasp_production_name;
}

void testLoadLib() {
    boost::shared_ptr<SystemProperty>  sysp = SystemProperty::create();
    std::string fullLibName = sysp->getDevelLibRootPath();
    std::string lib1 = fullLibName + "/" + "libLionLocalStorageRepositoryClientSystem.so";
    void *handle1 = dlopen(lib1.c_str(),RTLD_LAZY);
    dlclose(handle1);
}

int main(int argc, char ** argv) {
    GlogConfig::config(__COBOTSYS_MODULE_NAME__);
#if 1
    testUseFactory();
    testUseFactory();
    testUseFactory();
#else
    testLoadLib();
#endif
    LOG_INFO << "finish test";
    return 0;
}