/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-17           tangmingwu
============================================================== **/


#ifndef PROJECT_LSRLOST_H
#define PROJECT_LSRLOST_H
#include <LSRStatic.h>
#include <logger/Logger.h>
#include <logger/GlogConfig.h>
#include <LocalStorageRepositoryAPIKey.h>
#include <LocalStorageRepositoryInterface.h>
#include <boost/shared_ptr.hpp>
#include <dlfcn.h>
#include <systemproperty/SystemProperty.h>

using namespace LION_LSR_NS_API;

class LsrLost{
public:
    LsrLost(){
        _factory = LocalStorageRepositoryFactoryInterface::create();
    }
    void start(){
        _factory->createLocalStorageRepository();
    }
private:
    boost::shared_ptr<LocalStorageRepositoryFactoryInterface> _factory;
};
#endif //PROJECT_LSRLOST_H
