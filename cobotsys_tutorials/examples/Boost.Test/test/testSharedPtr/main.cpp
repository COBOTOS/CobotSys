/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "testShared_ptr.h"
#include "testWeak_ptr.h"
#include "compare_ptr.h"
#include <logger/Logger.h>
using namespace std;


void testshared_ptr01()
{

    //使用智能指针创建对象
    boost::shared_ptr<testshared_ptr> tes_ptrO(new testshared_ptr(5));
    //显示指针计数
    LOG_INFO<<"tes_ptrO count"<<tes_ptrO.use_count();

    boost::shared_ptr<testshared_ptr> tes_ptr1=tes_ptrO;
    LOG_INFO<<"tes_ptrO count"<<tes_ptrO.use_count();
    LOG_INFO<<"tes_ptr1 count"<<tes_ptr1.use_count();


}
void testWeak_ptr01()
{
    //weak_ptr测试

    boost::shared_ptr<testWeak_ptr> tes_wek1(new testWeak_ptr);
    boost::shared_ptr<testWeak_ptr> tes_wek2(new testWeak_ptr);
    boost::weak_ptr<testWeak_ptr> wp1(tes_wek1);
    boost::weak_ptr<testWeak_ptr> wp2(tes_wek2);

    tes_wek1->setvalue_wek(tes_wek2);
    tes_wek2->setvalue(tes_wek1);//setvalue中使用shared_ptr会导致tes_wek1两次引用计数

    LOG_INFO<<wp1.use_count();
    LOG_INFO<<wp2.use_count();
//tes_wek2的弱指针强化
    if(!wp2.expired()){
        boost::shared_ptr<testWeak_ptr>  tes_wek3 = wp2.lock();//强化
        LOG_INFO<<"转换后"<<wp1.use_count();
        LOG_INFO<<"转换后"<<wp2.use_count();
    }//离开作用域后tes_wek3自动解析,tes_wek2引用计数变为1

    LOG_INFO<<"test  "<<tes_wek1.use_count();
    LOG_INFO<<"test  "<<tes_wek2.use_count();
}

void compare01(){
    int c=3;
    A a1(c);
    A a2(a1);//复制的时候a2中的_a也指向c的地址,导致程序结束时调用两析构函数,释放两次内存,出现系统崩溃.
}

void testUseNakePointer() {
    testshared_ptr *test_ptrO = new testshared_ptr(5);
    delete test_ptrO;
}


int main() {
    //testshared_ptr01();
    //testWeak_ptr01();

    boost::shared_ptr<testshared_ptr> oneObj;
    if (oneObj) {
        LOG_INFO << "oneObj is null";
    } else {
        LOG_INFO << "oneObj is not null";
    }

    return 0;
}