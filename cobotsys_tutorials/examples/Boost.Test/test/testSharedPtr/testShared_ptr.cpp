/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/

#include "testShared_ptr.h"
testshared_ptr::testshared_ptr():_a(0) {
    std::cout<<_a<<"构造"<<std::endl;
}
testshared_ptr::testshared_ptr(int a) :_a(a){
    std::cout<<_a<<"参数构造"<<std::endl;
}
testshared_ptr::~testshared_ptr() {
    std::cout << "~testshared_ptr:" << _a << std::endl;
}