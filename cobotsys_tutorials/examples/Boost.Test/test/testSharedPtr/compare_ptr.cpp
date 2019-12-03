/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/


#include "compare_ptr.h"
A::A(int b):_a(&b){
    std::cout<<"A 构造函数   "<<(*_a)<<std::endl;
}
A::~A() {
    std::cout<<"~A"<<std::endl;
    delete  _a;
    _a= nullptr;
}