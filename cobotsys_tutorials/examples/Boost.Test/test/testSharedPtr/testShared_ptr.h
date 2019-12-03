/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/

#ifndef COBOTOS_TESTSHARED_PTR_H
#define COBOTOS_TESTSHARED_PTR_H
#include <iostream>
class testshared_ptr{
public:
    testshared_ptr();
    testshared_ptr(int a);
    ~testshared_ptr();

private:
    int _a;
};
#endif //COBOTOS_TESTSHARED_PTR_H
