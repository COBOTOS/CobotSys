/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/

#ifndef COBOTOS_COMPARE_PTR_H
#define COBOTOS_COMPARE_PTR_H

#include <iostream>

class A{
public:
    A(int b);
    ~A();
private:
    int* _a;
};


#endif //COBOTOS_COMPARE_PTR_H