/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/

#include "testWeak_ptr.h"
testWeak_ptr::testWeak_ptr() {
    std::cout<<"testWeak_ptr"<<std::endl;
}
void testWeak_ptr::setvalue_wek(boost::shared_ptr <testWeak_ptr> p){
    _pw=p;
    std::cout<<"weak_ptr"<<std::endl;
}

void testWeak_ptr::setvalue(boost::shared_ptr <testWeak_ptr> p) {
    _ps=p;
    std::cout<<"shared_ptr"<<std::endl;
}

testWeak_ptr::~testWeak_ptr() {
    std::cout<<"~testWeak_ptr"<<std::endl;
}