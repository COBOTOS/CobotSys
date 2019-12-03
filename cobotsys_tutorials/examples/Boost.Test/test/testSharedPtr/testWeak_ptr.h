/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           			Name      			    Description of Change
 19-4-15                 zhangxiaohan
============================================================== **/

#ifndef COBOTOS_TESTWEAK_PTR_H
#define COBOTOS_TESTWEAK_PTR_H

#include <boost/weak_ptr.hpp>
#include <iostream>

class testWeak_ptr {
public:
    testWeak_ptr();
    void setvalue_wek(boost::shared_ptr<testWeak_ptr> p);
    void setvalue(boost::shared_ptr<testWeak_ptr> p);
    ~testWeak_ptr();

private:
    boost::weak_ptr<testWeak_ptr> _pw;
    boost::shared_ptr<testWeak_ptr> _ps;
};
#endif //COBOTOS_TESTWEAK_PTR_H
