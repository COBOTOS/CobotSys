/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-7-9           QiShiMeng                Initial
============================================================== **/
#include <logger/Logger.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
void fun(int a,int b)
{
    LOG_INFO << a+b;
}
void fun(double a, double b, double c)
{
    LOG_INFO << a+b+c;
}
int main()
{
    int a=1,b=2;
    double c =3.0,d=4.0;
    auto funDouble = boost::bind(&fun,c,d,c);
    auto funInt = boost::bind(&fun,a,b);
    funDouble();
    funInt();
    return 0;
}