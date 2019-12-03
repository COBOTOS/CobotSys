//
// Created by lijun on 19-4-7.
//
#include "fileSystem.h"

#include <boost/utility/result_of.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <cassert>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace std;

int add(int a, int b)
{
    std::cout << a + b << std::endl;
    return 1;
}

bool compare(int i, int j)
{
    return i > j;
}

void add(int i, int j, std::ostream &os)
{
    os << i + j << std::endl;
}

struct world
{
    void hello(std::ostream &os)
    {
        os << "Hello, world!" << std::endl;
    }
};



int main(int argc, char **argv)
{
//    int x = 5;
//    boost::reference_wrapper<int> rw(x);
//
//    std::cout<<rw<<std::endl;
//    rw.get() = 10; //获得引用
//    std::cout<<rw<<std::endl;
//    *rw.get_pointer() = 15; //获得指针
//    std::cout<<rw<<std::endl;
//
////    int n = rw; //隐式转换为int
////    assert(x == rw); //隐式转换为int
//
//    (int&)rw = 20; //显示转换为int&，这样可以作为左值
//    assert(x == 20);
//
//    boost::reference_wrapper<int> rw2(rw); //拷贝构造
//    rw.get() = 25;
//    assert(rw2.get() == 25);//rw2也是x的一个引用对象

//    std::vector<int> v;
//    v.push_back(1);
//    v.push_back(3);
//    v.push_back(2);
//    std::for_each(v.begin(), v.end(), boost::bind(add, 10, _1));
//
//    std::sort(v.begin(), v.end(), boost::bind(compare, _1, _2));
//
//    std::for_each(v.begin(), v.end(), boost::bind(add, 10, _1, boost::ref(std::cout)));
//
//    try
//    {
//        boost::function<int (const char*)> f;
//        f = std::strlen;
//        std::cout << f("1609") << std::endl;
//    }
//    catch (boost::bad_function_call &ex)
//    {
//        std::cout << ex.what() << std::endl;
//    }
//
//    boost::function<void (world*, std::ostream&)> f = &world::hello;
//    assert(f.contains(&world::hello));
//    world w;
//    f(&w, boost::ref(std::cout));


//    boost::property_tree::ptree root,pt;
//    read_json("/home/lijun/cobotos/Eagle/Eagle.Vision.Test/test/testBoost/conf.json",root);
//
//    std::cout<<root.get<std::string>("conf.gui")<<std::endl;
//
//    pt = root.get_child("conf");
//    std::cout<<pt.get<std::string>("gui")<<std::endl;
//
//    if(root.count("classes"))
//    {
//        pt = root.get_child("classes");
//        std::vector<int> subArray;
////    for(BOOST_AUTO(pos, pt.begin()); pos != pt.end(); ++pos)
////    {
////        int job = pos->second.get_value<int>();
////        std::cout<<job<<std::endl;
////    }
//        for (auto iter = pt.begin();iter != pt.end();++iter)
//        {
//            int job = iter->second.get_value<int>();
//            std::cout<<job<<std::endl;
//            subArray.emplace_back(job);
//        }
//        if (subArray.size() % 4 != 0) {
//            std::cout << "the box json format is error"<<std::endl;
//        }
//        std::cout<<"size:"<<subArray.size()<<std::endl;
//    }
    std::stringstream outStream;
    boost::property_tree::ptree pt,taskJson;

    taskJson.put("orderNumber", "111");
    pt.put("valid", true);
    pt.add_child("taskJson",taskJson);
    boost::property_tree::write_json(outStream,pt);
    std::cout <<"send to cgrasp string:"<<outStream.str()<<std::endl;

    return 1;
}
