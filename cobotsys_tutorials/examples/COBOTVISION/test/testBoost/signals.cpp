//
// Created by lijun on 19-4-8.
//
#include <boost/utility/result_of.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/signals2.hpp>
#include <cassert>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <numeric>


using namespace boost::signals2;
using namespace std;

void slots1()
{
    std::cout<<"slots1 called"<<std::endl;
}

void slots2()
{
    std::cout<<"slots2 called"<<std::endl;
}


template<int N>
struct Slot
{
    int operator()(int x)
    {
        std::cout << "Slot current N * x value is : " << std::endl;

        return (N * x);
    }
};


template<typename T>
class combiner
{
public:
    typedef pair<T, T> result_type;
    combiner(T t = T()) : v(t)
    {

    }

    template<typename InputIterator>
    result_type operator()(InputIterator begin, InputIterator end) const
    {
        if (begin == end)
        {
            return result_type();
        }

        vector<T> vec(begin, end);
        T sum = accumulate(vec.begin(), vec.end(), v);
        T max = *max_element(vec.begin(), vec.end());

        return result_type(sum, max);
    }

private:
    T v;
};


void test1()
{
    signal<void()> sig;
    sig.connect(&slots1);
    sig.connect(&slots2);
    sig();


    boost::signals2::signal<int(int), combiner<int> > sig2;

    sig2.connect(Slot<10>());
    sig2.connect(Slot<20>());
    sig2.connect(Slot<3>());

    BOOST_AUTO(x, sig2(2));
    std::cout << x.first << ", " << x.second << std::endl;
}