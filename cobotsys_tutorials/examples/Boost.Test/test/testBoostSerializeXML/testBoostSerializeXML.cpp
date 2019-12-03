/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-10-11          zhoupeng
============================================================== **/

//ref https://www.oschina.net/question/129540_35899
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>

void save()
{
    std::ofstream file("archive.xml");
    boost::archive::xml_oarchive oa(file);
    int array1[] = {34, 78, 22, 1, 910};
    //oa & boost::serialization::make_nvp("array1", array1);
    oa & BOOST_SERIALIZATION_NVP(array1);
}

void load()
{
    std::ifstream file("archive.xml");
    boost::archive::xml_iarchive ia(file);
    int restored[5]; // Need to specify expected array size
    //ia >> boost::serialization::make_nvp("restored", restored);
    ia >> BOOST_SERIALIZATION_NVP(restored);
    std::ostream_iterator<int> oi(std::cout, " ");
    std::copy(restored, restored+5, oi);
}


int main()
{
    save();
    load();
}