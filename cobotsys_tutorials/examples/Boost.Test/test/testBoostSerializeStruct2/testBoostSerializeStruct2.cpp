/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-10-11          zhoupeng
============================================================== **/

#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <iostream>
#include <fstream>

typedef struct date {
    unsigned int m_day;
    unsigned int m_month;
    unsigned int m_year;

    date( int d,  int m,  int y) : m_day(d), m_month(m), m_year(y)
    {}
    date() : m_day(1), m_month(1), m_year(2000)
    {}
    friend std::ostream& operator << (std::ostream& out, date& d)
    {
        out << "day:" << d.m_day
            << " month:" << d.m_month
            << " year:" << d.m_year;
        return out;
    }

} date;

namespace boost {
    namespace serialization {

        template<class Archive>
        void serialize(Archive& archive, date& d, const unsigned int version)
        {
            archive & BOOST_SERIALIZATION_NVP(d.m_day);
            archive & BOOST_SERIALIZATION_NVP(d.m_month);
            archive & BOOST_SERIALIZATION_NVP(d.m_year);
        }

    } // namespace serialization
} // namespace boost

void save()
{
    std::ofstream file("archive.xml");
    boost::archive::xml_oarchive oa(file);
    date d(15, 8, 1947);
    //oa & boost::serialization::make_nvp("d", d);
    //将d序列化到 oa中，最终会调用自己写的serialize函数
    oa & BOOST_SERIALIZATION_NVP(d);
}

void load()
{
    std::ifstream file("archive.xml");
    boost::archive::xml_iarchive ia(file);
    date dr;
    //ia >> boost::serialization::make_nvp("dr", dr);
    ia >> BOOST_SERIALIZATION_NVP(dr);
    std::cout << dr;
}

int main()
{
    save();
    load();
}