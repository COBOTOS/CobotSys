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
    template<class Archive>
    void serialize(Archive& archive, const unsigned int version)
    {
        //archive & boost::serialization::make_nvp("m_day", m_day);
        archive & BOOST_SERIALIZATION_NVP(m_day);
        archive & BOOST_SERIALIZATION_NVP(m_month);
        archive & BOOST_SERIALIZATION_NVP(m_year);
    }
} date;


void save()
{
    std::ofstream file("archive.xml");
    boost::archive::xml_oarchive oa(file);
    date d(15, 8, 1947);
    //oa & boost::serialization::make_nvp("d", d);
    //将d序列化到 oa中，最终会调用自己写的serialize函数
    /*
#0  date::serialize<boost::archive::xml_oarchive> (this=0x7fffffffb9f0, archive=..., version=0)
    at /home/cobot/cobotos/cobotos/Sparrow/Sparrow.Boost.Test/test/testBoostSerializeStruct/testBoostSerializeStruct.cpp:25
#1  0x0000000000404be4 in boost::serialization::access::serialize<boost::archive::xml_oarchive, date> (ar=..., t=..., file_version=0)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/serialization/access.hpp:116
#2  0x0000000000404a89 in boost::serialization::serialize<boost::archive::xml_oarchive, date> (ar=..., t=..., file_version=0)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/serialization/serialization.hpp:68
#3  0x0000000000404865 in boost::serialization::serialize_adl<boost::archive::xml_oarchive, date> (ar=..., t=..., file_version=0)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/serialization/serialization.hpp:126
#4  0x00000000004046c5 in boost::archive::detail::oserializer<boost::archive::xml_oarchive, date>::save_object_data (this=0x60a2d0, ar=..., x=0x7fffffffb9f0)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/oserializer.hpp:153
#5  0x00007ffff7bbdec4 in boost::archive::detail::basic_oarchive_impl::save_object (this=0x60c860, ar=..., t=0x7fffffffb9f0, bos=...) at libs/serialization/src/basic_oarchive.cpp:294
#6  0x000000000040460b in boost::archive::detail::save_non_pointer_type<boost::archive::xml_oarchive>::save_standard::invoke<date> (ar=..., t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/oserializer.hpp:258
#7  0x0000000000404593 in boost::archive::detail::save_non_pointer_type<boost::archive::xml_oarchive>::invoke<date> (ar=..., t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/oserializer.hpp:315
#8  0x000000000040453f in boost::archive::save<boost::archive::xml_oarchive, date const> (ar=..., t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/oserializer.hpp:539
#9  0x00000000004044d8 in boost::archive::detail::common_oarchive<boost::archive::xml_oarchive>::save_override<date const> (this=0x7fffffffb760, t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/common_oarchive.hpp:71
#10 0x00000000004043d7 in boost::archive::basic_xml_oarchive<boost::archive::xml_oarchive>::save_override<date> (this=0x7fffffffb760, t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/basic_xml_oarchive.hpp:100
#11 0x0000000000404340 in boost::archive::detail::interface_oarchive<boost::archive::xml_oarchive>::operator<< <boost::serialization::nvp<date> > (this=0x7fffffffb760, t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/interface_oarchive.hpp:70
#12 0x00000000004041d6 in boost::archive::detail::interface_oarchive<boost::archive::xml_oarchive>::operator&<boost::serialization::nvp<date> > (this=0x7fffffffb760, t=...)
    at /home/cobot/cobotos/cobotos/install/x86-64-install/oss/include/boost/boost/archive/detail/interface_oarchive.hpp:77
#13 0x0000000000403b3f in save () at /home/cobot/cobotos/cobotos/Sparrow/Sparrow.Boost.Test/test/testBoostSerializeStruct/testBoostSerializeStruct.cpp:37
     */
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