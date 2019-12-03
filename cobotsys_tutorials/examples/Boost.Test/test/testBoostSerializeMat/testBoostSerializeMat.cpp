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
#include <opencv2/opencv.hpp>
#include <logger/Logger.h>

namespace boost {
    namespace serialization {
        template<class Archive>
        void serialize(Archive & ar, cv::Mat& m, const unsigned int version) {
            int cols = m.cols;
            int rows = m.rows;
            size_t elemSize = m.elemSize();
            size_t elemType = m.type();

            ar & BOOST_SERIALIZATION_NVP(cols);
            ar & BOOST_SERIALIZATION_NVP(rows);
            ar & BOOST_SERIALIZATION_NVP(elemSize);
            ar & BOOST_SERIALIZATION_NVP(elemType); // element type.

            if(m.type() != elemType || m.rows != rows || m.cols != cols) {
                m = cv::Mat(rows, cols, elemType, cv::Scalar(0));
            }

            size_t dataSize = cols * rows * elemSize;
            LOG_INFO << " datasize is " << dataSize;


            for (size_t dc = 0; dc < dataSize; dc++) {
                std::stringstream ss;
                ss << "elem_"<<dc;
                ar & boost::serialization::make_nvp(ss.str().c_str(), m.data[dc]);
            }
        }
    } // namespace serialization
} // namespace boost

void save()
{
    std::ofstream file("archive.xml");
    boost::archive::xml_oarchive oa(file);
    cv::Mat pointCloud(cv::Size(3, 4), CV_8U);

    pointCloud.at<float>(0,0)=56;
    pointCloud.at<float>(1,0)=120;

    //oa & boost::serialization::make_nvp("d", d);
    //将d序列化到 oa中，最终会调用自己写的serialize函数
    oa & BOOST_SERIALIZATION_NVP(pointCloud);
}

void load()
{
    std::ifstream file("archive.xml");
    boost::archive::xml_iarchive ia(file);
    cv::Mat pointCloud;
    //ia >> boost::serialization::make_nvp("dr", dr);
    ia >> BOOST_SERIALIZATION_NVP(pointCloud);
}

int main()
{
    save();
    load();
}