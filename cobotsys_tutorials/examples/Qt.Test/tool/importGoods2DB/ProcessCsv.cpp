/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-05-08          qishimeng
============================================================== **/

#include "ProcessCsv.h"
#include <SQLiteCpp/SQLiteCpp.h>
#include <SQLiteCpp/VariadicBind.h>
#include <logger/Logger.h>
#include <parser/StringTransform.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>

const std::string dbname = "/home/cobot/cobotos/install/x86-64-install/devel/data/Dolphin.CGrasp.EcsUI/GoodsDataBase.db3";
const std::string tablename = "cgrasp";
using namespace TIGER_COMMON_NS_API;
ProcessCsv::ProcessCsv() {
}
ProcessCsv::~ProcessCsv() {

}
bool ProcessCsv::write2DB(const std::vector<std::vector<std::string>>& data,const std::string& DBname) {
    SQLite::Database    db(DBname, SQLite::OPEN_READWRITE);
    SQLite::Statement insert(db,"INSERT INTO "+tablename+"(productId,sku,productName,factoryName,productLength,productWidth,productHeight,productWeight,toolNumber,needFinger,AlgorithmType,algorithmNumber,enable2DCapture,HDR,RGB,gain)"+"VALUES(?,?,?,?,? ,?,?,?,?,? ,?,?,?,?,?, ?)");
//    SQLite::Statement select(db,"SELECT * FROM "+tablename+" WHERE KEY = ?");
//    SQLite::Statement update(db,"UPDATE "+tablename+" SET VALUE = ? WHERE KEY = ?");
    for(std::size_t i = 0;i < data.size();++i)
    {
        for(std::size_t j = 0;j<data[i].size();++j)
        {
            std::cout << " " << (data[i])[j];
        }
        std::cout << std::endl;
    }
    try {
        SQLite::Transaction transaction(db);
        for(std::size_t i = 1;i<data.size();++i)
        {

            insert.bind(1, boost::trim_copy(data[i].at(1)));//prouctionid
            insert.bind(2, boost::trim_copy(data[i].at(0)));//sku
            insert.bind(3,"null");
            insert.bind(4,"null");

            insert.bind(5,boost::lexical_cast<double>(boost::trim_copy(data[i].at(2))));//length
            insert.bind(6,boost::lexical_cast<double>(boost::trim_copy(data[i].at(3))));//width
            insert.bind(7,boost::lexical_cast<double>(boost::trim_copy(data[i].at(4))));//height
            insert.bind(8,boost::lexical_cast<double>(boost::trim_copy(data[i].at(5)))*0.001);//weight

            insert.bind(9,boost::lexical_cast<int>(boost::trim_copy(data[i].at(6))));//toolNunber
            insert.bind(10,boost::lexical_cast<int>(boost::trim_copy(data[i].at(9))));//needFinder
            insert.bind(11,boost::lexical_cast<int>(boost::trim_copy(data[i].at(7))));//AlgorithmTypy
            insert.bind(12,boost::lexical_cast<int>(boost::trim_copy(data[i].at(8))));//AlgorithmNum

            insert.bind(13,boost::lexical_cast<int>(boost::trim_copy(data[i].at(10))));//enable2DCapture
            insert.bind(14,boost::lexical_cast<int>(boost::trim_copy(data[i].at(11))));//HDR
            insert.bind(15,boost::lexical_cast<int>(boost::trim_copy(data[i].at(13))));//RGB

            insert.bind(16,boost::lexical_cast<int>(boost::trim_copy(data[i].at(12))));//gain
            LOG_INFO << "END";
            insert.exec();
            insert.reset();
        }

        transaction.commit();
    }
    catch(std::exception& e)
    {
        LOG_ERROR << "exception : " << e.what();
        return false;
    }


    return true;
}
//void ProcessCsv::doPocessid(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionid(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocesssku(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionName(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessfactoryName(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionLength(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionWidth(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionHeight(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionWeight(const std::string& value)
//{
//
//}
//
//void ProcessCsv::doPocessproductionToolNun(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionNeedFinger(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionAlgorithmTypy(const std::string& value)
//{
//
//}
//
//void ProcessCsv::doPocessproductionalgorithmNumber(const std::string& value)
//{
//
//}
//
//void ProcessCsv::doPocessproductionenable2DCapture(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionHDR(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductionRGB(const std::string& value)
//{
//
//}
//void ProcessCsv::doPocessproductiongain(const std::string& value)
//{
//
//}

