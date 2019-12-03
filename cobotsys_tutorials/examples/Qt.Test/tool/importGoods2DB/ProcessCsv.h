/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-05-08          qishimeng
============================================================== **/

#ifndef COBOTOS_PROCESSCSV_H
#define COBOTOS_PROCESSCSV_H

#include <vector>
#include <map>
#include <boost/function.hpp>
class ProcessCsv
{
    public:
        typedef void (write)(const std::string& value);
        typedef std::map<int,boost::function<write>>  FunctionMap;
        ProcessCsv();
        ~ProcessCsv();
        static bool write2DB(const std::vector<std::vector<std::string>>& data,const std::string& dbname);

    private:
//        FunctionMap _functionMap;
//        void doPocessid(const std::string& value);
//        void doPocessproductionid(const std::string& value);
//        void doPocesssku(const std::string& value);
//        void doPocessproductionName(const std::string& value);
//        void doPocessfactoryName(const std::string& value);
//        void doPocessproductionLength(const std::string& value);
//        void doPocessproductionWidth(const std::string& value);
//        void doPocessproductionHeight(const std::string& value);
//        void doPocessproductionWeight(const std::string& value);
//
//        void doPocessproductionToolNun(const std::string& value);
//        void doPocessproductionNeedFinger(const std::string& value);
//        void doPocessproductionAlgorithmTypy(const std::string& value);
//
//        void doPocessproductionalgorithmNumber(const std::string& value);
//
//        void doPocessproductionenable2DCapture(const std::string& value);
//        void doPocessproductionHDR(const std::string& value);
//        void doPocessproductionRGB(const std::string& value);
//        void doPocessproductiongain(const std::string& value);

};
#endif //COBOTOS_PROCESSCSV_H
