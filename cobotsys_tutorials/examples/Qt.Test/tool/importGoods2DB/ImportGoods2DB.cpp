/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-05-08          qishimeng
============================================================== **/
#include <parser/CSVParser.h>
#include <logger/Logger.h>
#include "ProcessCsv.h"

using namespace TIGER_COMMON_NS_API;
int main(int argc , char ** argv)
{
    if(argc !=3)
    {
        LOG_ERROR << "too few parm,please importGood2DB xxx.csv xxx.db3 !";
        return -1;
    }
    boost::shared_ptr<CSVParser> csvParser = CSVParser::create();
    std::vector<std::vector<std::string>> temp = csvParser->parseCSV(argv[1]);
    ProcessCsv::write2DB(temp,argv[2]);
    return 0;
}

