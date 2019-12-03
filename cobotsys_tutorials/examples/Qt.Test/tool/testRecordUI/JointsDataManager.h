/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-7-11           tangmingwu
============================================================== **/


#ifndef COBOTOS_JOINTSDATAMANAGER_H
#define COBOTOS_JOINTSDATAMANAGER_H
#include <map>
#include <string>
#include <vector>
#include <parser/XMLParser.h>

using namespace TIGER_COMMON_NS_API;

class JointsDataManager
{
public:

    typedef std::map<std::string,std::string> KeyValueMap;

    JointsDataManager();
    bool jointsMap2Xml(const std::string fileName);
    bool xml2JointsMap(const std::string fileName, std::map<std::string,std::vector<std::string>>& tempMap);
    bool addJoints(int index, std::vector<double> & correntJoints);
    bool deleteJoints(int index);
    bool modifyJoints(int index, std::vector<double>& correntJoints);
    void printMap()const;
private:
    KeyValueMap _jointsMap;
    boost::shared_ptr<XMLParser> _xmlParser;
};


#endif //COBOTOS_JOINTSDATAMANAGER_H
