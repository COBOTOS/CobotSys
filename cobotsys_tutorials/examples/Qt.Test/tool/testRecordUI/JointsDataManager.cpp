/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-7-11           tangmingwu
============================================================== **/
#include "JointsDataManager.h"
#include <boost/algorithm/string.hpp>
#include <logger/Logger.h>

//初始化
JointsDataManager::JointsDataManager()
{
    _xmlParser = XMLParser::create();
}

//保存示教点Joints信息到xml文件
bool JointsDataManager::jointsMap2Xml(const std::string fileName)
{
    if(_xmlParser->saveXml(fileName, _jointsMap))
    {
        return true;
    }
    else
    {
        LOG_ERROR << "保存map失败！";
        return false;
    }
}

//打开xml文件，并返回显示到UI界面，如果UI界面有Joints信息，则提醒是否清除，或可以转到保存xml文件功能
bool JointsDataManager::xml2JointsMap(const std::string fileName, std::map<std::string,std::vector<std::string>>& tempMap)
{
    if(fileName.empty())
    {
        LOG_ERROR << "打开的文件名为空！";
        return false;
    }
    tempMap.clear();//确保tempMap为空
    _jointsMap = _xmlParser->parseXML(fileName);//map<string,string>

    for(KeyValueMap::const_iterator iter = _jointsMap.begin();iter != _jointsMap.end(); ++iter)
    {
        std::vector<std::string> vec;
        boost::split(vec, iter->second,boost::is_any_of(","));
        if(vec.size() != 6)
        {
            LOG_ERROR << "joint个数不为6，解析失败！";
            return false;
        }
        tempMap[iter->first] = vec;//map<string,vector<string>>
    }
    return true;

}

bool JointsDataManager::addJoints(int index, std::vector<double> & correntJoints)
{
    if(correntJoints.size() != 6)
    {
        LOG_ERROR << "joint个数不为6，添加失败！";
        return false;
    }
    std::string key = std::to_string(index);
    std::string value;
    value = std::to_string(correntJoints[0]);
    for(int i=1; i < correntJoints.size(); ++i)
    {
        value += ",";
        value += std::to_string(correntJoints[i]);
    }
    if(_jointsMap.find(key) == _jointsMap.end())
    {
        _jointsMap[key] = value;
    }
    else
    {
        return false;
    }
    return true;
}

bool JointsDataManager::deleteJoints(int index)
{
    std::string key = std::to_string(index);
    KeyValueMap::iterator iter = _jointsMap.find(key);
    if(iter != _jointsMap.end())
    {
        _jointsMap.erase(iter);
        return true;
    }
    else
    {
        LOG_ERROR << "不存在该序号点，删除失败！";
        return false;
    }
}

bool JointsDataManager::modifyJoints(int index, std::vector<double>& correntJoints)
{
    std::string key = std::to_string(index);
    KeyValueMap::iterator iter = _jointsMap.find(key);
    if(iter != _jointsMap.end())
    {
        std::string value;
        value = std::to_string(correntJoints[0]);
        for(int i=1; i < correntJoints.size(); ++i)
        {
            value += ",";
            value += std::to_string(correntJoints[i]);
        }
        _jointsMap[key] = value;
        return true;
    }
    else
    {
        LOG_ERROR << "不存在该序号点，修改失败！";
        return false;
    }
}

void JointsDataManager::printMap()const
{
    std::map<std::string, std::string>::const_iterator iter;
    for(iter = _jointsMap.begin();iter != _jointsMap.end();++iter)
    {
        LOG_INFO << iter->first <<" "<<iter->second;
    }
}