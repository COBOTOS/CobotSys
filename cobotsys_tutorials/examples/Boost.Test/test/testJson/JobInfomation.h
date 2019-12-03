//
// Created by cobot on 19-6-17.
//

#ifndef COBOTOS_JOBINFOMATION_H
#define COBOTOS_JOBINFOMATION_H

#include <sstream>
#include <map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional/optional.hpp>
using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;
class JobInfomation{
public:
    std::string orderNumber="";
    std::string nodeNumber="";
    std::string productionNumber="";
    int pickQuantity=-1;
    int finishQuantity=-1;
    std::string targetContainerLocation="";
    std::string sourceContainerNumber="";
    std::string status="";
    std::string time="";

public:
    JobInfomation(){

    }

    JobInfomation(ptree tree){
        initInfomation(tree);
    }

    JobInfomation(std::string json){
        ptree pt2;
        std::istringstream is (json);
        read_json (is, pt2);
        initInfomation(pt2);
    }

    ptree toPtree(){
        ptree pt;
        pt.put ("orderNumber", orderNumber);
        pt.put ("nodeNumber", nodeNumber);
        pt.put ("productionNumber", productionNumber);
        pt.put ("pickQuantity", pickQuantity);
        pt.put ("finishQuantity", finishQuantity);
        pt.put ("targetContainerLocation", targetContainerLocation);
        pt.put ("sourceContainerNumber", sourceContainerNumber);
        pt.put ("status", status);
        pt.put ("time", time);
        return pt;
    }

    std::string toString(){
        ptree pt = toPtree();
        std::ostringstream buf;
        write_json (buf, pt, false);
        std::string json = buf.str();
        return json;
    }

    void initInfomation(ptree pt2){
        auto child = pt2.get_child_optional( "orderNumber" );
        if( !child ){
            orderNumber="";
        }else{
            orderNumber = pt2.get<std::string>("orderNumber");
        }


        child = pt2.get_child_optional( "nodeNumber" );
        if( !child ){
            nodeNumber="";
        }else{
            nodeNumber = pt2.get<std::string>("nodeNumber");
        }

        child = pt2.get_child_optional( "productionNumber" );
        if( !child ){
            productionNumber="";
        }else{
            productionNumber = pt2.get<std::string>("productionNumber");
        }

        child = pt2.get_child_optional( "pickQuantity" );
        if( !child ){
            pickQuantity=-1;
        }else{
            pickQuantity = pt2.get<int>("pickQuantity");
        }

        child = pt2.get_child_optional( "finishQuantity" );
        if( !child ){
            finishQuantity=-1;
        }else{
            finishQuantity = pt2.get<int>("finishQuantity");
        }

        child = pt2.get_child_optional( "targetContainerLocation" );
        if( !child ){
            targetContainerLocation="";
        }else{
            targetContainerLocation = pt2.get<std::string>("targetContainerLocation");
        }

        child = pt2.get_child_optional( "sourceContainerNumber" );
        if( !child ){
            sourceContainerNumber="";
        }else{
            sourceContainerNumber = pt2.get<std::string>("sourceContainerNumber");
        }

        child = pt2.get_child_optional( "status" );
        if( !child ){
            status="";
        }else{
            status = pt2.get<std::string>("status");
        }

        child = pt2.get_child_optional( "time" );
        if( !child ){
            time="";
        }else{
            time = pt2.get<std::string>("time");
        }
    }
};

#endif //COBOTOS_JOBINFOMATION_H
