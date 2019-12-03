#include "JobInfomation.h"
#include <iostream>
int main(){


    JobInfomation jobInfomation;
    jobInfomation.finishQuantity=100;
    jobInfomation.nodeNumber="12";
    jobInfomation.orderNumber="123123";
    jobInfomation.pickQuantity=200;
    jobInfomation.productionNumber="productionNumber";
    jobInfomation.sourceContainerNumber="sourceContainerNumber";
    jobInfomation.status="status";
    jobInfomation.targetContainerLocation="targetContainerLocation";
    jobInfomation.time = "time";
    std::string json = jobInfomation.toString();

    JobInfomation jobInfomation1(json);

    std::cout<<jobInfomation1.toString()<<std::endl;


    ptree pt ,children;
    children.push_back(std::make_pair("", jobInfomation.toPtree()));
    children.push_back(std::make_pair("", jobInfomation.toPtree()));
    children.push_back(std::make_pair("", jobInfomation.toPtree()));

    pt.add_child("JOBS", children);

    std::ostringstream buf;
    write_json (buf, pt, false);
     json = buf.str();

    std::cout<<json<<std::endl;


    ptree pt2;
    std::istringstream is (json);
    read_json (is, pt2);

    auto child = pt2.get_child_optional( "JOBS" );
    ptree jobs;
    if( child ){
        jobs = pt2.get_child("JOBS");

        for(auto job=jobs.begin();job!=jobs.end();job++){
//            std::ostringstream buf;
//            write_json (buf, job->second, false);
//            json = buf.str();
//
//            std::cout<< json<<std::endl;
            JobInfomation infomation(job->second);
            std::cout<< infomation.toString()<<std::endl;
        }

    }
}