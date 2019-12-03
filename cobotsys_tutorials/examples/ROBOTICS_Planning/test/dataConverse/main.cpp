//
// Created by cobot on 19-11-11.
//

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <fstream>
#include <logger/Logger.h>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

using namespace rw;
using namespace rw::loaders;

int main(int argc, char** argv){
    //input&output QPath file
    std::string filenameInput = "../src/input.txt";
    std::string filenameOutput = "../src/output.txt";

    QFile inputFile(filenameInput.c_str());
    if(!inputFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        LOG_ERROR << "open fail!";
        return 0;
    }
    QTextStream ss(&inputFile);
    QStringList cmdList= ss.readAll().split("\n");
    rw::trajectory::TimedQPath timedQPath;
//    LOG_INFO << cmdList.size();
    for(auto cmd:cmdList){
//        LOG_INFO << cmd.toStdString();
        QStringList dat=cmd.split(",");
//        LOG_INFO << dat.size();
        if(dat.size() != 8) { continue;}
        double time = dat.at(0).toDouble();
        rw::math::Q q(7,0.0);
        for(int jj=0;jj<7;jj++){
//            q[jj]=((QString)dat.at(jj+1)).toDouble();
            q[jj]=dat.at(jj+1).toDouble();
//            LOG_INFO << dat.at(jj).toDouble();
        }
        rw::trajectory::TimedQ timedQ(time,q);
        timedQPath.push_back(timedQ);
    }
    auto traj = rw::trajectory::TrajectoryFactory::makeLinearTrajectory(timedQPath);
    auto path = traj->getPath(0.001);
    std::ofstream out(filenameOutput);
    for(rw::math::Q& q:path){
//        LOG_INFO << path.size();
//        LOG_INFO << q.size();
        for(int ii=0;ii<q.size();ii++){
            if(ii!=q.size()-1){
                out <<q[ii]<<",";
            }else{
                out <<(q[ii]>0)<< "\n";
            }
        }
    }
    out.close();
}
