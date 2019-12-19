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
    std::string filenameOutput = "../src/output.txt";

    std::ofstream out(filenameOutput);
    rw::math::Q initQ(6,0.0);
    rw::math::Q q(6,0.0);
        for(double t = 0.0;t < 5;t += 0.008){
            for(int mm = 0;mm < 6;mm++){
                q[mm] = initQ[mm] + 0.5 * sin(0.1 * 2 *M_PI * t);
            }
            for(int ii = 0;ii < q.size();ii++){
                if(ii!=q.size()-1){
                    out << q[ii] << ",";
                }else{
                    out << q[ii] << "\n";
                }
            }
        }
    out.close();
}
