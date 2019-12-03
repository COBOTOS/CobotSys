//
// Created by cobot on 19-4-29.
//


//#include <DetectronProxy.h>
//#include "../../../../../../Eagle/Eagle.Vision.Plugin/DeepLearning/src/DetectronProxy.h"
#include <memory>

int main(int argc, char** argv)
{
    using namespace caid::DetectronProxy;
    std::shared_ptr<DetectronProxy> model = std::make_share<DetectronProxy>("maskrcnn_tube");

}