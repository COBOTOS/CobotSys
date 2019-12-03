#include "dh_mer_camera.h"
#include <logger/Logger.h>

using namespace caid;

int main() {
    CameraMer cameraMer;
    bool ret = cameraMer.open("NT0190010020");
    LOG_INFO << "open status " << ret;
    cameraMer.grab(100000);
}