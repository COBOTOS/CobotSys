//
// Created by zhoupeng on 19-6-13.
//
#include <logger/Logger.h>
#include <boost/make_shared.hpp>
#include <vector>

class OneObject{
public:
    OneObject() {
        LOG_INFO << "OneObject invoked";
    }

    ~OneObject() {
        LOG_INFO << "~OneObject invoked";
    }
};

void testCreateObject(std::vector<boost::shared_ptr<OneObject>>& objects, int count) {
    for (int i = 0; i < count ; i++) {
        objects.push_back(boost::make_shared<OneObject>());
        //objects[i]=(boost::make_shared<OneObject>());
    }
}

void createManyObjects(int count) {
    std::vector <boost::shared_ptr<OneObject>> objects;
    //objects.reserve(count);
    testCreateObject(objects, count);
    //objects.clear();
}

int main(int argc, char** argv) {
    createManyObjects(30);
    LOG_INFO << "finish invoke createManyObjects";
}
