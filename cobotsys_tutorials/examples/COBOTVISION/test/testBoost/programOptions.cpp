//
// Created by lijun on 19-4-7.
//
#include<iostream>
#include <boost/program_options.hpp>
using namespace std;
using namespace boost::program_options;


int test(int argc, char **argv)
{
    try {
        options_description desc("Allowed options");
        desc.add_options()
                ("help,h", "produce help message")
                ("compression", value<int>(), "set compression level")
                ("file,D",value< vector<std::string> >()->multitoken(), "search dir")
                ("depth,d",value<int>()->implicit_value(5),"search depth");

        variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);

        if (vm.count("help")) {
            cout << desc << "\n";
            return 1;
        }

        if (vm.count("compression")) {
            cout << "Compression level was set to " << vm["compression"].as<int>() << ".\n";
        } else {
            cout << "Compression level was not set.\n";
        }
    }catch (...){
        std::cout << "输入的参数中存在未定义的选项！\n";
        return 0;
    }

    return 0;
}