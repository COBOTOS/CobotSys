//
// Created by lijun on 19-4-3.
//

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

bool test()
{
    boost::filesystem::path p("home/Downloads");
    std::cout << p.string() << std::endl;

    std::cout << p.root_name() << std::endl;
    std::cout << p.root_directory() << std::endl;
    std::cout << p.root_path() << std::endl;
    std::cout << p.relative_path() << std::endl;
    std::cout << p.parent_path() << std::endl;
    std::cout << p.filename() << std::endl;

    boost::filesystem::path p1("photo.jpg");
    std::cout << p1.stem() << std::endl;
    std::cout << p1.extension() << std::endl;

    for (boost::filesystem::path::iterator it = p.begin(); it != p.end(); ++it)
        std::cout << *it << std::endl;


    boost::interprocess::shared_memory_object::remove("Highscore");
    boost::interprocess::managed_shared_memory managed_shm(boost::interprocess::open_or_create, "Highscore", 1024);
    int *i = managed_shm.construct<int>("Integer")(99);
    std::cout << *i << std::endl;
    managed_shm.destroy<int>("Integer");
    std::pair<int*, std::size_t> p2 = managed_shm.find<int>("Integer");
    if (p2.first)
    {
        std::cout << *p2.first << std::endl;
        std::cout << p2.second << std::endl;
    }

}