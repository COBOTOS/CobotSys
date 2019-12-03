#include <boost/python.hpp>
#include "AddPython.h"

using namespace std;
using namespace boost::python;

char const * greet()
{
    return "hello,worldddd";
}

//the code it generated
#if 0
void init_module_hello_ext();
extern "C" __attribute__((__visibility__("default"))) PyObject* PyInit_hello_ext()
{
    static PyModuleDef_Base initial_m_base = { { 1, __null }, 0, 0, 0 };
    static PyMethodDef initial_methods[] = { { 0, 0, 0, 0 } };
    static struct PyModuleDef moduledef = { initial_m_base, "hello_ext", 0, -1, initial_methods, 0, 0, 0, 0, };
    return boost::python::detail::init_module( moduledef, init_module_hello_ext );
}

void init_module_hello_ext()
{
    def("greet", greet);
}

#else
BOOST_PYTHON_MODULE(hello_ext)
{
    def("greet", greet);
}
#endif

#if 1
#if 1
void init_module_AddPython();
extern "C" __attribute__((__visibility__("default"))) PyObject* PyInit_AddPython() {
    static PyModuleDef_Base initial_m_base = { { 1, __null }, 0, 0, 0 };
    static PyMethodDef initial_methods[] = { { 0, 0, 0, 0 } };
    static struct PyModuleDef moduledef = { initial_m_base, "AddPython", 0, -1, initial_methods, 0, 0, 0, 0, };
    return boost::python::detail::init_module( moduledef, init_module_AddPython );
}

void init_module_AddPython()
{
    using namespace boost::python;

    class_<AddPython>("AddPython", init<>())
            .def(init<int>())
            .def("get", &AddPython::get)
            .def("set", &AddPython::set)
            .def_readwrite("publicVal", &AddPython::publicVal)
            ;
    def("printA", &printA);
    def("addA", &addA);
}
#else
BOOST_PYTHON_MODULE(AddPython)
{
    using namespace boost::python;
    // 导出类
    class_<AddPython>("AddPython", init<>())                            //如果默认构造函数没有参数，可以省略
    .def(init<int>())                               //其他构造函数
    .def("get", &AddPython::get)                            //成员函数
    .def("set", &AddPython::set)                            //成员函数
    .def_readwrite("publicVal", &AddPython::publicVal)      //数据成员，当然是公共的
    ;
    def("printA", &printA);
    def("addA", &addA);
}
#endif
#endif