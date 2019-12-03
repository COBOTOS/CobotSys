// 定义一个类
class AddPython
{
public:
    AddPython(){privateVal=0;}                      //默认构造函数
    AddPython(int val){privateVal=val;}             //带参数的构造函数
    void set(int val){privateVal=val;}      //成员函数
    int get() const {return privateVal;};   //成员函数
    int publicVal;                          //公共数据成员
private:
    int privateVal;                         //私有数据成员
};

int addA(AddPython &a, int addVal);                   //普通函数，有返回值，通过引用修改参数
void printA(const AddPython& a);   