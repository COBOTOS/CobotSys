#include <stdio.h>
#include "AddPython.h"

int addA(AddPython &a, int addVal)
{
    int val = a.get();
    val += addVal;
    a.set(val);
    return val;
}
void printA(const AddPython& a)
{
    printf("%d\n", a.get());
}