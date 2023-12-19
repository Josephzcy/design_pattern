// GlobalObject.h

#ifndef GLOBAL_OBJECT_H
#define GLOBAL_OBJECT_H

class GlobalObject {
public:
    // ...
};

// 声明全局对象的外部链接
extern GlobalObject g_GlobalObject;

#endif

// way2
// GlobalObject.cpp

#include "GlobalObject.h"

// 初始化全局对象
GlobalObject g_GlobalObject;

// ...
// OtherFile.cpp

#include "GlobalObject.h"

void someFunction() {
    // 在这个函数中无法直接访问 g_GlobalObject
    // ...
}


/*
在 OtherFile.cpp 中，我们包含了 GlobalObject.h 头文件，但我们无法直接访问 g_GlobalObject。这是因为 g_GlobalObject 具有内部链接，只能在 GlobalObject.cpp 文件中访问。

这种方法适用于那些只在单个源文件中使用的全局对象。通过将其定义限制在一个源文件中，可以避免全局对象的重复定义问题，并确保全局对象的作用域受限于当前源文件。

需要注意的是，如果需要在多个源文件中访问全局对象，那么这种方法就不适用了。在这种情况下，应该使用第一种方法，将全局对象的定义放在一个头文件中，并在需要使用它的源文件中包含该头文件
*/
