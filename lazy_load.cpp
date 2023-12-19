#include <ros/ros.h>

//如果你在一个全局静态类中使用ROS代码，并且需要调用ros::init()进行ROS系统的初始化，你可以使用延迟初始化（lazy initialization）的方法来解决ros::init()没有初始化的问题。

//延迟初始化是指将初始化操作推迟到第一次需要使用该对象时执行。在这种情况下，你可以将ros::init()的调用放在静态成员函数中，并使用静态成员变量来追踪是否已经进行了初始化。这样，在第一次调用该静态成员函数时，会执行初始化操作，而后续调用则会直接使用已经初始化的ROS系统。

//下面是一个示例，展示了如何在全局静态类中进行延迟初始化：

// lazy 用的时候采取初始化某些东西，不在类初始化的时候去定义
// 用其他类套一层
class MyStaticClass {
public:
    static void doSomething() {
        if (!initialized) {
            ros::init(ros::M_string(), "my_node");  // Initialize ROS system
            initialized = true;
        }

        // ROS logic...
    }

private:
    static bool initialized;
};

bool MyStaticClass::initialized = false;

int main(int argc, char** argv) {
    MyStaticClass::doSomething();  // First call initializes ROS system
    // ...

    return 0;
}
