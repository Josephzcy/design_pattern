#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

// 基类 Platform
class Platform {
public:
    virtual ~Platform() {}
    virtual std::string getName() const = 0;
    virtual void execute() const = 0;
};

// 派生类 WindowsPlatform
class WindowsPlatform : public Platform {
public:
    std::string getName() const override {
        return "Windows";
    }

    void execute() const override {
        std::cout << "Running on Windows platform." << std::endl;
    }
};

// 派生类 LinuxPlatform
class LinuxPlatform : public Platform {
public:
    std::string getName() const override {
        return "Linux";
    }

    void execute() const override {
        std::cout << "Running on Linux platform." << std::endl;
    }
};

// 平台工厂类 PlatformFactory
class PlatformFactory {
public:
    static std::shared_ptr<Platform> createPlatform(const std::string& name) {
        auto it = platformRegistry().find(name);
        if (it != platformRegistry().end()) {
            return it->second();
        } else {
            return nullptr;
        }
    }

    static void registerPlatform(const std::string& name, std::function<std::shared_ptr<Platform>()> createFunc) {
        platformRegistry()[name] = createFunc;
    }

private:
    static std::unordered_map<std::string, std::function<std::shared_ptr<Platform>()>>& platformRegistry() {
        static std::unordered_map<std::string, std::function<std::shared_ptr<Platform>()>> registry;
        return registry;
    }
};
/*
在上述示例中，我们使用了一个静态成员函数 platformRegistry() 来维护平台类型和创建函数的注册表。platformRegistry() 返回一个对应平台名称和创建函数的 std::unordered_map。

然后，我们定义了一个静态成员函数 registerPlatform()，用于将平台名称和创建函数注册到注册表中。在 registerPlatform() 函数中，我们使用 std::function 来保存创建函数，并将其与平台名称关联起来。

在全局作用域中，我们使用静态变量和匿名函数来进行动态注册。通过在静态变量初始化期间调用匿名函数，我们可以在程序启动时自动注册平台类型。

在 main() 函数中，我们使用 PlatformFactory::createPlatform() 来创建不同平台的对象，并使用智能指针进行管理。

通过使用动态注册机制，我们可以在运行时动态注册和创建不同平台的对象，而无需修改工厂类的代码。这种方法提供了更大的灵活性和可扩展性，可以方便地添加新的平台类型
*/
// 注册 WindowsPlatform
static bool windowsPlatformRegistered = []{
    PlatformFactory::registerPlatform("Windows", [](){
        return std::make_shared<WindowsPlatform>();
    });
    return true;
}();

// 注册 LinuxPlatform
static bool linuxPlatformRegistered = []{
    PlatformFactory::registerPlatform("Linux", [](){
        return std::make_shared<LinuxPlatform>();
    });
    return true;
}();

int main() {
    // 创建 WindowsPlatform 对象
    std::shared_ptr<Platform> windowsPlatform = PlatformFactory::createPlatform("Windows");
    if (windowsPlatform) {
        std::cout << "Platform: " << windowsPlatform->getName() << std::endl;
        windowsPlatform->execute();
    }

    // 创建 LinuxPlatform 对象
    std::shared_ptr<Platform> linuxPlatform = PlatformFactory::createPlatform("Linux");
    if (linuxPlatform) {
        std::cout << "Platform: " << linuxPlatform->getName() << std::endl;
        linuxPlatform->execute();
    }

    return 0;
}
