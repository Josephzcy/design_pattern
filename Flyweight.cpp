#include <iostream>
#include <unordered_map>
#include <string>

// 享元对象接口
class Flyweight {
public:
    virtual void operation(const std::string& extrinsicState) = 0;
};

// 具体享元对象
class ConcreteFlyweight : public Flyweight {
public:
    void operation(const std::string& extrinsicState) override {
        std::cout << "ConcreteFlyweight: " << extrinsicState << std::endl;
    }
};

// 享元工厂
class FlyweightFactory {
private:
    std::unordered_map<std::string, Flyweight*> flyweights;

public:
    Flyweight* getFlyweight(const std::string& key) {
        if (flyweights.find(key) == flyweights.end()) {
            flyweights[key] = new ConcreteFlyweight();
        }
        return flyweights[key];
    }
};

int main() {
    FlyweightFactory factory;

    // 使用享元对象
    Flyweight* flyweight1 = factory.getFlyweight("key1");
    flyweight1->operation("Extrinsic State 1");

    Flyweight* flyweight2 = factory.getFlyweight("key2");
    flyweight2->operation("Extrinsic State 2");

    Flyweight* flyweight3 = factory.getFlyweight("key1");
    flyweight3->operation("Extrinsic State 3");

    // 输出：ConcreteFlyweight: Extrinsic State 1
    // 输出：ConcreteFlyweight: Extrinsic State 2
    // 输出：ConcreteFlyweight: Extrinsic State 3

    return 0;
}
