#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <unordered_map>

// Marker 类型枚举
enum MarkerType {
    SPHERE,
    CUBE,
    CYLINDER
};

// 共享的属性对象
class MarkerProperties {
public:
    std_msgs::ColorRGBA color;
    // 其他共享属性

    MarkerProperties(const std_msgs::ColorRGBA& color) : color(color) {}
};

// Marker 抽象基类
class Marker {
protected:
    MarkerProperties* properties;

public:
    Marker(MarkerProperties* properties) : properties(properties) {}
    virtual visualization_msgs::Marker createMarker() const = 0;
};

// SphereMarker 类
class SphereMarker : public Marker {
public:
    SphereMarker(MarkerProperties* properties) : Marker(properties) {}

    visualization_msgs::Marker createMarker() const override {
        visualization_msgs::Marker marker;
        // 使用共享的属性设置 sphere marker 的属性
        marker.color = properties->color;
        // ...
        return marker;
    }
};

// CubeMarker 类
class CubeMarker : public Marker {
public:
    CubeMarker(MarkerProperties* properties) : Marker(properties) {}

    visualization_msgs::Marker createMarker() const override {
        visualization_msgs::Marker marker;
        // 使用共享的属性设置 cube marker 的属性
        marker.color = properties->color;
        // ...
        return marker;
    }
};

// CylinderMarker 类
class CylinderMarker : public Marker {
public:
    CylinderMarker(MarkerProperties* properties) : Marker(properties) {}

    visualization_msgs::Marker createMarker() const override {
        visualization_msgs::Marker marker;
        // 使用共享的属性设置 cylinder marker 的属性
        marker.color = properties->color;
        // ...
        return marker;
    }
};

// Marker 工厂类
class MarkerFactory {
private:
    static std::unordered_map<MarkerType, MarkerProperties*> propertiesMap;

public:
    static void initialize() {
        propertiesMap[SPHERE] = new MarkerProperties({1.0, 0.0, 0.0, 1.0});  // 红色
        propertiesMap[CUBE] = new MarkerProperties({0.0, 1.0, 0.0, 1.0});   // 绿色
        propertiesMap[CYLINDER] = new MarkerProperties({0.0, 0.0, 1.0, 1.0});  // 蓝色
    }

    static Marker* createMarker(MarkerType type) {
        MarkerProperties* properties = propertiesMap[type];
        switch (type) {
            case SPHERE:
                return new SphereMarker(properties);
            case CUBE:
                return new CubeMarker(properties);
            case CYLINDER:
                return new CylinderMarker(properties);
            default:
                return nullptr;
        }
    }
};

std::unordered_map<MarkerType, MarkerProperties*> MarkerFactory::propertiesMap;

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle nh;

    // 创建一个 ROS Publisher 发布 Marker
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker_topic", 10);

    // 初始化 MarkerFactory
    MarkerFactory::initialize();

    // 创建不同类型的 Marker
    Marker* sphereMarker = MarkerFactory::createMarker(SPHERE);
    Marker* cubeMarker = MarkerFactory::createMarker(CUBE);
    Marker* cylinderMarker = MarkerFactory::createMarker(CYLINDER);

    // 发布 Marker
    marker_pub.publish(sphereMarker->createMarker());
    marker_pub.publish(cubeMarker->createMarker());
    marker_pub.publish(cylinderMarker->createMarker());

    // 释放内存
    delete sphereMarker;
    delete cubeMarker;
    delete cylinderMarker;

    // 释放属性对象内存
    for (auto& pair : MarkerFactory::propertiesMap) {
        delete pair.second;
    }

    // 循环等待退出
    ros::spin();

    return 0;
}
