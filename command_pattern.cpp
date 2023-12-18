#include <iostream>
#include <fstream>
#include <map>
#include <memory>

// 命令接口
class Command {
public:
    virtual void execute() = 0;
};

// 具体命令类
class ConcreteCommand1 : public Command {
public:
    void execute() override {
        std::cout << "Executing ConcreteCommand1" << std::endl;
    }
};

class ConcreteCommand2 : public Command {
public:
    void execute() override {
        std::cout << "Executing ConcreteCommand2" << std::endl;
    }
};

// 命令工厂类
class CommandFactory {
public:
    using CommandCreator = std::unique_ptr<Command> (*)();

    static std::unique_ptr<Command> createCommand(const std::string& commandName) {
        auto it = commandCreators.find(commandName);
        if (it != commandCreators.end()) {
            return it->second();
        } else {
            return nullptr; // 未知命令
        }
    }

    static void registerCommand(const std::string& commandName, CommandCreator creator) {
        commandCreators[commandName] = creator;
    }

private:
    static std::map<std::string, CommandCreator> commandCreators;
};

std::map<std::string, CommandFactory::CommandCreator> CommandFactory::commandCreators;

// 注册具体命令类到工厂
void registerCommandsToFactory() {
    CommandFactory::registerCommand("Command1", []() { return std::make_unique<ConcreteCommand1>(); });
    CommandFactory::registerCommand("Command2", []() { return std::make_unique<ConcreteCommand2>(); });
}

// 读取配置文件，创建并执行命令
void executeCommandsFromConfig(const std::string& configFilePath) {
    std::ifstream configFile(configFilePath);
    if (!configFile) {
        std::cerr << "Failed to open config file: " << configFilePath << std::endl;
        return;
    }

    std::map<std::string, std::unique_ptr<Command>> commands;

    std::string line;
    while (std::getline(configFile, line)) {
        if (!line.empty()) {
            std::unique_ptr<Command> command = CommandFactory::createCommand(line);
            if (command) {
                commands[line] = std::move(command);
            } else {
                std::cerr << "Unknown command: " << line << std::endl;
            }
        }
    }

    configFile.close();

    // 执行命令
    for (const auto& pair : commands) {
        std::cout << "Executing command: " << pair.first << std::endl;
        pair.second->execute();
    }
}

int main() {
    registerCommandsToFactory();

    std::string configFilePath = "commands.txt";
    executeCommandsFromConfig(configFilePath);

    return 0;
}
