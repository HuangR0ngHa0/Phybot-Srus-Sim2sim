#include "StateMachine/include/statemacine.h"

#include "StateMachine/include/Zero.h"
#include "StateMachine/include/RL_walk.h"

class StateMachineManager {
public:

    // bool IfWalkStart;
    // bool IfGaitEnd;
    // bool IfStand2Walk;
    // bool IfCanStop;
    // State CurrentState{State::STANDING};
    // State NextState{State::STANDING};
    State CurrentState{State::ZERO};
    State NextState{State::ZERO};
    
    // 构造函数，自动注册所有状态机处理器
    StateMachineManager() {
        // 创建并注册事件处理器

        auto ZeroHandler = std::make_shared<Zero>();
        auto RL_walk_Handler = std::make_shared<RL_walk>();


        registerHandler(State::ZERO, ZeroHandler);
        registerHandler(State::RL_walk, RL_walk_Handler);

    }

    // 注册事件处理器
    void registerHandler(State state, std::shared_ptr<StateMachine> handler) {
        handlers[state] = handler;
    }

    void handleEvent(State state, Event event, DataPackage& data) {
        auto handler = handlers.find(state);
        if (handler != handlers.end()) {
            if (event == Event::START) {
                handler->second->start(event, data);
            } else if (event == Event::RUN) {
                handler->second->run(event, data);
            } else if (event == Event::EXIT) {
                handler->second->exit(event, data);
            } 
        } else {
            std::cout << "No handler for state: " << static_cast<int>(state) << std::endl;
        }
    }

    void init(DataPackage& data) {

        handleEvent(State::ZERO, Event::START, data);  // 自动触发 start 事件
        handleEvent(State::RL_walk, Event::START, data);  // 自动触发 start 事件

    }

    void GetDataFromPackage(DataPackage &DataPackage);
    void SetDataToPackage(DataPackage &DataPackage);
    void run(DataPackage &DataPackage);

private:
    // 状态到事件处理器的映射
    std::unordered_map<State, std::shared_ptr<StateMachine>> handlers;
    std::unordered_map<State, State> stateTransitionMap;    
};
