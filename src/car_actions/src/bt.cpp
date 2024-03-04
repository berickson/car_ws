#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class Task1 : public BT::SyncActionNode
{
  public:
    Task1(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "Task1: " << this->name() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

class Task2 : public BT::SyncActionNode
{
  public:
    Task2(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "Task2: " << this->name() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

class Task3 : public BT::SyncActionNode
{
  public:
    Task3(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "Task3: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class Task4 : public BT::SyncActionNode
{
  public:
    Task4(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "Task4: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

static const char* xml_text_medium = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Fallback name="root">
            <Task1 name="task_1"/>
            <Task2 name="task_2"/>
            <Sequence>
                <Task3 name="task_3"/>
                <Task4  name="task_4"/>
            </Sequence>
        </Fallback>
     </BehaviorTree>

 </root>
 )";

int main()
{
    BehaviorTreeFactory factory;

    factory.registerNodeType<Task1>("Task1");
    factory.registerNodeType<Task2>("Task2");
    factory.registerNodeType<Task3>("Task3");
    factory.registerNodeType<Task4>("Task4");

    std::cout << "\n------------ BUILDING A NEW TREE ------------" << std::endl;

    auto tree = factory.createTreeFromText(xml_text_medium);

    tree.tickRoot();

    std::cout << std::endl;

    // }

    return 0;
}