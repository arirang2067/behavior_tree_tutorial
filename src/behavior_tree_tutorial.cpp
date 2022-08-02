#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

BT::BehaviorTreeFactory factory;

class Move : public BT::AsyncActionNode
{
public:
  Move(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config){};
  static BT::PortsList providedPorts();
  BT::NodeStatus tick();
  void halt() override
  {
    _halt_requested.store(true);
  }

private:
  std::atomic_bool _halt_requested;
};

BT::PortsList Move::providedPorts()
{
  return {BT::InputPort<double>("move_command")};
}
BT::NodeStatus Move::tick()
{
  _halt_requested.store(false);
  double cmd;
  getInput<double>("move_command", cmd);
  ROS_INFO_STREAM("[Move] : " << cmd);
  return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

void RegisterNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<Move>("Move");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "behavior_tree_tutorial_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  std::string setting_path;
  setting_path = ros::package::getPath("behavior_tree_tutorial") + "/config/setting.yaml";
  YAML::Node doc;
  doc = YAML::LoadFile(setting_path.c_str());

  std::string tree_name;
  tree_name = doc["tree_name"].as<std::string>();

  std::string tree_path;
  tree_path = ros::package::getPath("behavior_tree_tutorial") + "/config/" + tree_name;
  RegisterNodes(factory);
  auto tree = factory.createTreeFromFile(tree_path);



  while(ros::ok())
  {
    tree.tickRoot();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
