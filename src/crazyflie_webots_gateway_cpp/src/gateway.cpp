#include <cstdio>
#include <mutex>
#include <map>
#include <sys/wait.h>    
#include "rclcpp/rclcpp.hpp"
#include "crazyflie_webots_gateway_interfaces/srv/webots_crazyflie.hpp"

#include "crazyflie_webots_gateway_cpp/crazyflie_lifecycle_client.hpp"

class Gateway : public rclcpp::Node
{
public:
    Gateway()
    : Node("crazyflie_webots_gateway")
    , p_webots_port(declare_parameter("webots_port", rclcpp::ParameterValue(1234), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , p_webots_use_tcp(declare_parameter("webots_use_tcp", rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<bool>())
    , p_webots_tcp_ip(declare_parameter("webots_tcp_ip", rclcpp::ParameterValue("127.0.0.1"), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>()) 
    {
      m_lifecycle_client_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      m_add_crazyflie_service = this->create_service<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie>(
        "~/add_crazyflie",
        std::bind(&Gateway::add_crazyflie_callback, this, std::placeholders::_1, std::placeholders::_2));
      
      m_remove_crazyflie_service = this->create_service<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie>(
        "~/remove_crazyflie",
        std::bind(&Gateway::remove_crazyflie_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    
      m_check_crazyflie_processes_timer = this->create_wall_timer(std::chrono::milliseconds(500),
        std::bind(&Gateway::check_crazyflie_processes, this));
    }

    ~Gateway()
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      std::cerr << "Shutting down CrazyflieWebotsGateway..." << std::endl;
      // Clean up all crazyflie processes
      for (auto &pair : m_crazyflie_processes)
      {
          bool success = std::get<1>(pair.second)->shutdown_crazyflie_sync(std::chrono::milliseconds(100));
          if (!success)
          {
              std::cerr << "Shutdown of crazyflie " << pair.first << " timed out. Killing process." << std::endl;
              pid_t pid = std::get<0>(pair.second);
              kill(-pid, SIGKILL); //Kill the whole process group
          }
          //pid_t pid = std::get<0>(pair.second);
          //kill(-pid, SIGINT); //Kill the whole process group
      }
      std::cerr << "CrazyflieWebotsGateway shut down cleanly." << std::endl;
    }

    void check_crazyflie_processes()
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      std::vector<int> to_remove;
      for (auto &pair : m_crazyflie_processes)
      {
          int id = pair.first;
          pid_t pid = std::get<0>(pair.second);
          int status;
          pid_t result = waitpid(pid, &status, WNOHANG);
          if (result == 0) continue; // still running
          else if (result == -1) to_remove.push_back(id); // There was an error checking the process
          else to_remove.push_back(id);
      }

      for (int id : to_remove)
      {
          RCLCPP_INFO(this->get_logger(), "Crazyflie process %d has exited. Removing.", id);
          m_crazyflie_processes.erase(id);
      }
    }

    void add_crazyflie_callback(
      const std::shared_ptr<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie::Request> request,
      std::shared_ptr<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie::Response> response)
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      int id = request->id;
      RCLCPP_INFO(this->get_logger(), "Add crazyflie service called for id: %d.", id);
      if (m_crazyflie_processes.find(id) != m_crazyflie_processes.end())
      {
          RCLCPP_WARN(this->get_logger(), "Crazyflie with id %d already exists.", id);
          response->success = false;
          return;
      }

      pid_t pid = fork();
      if (pid == 0) 
      {
          setpgid(0,0); //Create new process group so that we can kill all child processes later       
          std::string node_arg = "__node:=cf" + std::to_string(id);  
          std::string id_arg = "id:=" + std::to_string(id);
          std::string webots_port_arg = "webots_port:=" + std::to_string(p_webots_port);
          std::string webots_use_tcp_arg = p_webots_use_tcp ? "webots_use_tcp:=true" : "webots_use_tcp:=false";
          std::string webots_tcp_ip_arg = "webots_tcp_ip:=" + p_webots_tcp_ip;
          
          std::vector<char*> argv;
          argv.push_back(const_cast<char*>("ros2"));
          argv.push_back(const_cast<char*>("run"));
          argv.push_back(const_cast<char*>("crazyflie_webots_cpp"));
          argv.push_back(const_cast<char*>("crazyflie"));
          argv.push_back(const_cast<char*>("--ros-args"));
          argv.push_back(const_cast<char*>("-p"));
          argv.push_back(const_cast<char*>(id_arg.c_str()));
          argv.push_back(const_cast<char*>("-p"));
          argv.push_back(const_cast<char*>(webots_port_arg.c_str())); 
          argv.push_back(const_cast<char*>("-p"));
          argv.push_back(const_cast<char*>(webots_use_tcp_arg.c_str()));
          argv.push_back(const_cast<char*>("-p"));
          argv.push_back(const_cast<char*>(webots_tcp_ip_arg.c_str()));
          argv.push_back(const_cast<char*>("-r"));
          argv.push_back(const_cast<char*>(node_arg.c_str()));  // safe: node_arg lives
          argv.push_back(nullptr);
  
          execvp("ros2", argv.data());
          _exit(1);
      }

      RCLCPP_INFO(this->get_logger(), "Started crazyflie process with PID %d for id %d.", pid, id);

      std::shared_ptr<CrazyflieLifecycleClient> cf_lifecycle_client = 
        std::make_shared<CrazyflieLifecycleClient>(
          this->get_node_topics_interface(),
          this->get_node_base_interface(),
          this->get_node_graph_interface(),
          this->get_node_services_interface(),
          m_lifecycle_client_callback_group,
          id,
          std::bind(&Gateway::on_crazyflie_shutdown, this, std::placeholders::_1));

      m_crazyflie_processes[id] = std::make_tuple(pid, cf_lifecycle_client);

      response->success = true;
    }

    void remove_crazyflie_callback(
      const std::shared_ptr<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie::Request> request,
      std::shared_ptr<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie::Response> response)
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      int id = request->id;
      RCLCPP_INFO(this->get_logger(), "Remove crazyflie service called for id: %d.", id);

      auto it = m_crazyflie_processes.find(id);
      if (it != m_crazyflie_processes.end())
      {
          bool success = std::get<1>(it->second)->shutdown_crazyflie_sync(std::chrono::milliseconds(100));
          std::cerr << "Shutdown crazyflie " << id << (success ? " succeeded." : " timed out.") << std::endl;
          if (!success)
          {
              RCLCPP_WARN(this->get_logger(), "Shutdown of crazyflie %d timed out.", id);
              pid_t pid = std::get<0>(it->second);
              kill(-pid, SIGTERM); //Kill the whole process group
          }
          m_crazyflie_processes.erase(it);

          RCLCPP_INFO(this->get_logger(), "Crazyflie with id %d removed.", id);

      } else {
          RCLCPP_WARN(this->get_logger(), "Crazyflie with id %d does not exist.", id);
      }

    }

    void on_crazyflie_shutdown(int id)
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      auto it = m_crazyflie_processes.find(id);
      if (it != m_crazyflie_processes.end())
      {
          RCLCPP_INFO(this->get_logger(), "Crazyflie with id %d has shut down.", id);
          //pid_t pid = std::get<0>(it->second);
          //kill(-pid, SIGINT); //Kill the whole process group
          m_crazyflie_processes.erase(it);
      }
      
    }


private: 
    std::shared_ptr<rclcpp::CallbackGroup> m_lifecycle_client_callback_group;
    std::shared_ptr<rclcpp::Service<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie>> m_add_crazyflie_service;
    std::shared_ptr<rclcpp::Service<crazyflie_webots_gateway_interfaces::srv::WebotsCrazyflie>> m_remove_crazyflie_service; 
    
    std::shared_ptr<rclcpp::TimerBase> m_check_crazyflie_processes_timer;

    std::mutex m_crazyflie_processes_mutex;
    std::map<int, std::tuple<pid_t, std::shared_ptr<CrazyflieLifecycleClient>>> m_crazyflie_processes;

private: 
    int p_webots_port;
    bool p_webots_use_tcp;
    std::string p_webots_tcp_ip;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto gateway = std::make_shared<Gateway>();

  executor.add_node(gateway);
  executor.spin();
  executor.remove_node(gateway->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
