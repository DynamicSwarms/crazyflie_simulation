#include <cstdio>
#include <mutex>
#include <map>
#include <sys/wait.h>    
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>

#include "rclcpp_components/node_factory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "class_loader/class_loader.hpp"
#include <filesystem>

#include "crazyflie_interfaces/srv/add_crazyflie.hpp"
#include "crazyflie_interfaces/srv/remove_crazyflie.hpp"

#include "crazyflie_simulation_gateway/crazyflie_lifecycle_client.hpp"

#include "signal.h"

using namespace std::chrono_literals;

std::atomic_bool sigint_received(false);
std::atomic_bool gateway_shutdown_done(false);

class GatewayException : public std::runtime_error
{
public:
  explicit GatewayException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

struct DedicatedExecutorWrapper
{
  std::shared_ptr<rclcpp::Executor> executor;
  std::thread thread;
  std::atomic_bool thread_initialized;

  /// Constructor for the wrapper.
  /// This is necessary as atomic variables don't have copy/move operators
  /// implemented so this structure is not copyable/movable by default
  explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> exec)
  : executor(exec),
    thread_initialized(false)
  {
  }
};


class Gateway : public rclcpp::Node
{
public:
    Gateway()
    : Node("crazyflie_simulation_gateway")
    {
      m_lifecycle_client_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


      auto service_qos = rmw_qos_profile_services_default;
      service_qos.depth = 100; // This way it is possible to queue up multiple add requestst
      m_gateway_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      m_add_crazyflie_service = this->create_service<crazyflie_interfaces::srv::AddCrazyflie>(
        "~/add_crazyflie",
        std::bind(&Gateway::add_crazyflie_callback, this, std::placeholders::_1, std::placeholders::_2),
        service_qos,
        m_gateway_callback_group);
      
      m_remove_crazyflie_service = this->create_service<crazyflie_interfaces::srv::RemoveCrazyflie>(
        "~/remove_crazyflie",
        std::bind(&Gateway::remove_crazyflie_callback, this, std::placeholders::_1, std::placeholders::_2),
        service_qos,
        m_gateway_callback_group);
    
    
      m_check_crazyflie_processes_timer = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&Gateway::check_crazyflie_processes, this),
        m_gateway_callback_group);

      m_factory = create_component_factory("crazyflie_simulation", "Crazyflie");

      RCLCPP_INFO(this->get_logger(), "CrazyflieSimulationGateway node initialized.");
    }

    ~Gateway(){}

    void check_crazyflie_processes()
    {
      std::lock_guard<std::mutex> lock(m_crazyflies_mutex);

      static bool shutdown_completed = false;
      if (sigint_received.load() && !shutdown_completed)
      {
         shutdown_completed = true;
         if (m_crazyflies.empty()) gateway_shutdown_done.store(true);
         else RCLCPP_INFO(this->get_logger(), "Shutting down all crazyflies due to SIGINT.");
         for (auto &pair : m_crazyflies)
         {
            RCLCPP_INFO(this->get_logger(), "Shutting down crazyflie with id %d.", pair.first);
            auto cf_client = std::get<2>(pair.second);
            cf_client->shutdown_crazyflie_async();
         }
      }
    }

    void add_crazyflie_callback(
      const std::shared_ptr<crazyflie_interfaces::srv::AddCrazyflie::Request> request,
      std::shared_ptr<crazyflie_interfaces::srv::AddCrazyflie::Response> response)
    {
      int id;
      try {
        id = uri_to_id(request->uri);
      } catch (const GatewayException & e) {
          response->success = false;
          response->msg = e.what();
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Adding crazyflie with id: %d.", id);

      std::lock_guard<std::mutex> lock(m_crazyflies_mutex);

      if (m_crazyflies.count(id))
      {
          RCLCPP_WARN(this->get_logger(), "Crazyflie with id %d already exists.", id);
          response->success = false;
          response->msg = "Crazyflie with this ID already exists";
          return;
      }

      auto options = create_node_options(id, request->initial_pose);

      try {
        auto node = m_factory->create_node_instance(options);
        auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        exec->add_node(node.get_node_base_interface());

        std::shared_ptr<CrazyflieLifecycleClient> cf_lifecycle_client = 
          std::make_shared<CrazyflieLifecycleClient>(
            this->get_node_topics_interface(),
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_services_interface(),
            m_lifecycle_client_callback_group,
            id,
            std::bind(&Gateway::on_crazyflie_shutdown, this, std::placeholders::_1));

        auto entry = m_crazyflies.emplace(id, std::make_tuple(node, exec, cf_lifecycle_client));

        DedicatedExecutorWrapper & wrapper = std::get<1>(entry.first->second);
        wrapper.executor = exec;

        auto & thread_initialized = wrapper.thread_initialized;
        wrapper.thread = std::thread(
          [exec, &thread_initialized ]() {
            thread_initialized = true;
            try {
              exec->spin();
            } catch (...) {}
          }
        );

      } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create crazyflie node: %s", e.what());
            response->success = false;
            response->msg = "Failed to create crazyflie node";
            return;
      }

      response->success = true;
    }

    void remove_crazyflie_callback(
      const std::shared_ptr<crazyflie_interfaces::srv::RemoveCrazyflie::Request> request,
      std::shared_ptr<crazyflie_interfaces::srv::RemoveCrazyflie::Response> response)
    {
      int id; 
      try {
        id = uri_to_id(request->uri);
      } catch (const GatewayException & e) {
          response->success = false;
          response->msg = e.what();
          return;
      }
      
      RCLCPP_INFO(this->get_logger(), "Remove crazyflie service called for id: %d.", id);

      std::lock_guard<std::mutex> lock(m_crazyflies_mutex);

      auto cf_entry = m_crazyflies.find(id);
      if (cf_entry != m_crazyflies.end())
      {
          if (!std::get<1>(cf_entry->second).thread_initialized) rclcpp::sleep_for(std::chrono::milliseconds(1)); // Race condition of add and remove
          std::get<2>(cf_entry->second)->shutdown_crazyflie_sync(100ms);
          std::get<1>(cf_entry->second).executor->cancel();
          std::get<1>(cf_entry->second).thread.join();
          m_crazyflies.erase(cf_entry);
          response->success = true;
          response->msg = "Crazyflie removed successfully";

          RCLCPP_INFO(this->get_logger(), "Crazyflie with id %d removed.", id);
      } else {
          response->success = false;
          RCLCPP_WARN(this->get_logger(), "Crazyflie with id %d does not exist.", id);
      }
    }

    void on_crazyflie_shutdown(int id)
    {
      std::lock_guard<std::mutex> lock(m_crazyflies_mutex);

      auto it = m_crazyflies.find(id);
      if (it != m_crazyflies.end())
      {
          RCLCPP_INFO(this->get_logger(), "Detected shutdown on crazyflie with id %d.", id);
          m_crazyflies.erase(it);
      }

      if (m_crazyflies.empty() && sigint_received.load())
      {
          RCLCPP_INFO(this->get_logger(), "All crazyflie processes have shut down. Proceeding with gateway shutdown.");
          gateway_shutdown_done.store(true);
      }      
    }

private: 
    int 
    uri_to_id(const std::string & uri)
    {
      const std::string prefix = "sim://";
      if (uri.rfind(prefix, 0) != 0) {
          throw GatewayException("URI must start with sim://");
      }

      try {
          return std::stoi(uri.substr(prefix.length()));
      } catch (...) {
          throw GatewayException("ID must be an integer");
      }
    }

    rclcpp::NodeOptions
    create_node_options(int id, 
                        const geometry_msgs::msg::Pose & initial_pose)
    {  
      std::vector<std::string> remap_rules;
  
      remap_rules.push_back("--ros-args");
      remap_rules.push_back("-r");
      remap_rules.push_back("__node:=cf" + std::to_string(id));
  
      auto add_parameter = [&](const std::string &name, const std::string &value) {
        remap_rules.push_back("-p");
        remap_rules.push_back(name + ":=" + value);
      };
  
      add_parameter("id", std::to_string(id));
  
      std::ostringstream pos_stream;
      pos_stream << std::fixed << std::setprecision(1) << "[" 
                << initial_pose.position.x << "," 
                << initial_pose.position.y << "," 
                << initial_pose.position.z << "]";
      
      add_parameter("initial_position", pos_stream.str());
  
      auto options = rclcpp::NodeOptions()
        .arguments(remap_rules);
        return options;
    }

    std::vector<std::pair<std::string, std::string>>
    get_component_resources(
      const std::string & package_name, const std::string & resource_index) const
    {
      std::string content;
      std::string base_path;
      if (
        !ament_index_cpp::get_resource(
          resource_index, package_name, content, &base_path))
      {
        throw GatewayException("Could not find requested resource in ament index");
      }

      std::vector<std::pair<std::string, std::string>> resources;
      std::vector<std::string> lines = rcpputils::split(content, '\n', true);
      for (const auto & line : lines) {
        std::vector<std::string> parts = rcpputils::split(line, ';');
        if (parts.size() != 2) {
          throw GatewayException("Invalid resource entry");
        }

        std::filesystem::path library_path = parts[1];
        if (!library_path.is_absolute()) {
          library_path = (base_path / library_path);
        }
        resources.push_back({parts[0], library_path.string()});
      }
      return resources;
    }


    std::shared_ptr<rclcpp_components::NodeFactory>
    create_component_factory(const std::string & package_name, const std::string & class_name)
    {
      auto resources = get_component_resources(package_name, "rclcpp_components");
      
      std::string library_path = resources[0].second;
      std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

      class_loader::ClassLoader * loader;
      RCLCPP_DEBUG(get_logger(), "Load Library: %s", library_path.c_str());
      try {
        m_loader = std::make_unique<class_loader::ClassLoader>(library_path);
      } catch (const std::exception & ex) {
        throw GatewayException("Failed to load library: " + std::string(ex.what()));
      } catch (...) {
        throw GatewayException("Failed to load library");
      }
    
      loader = m_loader.get();

      auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
      for (const auto & clazz : classes) {
        RCLCPP_DEBUG(get_logger(), "Found class: %s", clazz.c_str());
        if (clazz == class_name || clazz == fq_class_name) {
          RCLCPP_DEBUG(get_logger(), "Instantiate class: %s", clazz.c_str());
          return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
        }
      }
      return {};
    }


private: 
    std::shared_ptr<rclcpp::CallbackGroup> m_lifecycle_client_callback_group;

    std::shared_ptr<rclcpp::CallbackGroup> m_shutdown_detector_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_shutdown_detector_timer;

    std::shared_ptr<rclcpp::CallbackGroup> m_gateway_callback_group;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::AddCrazyflie>> m_add_crazyflie_service;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::RemoveCrazyflie>> m_remove_crazyflie_service; 
    
    std::shared_ptr<rclcpp::TimerBase> m_check_crazyflie_processes_timer;

    std::unique_ptr<class_loader::ClassLoader> m_loader;
    std::shared_ptr<rclcpp_components::NodeFactory> m_factory;

    std::mutex m_crazyflies_mutex;
    std::map<int, std::tuple<rclcpp_components::NodeInstanceWrapper, DedicatedExecutorWrapper, std::shared_ptr<CrazyflieLifecycleClient>>> m_crazyflies;
};



void sigint_handler(int signum)
{
    (void)signum;
    sigint_received.store(true);
    int safey_counter = 0;
    while (!gateway_shutdown_done.load())
    { 
        safey_counter++;
        if (safey_counter > 500) break;// 3 seconds timeout
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (gateway_shutdown_done.load()) std::cerr << "Gateway shut down cleanly after SIGINT." << std::endl;
    else std::cerr << "Gateway shutdown after SIGINT timed out." << std::endl;
}


int main(int argc, char ** argv)
{
  signal(SIGINT, sigint_handler);
  // Install before rclcpp this way rclcpp will store it as a "old" handler and execute it before its own shutdown

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto gateway = std::make_shared<Gateway>();
  
  executor.add_node(gateway);
  executor.spin();
  executor.remove_node(gateway->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
