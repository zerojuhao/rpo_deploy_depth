#pragma once

#include <sys/mman.h>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <queue>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp> 
#include "utils/motion_loader.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "robot_interface.hpp"

enum class ObsStackOrder {
    FrameMajor,
    ObsMajor,
};

class InferenceNode;

struct ObsSourceDefinition {
    const char* name;
    void (InferenceNode::*get)(std::vector<float>& segment);
};

struct ObsSourceSpec {
    std::string name;
    const ObsSourceDefinition* source;
    int size;
};

class InferenceNode : public rclcpp::Node {
   public:
    struct ModelContext {
        std::unique_ptr<Ort::Session> session;
        std::unique_ptr<Ort::MemoryInfo> memory_info;
        std::unique_ptr<Ort::Value> input_tensor;
        std::unique_ptr<Ort::Value> output_tensor;
        std::vector<std::string> input_names;
        std::vector<std::string> output_names;
        std::vector<const char *> input_names_raw;
        std::vector<const char *> output_names_raw;
        std::vector<int64_t> input_shape;
        std::vector<int64_t> output_shape;
        std::vector<float> input_buffer;
        std::vector<float> output_buffer;
        size_t num_inputs;
        size_t num_outputs;
    };

    struct PolicyRuntime {
        std::string name;
        std::string model_path;
        std::string motion_path;
        std::vector<ObsSourceSpec> obs_layout;
        std::vector<int> obs_layout_sizes;
        std::vector<std::vector<float>> obs_segments;
        std::vector<float> obs;
        std::vector<ObsSourceSpec> extra_obs_layout;
        std::vector<std::vector<float>> extra_obs_segments;
        int obs_num = 0;
        int extra_obs_num = 0;
        int frame_stack = 1;
        ObsStackOrder stack_order = ObsStackOrder::FrameMajor;
        std::unique_ptr<ModelContext> ctx;
        std::shared_ptr<MotionLoader> motion_loader;
        size_t motion_frame = 0;
        bool is_first_frame = true;
    };

    InferenceNode() : Node("inference_node") {
        load_config();

        robot_ = std::make_shared<RobotInterface>(std::string(ROOT_DIR) + "config/robot.yaml");

        Ort::ThreadingOptions thread_opts;
        if (intra_threads_ > 0) {
            thread_opts.SetGlobalIntraOpNumThreads(intra_threads_);
        }
        env_ = std::make_unique<Ort::Env>(thread_opts, ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeInference");
        if (policies_.empty()) {
            throw std::runtime_error("At least one policy must be configured");
        }
        for (size_t i = 0; i < policies_.size(); i++) {
            PolicyRuntime& policy = policies_[i];
            policy.obs.resize(policy.obs_num, 0.0f);
            policy.obs_segments.resize(policy.obs_layout.size());
            for (size_t j = 0; j < policy.obs_layout.size(); j++) {
                policy.obs_segments[j].resize(policy.obs_layout[j].size, 0.0f);
            }
            policy.extra_obs_segments.resize(policy.extra_obs_layout.size());
            for (size_t j = 0; j < policy.extra_obs_layout.size(); j++) {
                policy.extra_obs_segments[j].resize(policy.extra_obs_layout[j].size, 0.0f);
            }
            if (!policy.motion_path.empty()) {
                policy.motion_loader = std::make_shared<MotionLoader>(policy.motion_path);
                if (policy.motion_loader->get_num_frames() == 0) {
                    throw std::runtime_error("Motion file has no frames: " + policy.motion_path);
                }
                if (policy.motion_loader->get_num_joints() != static_cast<size_t>(joint_num_)) {
                    throw std::runtime_error("Motion joint count mismatch: " + policy.motion_path);
                }
            }
            setup_model(policy.ctx, policy.model_path,
                        policy.obs_num * policy.frame_stack + policy.extra_obs_num);
        }
        initialize_runtime_state();
        reset_runtime_state();

        auto data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", data_qos, std::bind(&InferenceNode::subs_joy_callback, this, std::placeholders::_1));
        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", data_qos, std::bind(&InferenceNode::subs_cmd_callback,this, std::placeholders::_1
        ));
        elevation_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            use_depth_ ? depth_obs_topic_ : perception_obs_topic_, data_qos,
            std::bind(&InferenceNode::subs_elevation_callback, this, std::placeholders::_1));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_ref_states", data_qos,
            std::bind(&InferenceNode::subs_joint_state_callback, this, std::placeholders::_1));
        action_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/action", data_qos);
        imu_publisher_ =
            this->create_publisher<sensor_msgs::msg::Imu>("/imu", data_qos);
        joint_state_publisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", data_qos);
        inference_thread_ = std::thread(&InferenceNode::inference, this);
        control_thread_ = std::thread(&InferenceNode::control, this);

        reset_joints_service_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_joints", std::bind(&InferenceNode::reset_joints_srv, this, std::placeholders::_1, std::placeholders::_2));
        set_zeros_service_ = this->create_service<std_srvs::srv::Trigger>(
            "set_zeros", std::bind(&InferenceNode::set_zeros_srv, this, std::placeholders::_1, std::placeholders::_2));
        clear_errors_service_ = this->create_service<std_srvs::srv::Trigger>(
            "clear_errors", std::bind(&InferenceNode::clear_errors_srv, this, std::placeholders::_1, std::placeholders::_2));
        refresh_joints_service_ = this->create_service<std_srvs::srv::Trigger>(
            "refresh_joints", std::bind(&InferenceNode::refresh_joints_srv, this, std::placeholders::_1, std::placeholders::_2));
        read_joints_service_ = this->create_service<std_srvs::srv::Trigger>(
            "read_joints", std::bind(&InferenceNode::read_joints_srv, this, std::placeholders::_1, std::placeholders::_2));
        read_imu_service_ = this->create_service<std_srvs::srv::Trigger>(
            "read_imu", std::bind(&InferenceNode::read_imu_srv, this, std::placeholders::_1, std::placeholders::_2));
        init_motors_service_ = this->create_service<std_srvs::srv::Trigger>(
            "init_motors", std::bind(&InferenceNode::init_motors_srv, this, std::placeholders::_1, std::placeholders::_2));
        deinit_motors_service_ = this->create_service<std_srvs::srv::Trigger>(
            "deinit_motors", std::bind(&InferenceNode::deinit_motors_srv, this, std::placeholders::_1, std::placeholders::_2));
        start_inference_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_inference", std::bind(&InferenceNode::start_inference_srv, this, std::placeholders::_1, std::placeholders::_2));
        stop_inference_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_inference", std::bind(&InferenceNode::stop_inference_srv, this, std::placeholders::_1, std::placeholders::_2));
    }
    ~InferenceNode() {
        if (inference_thread_.joinable()) {
            inference_thread_.join();
        }
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        reset_runtime_state();
        if(robot_){
            robot_.reset();
        }
    }
    bool supports_interrupt() const;
    bool has_motion_policy() const;
   private:
    std::shared_ptr<RobotInterface> robot_;
    std::atomic<bool> is_running_{false}, is_joy_control_{true}, is_interrupt_{false}, is_motion_policy_{false};
    std::string perception_obs_topic_, depth_obs_topic_;
    size_t current_motion_policy_idx_ = 0;
    int active_policy_idx_ = 0;
    int perception_obs_num_, joint_num_;
    bool use_depth_ = false;
    int decimation_;
    std::unique_ptr<Ort::Env> env_;
    int intra_threads_;
    Ort::AllocatorWithDefaultOptions allocator_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr elevation_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr action_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    std::thread inference_thread_;
    std::thread control_thread_;
    float act_alpha_;
    float dt_;
    float obs_scales_lin_vel_, obs_scales_ang_vel_, obs_scales_dof_pos_, obs_scales_dof_vel_,
        obs_scales_gravity_b_, clip_observations_;
    float action_scale_, clip_actions_;
    std::vector<double> clip_cmd_, joint_default_angle_, joint_limits_;
    std::vector<long int> usd2urdf_;
    float gravity_z_upper_;
    int last_button0_ = 0, last_button1_ = 0, last_button2_ = 0, last_button3_ = 0, last_button4_ = 0, last_button5_ = 0;
    std::vector<PolicyRuntime> policies_;
    std::vector<int> motion_policy_indices_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_joints_service_, set_zeros_service_, clear_errors_service_, refresh_joints_service_, read_joints_service_, read_imu_service_, init_motors_service_, deinit_motors_service_, start_inference_service_, stop_inference_service_;

    std::mutex act_mutex_, perception_mutex_, interrupt_mutex_, cmd_mutex_, mode_mutex_, lb_switch_mutex_;
    std::vector<float> act_, last_act_, cmd_vel_, interrupt_action_, perception_obs_buffer_;
    std::vector<float> joint_pos_buffer_, joint_vel_buffer_, joint_torques_buffer_, quat_buffer_, ang_vel_buffer_;
    sensor_msgs::msg::JointState joint_state_msg_, action_msg_;

    void subs_joy_callback(const std::shared_ptr<sensor_msgs::msg::Joy> msg);
    void subs_cmd_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
    void subs_elevation_callback(const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg);
    void subs_joint_state_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void inference();
    void control();
    void apply_action();
    PolicyRuntime& active_policy();
    const PolicyRuntime& active_policy() const;

    void load_config();
    void setup_model(std::unique_ptr<ModelContext>& ctx, std::string model_path, int input_size);

    // Policy/model runtime helpers.
    void initialize_runtime_state();
    void reset_runtime_state();
    void reset_policy_runtime(PolicyRuntime& policy);
    void step_motion_frame();

    // Observation registry and layout helpers.
    static const std::vector<ObsSourceDefinition>& obs_source_definitions();
    std::vector<ObsSourceSpec> parse_obs_layout(const std::string& layout_spec,
                                                const std::string& layout_name);
    bool has_obs_source(const std::string& source_name) const;
    ObsStackOrder parse_obs_stack_order(const std::string& stack_order_name);

    // Observation runtime helpers.
    void update_obs_segments(std::vector<std::vector<float>>& segments,
                             const std::vector<ObsSourceSpec>& layout);
    void flatten_obs_segments(const std::vector<std::vector<float>>& segments,
                              std::vector<float>::iterator output_begin);
    void update_stacked_obs(std::vector<float>& input_buffer, const std::vector<float>& obs,
                            int obs_num, int frame_stack, ObsStackOrder stack_order,
                            const std::vector<int>& field_sizes, bool is_first_frame);

    // Observation getters.
    void get_cmd_vel_obs(std::vector<float>& segment);
    void get_ang_vel_obs(std::vector<float>& segment);
    void get_gravity_b_obs(std::vector<float>& segment);
    void get_dof_pos_obs(std::vector<float>& segment);
    void get_dof_vel_obs(std::vector<float>& segment);
    void get_last_action_obs(std::vector<float>& segment);
    void get_interrupt_obs(std::vector<float>& segment);
    void get_perception_obs(std::vector<float>& segment);
    void get_motion_pos_obs(std::vector<float>& segment);
    void get_motion_vel_obs(std::vector<float>& segment);

    void init_motors_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void deinit_motors_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reset_joints_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void set_zeros_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void clear_errors_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void refresh_joints_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void read_joints_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void read_imu_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void start_inference_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stop_inference_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void publish_joint_states();
    void publish_action();
    void publish_imu();
    
    template <typename T>
    void print_vector(const std::string& name, const std::vector<T>& vec) {
        std::stringstream ss;
        ss << name << ": [";
        for (size_t i = 0; i < vec.size(); ++i) {
            ss << vec[i] << (i == vec.size() - 1 ? "" : ", ");
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
};
