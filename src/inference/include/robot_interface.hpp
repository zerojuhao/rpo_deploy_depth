#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <memory>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <queue>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "utils/close_chain_mapping.hpp"
#include "utils/thread_pool.hpp"
#include "motor_driver.hpp"
#include "imu_driver.hpp"

class RobotInterface {
   public:
    RobotInterface(const std::string& config_file);
    ~RobotInterface() {
        deinit_motors();
        motors_.clear();
        imu_.reset();
    }
    struct IMUCfg{
        int imu_id_, baudrate_;
        std::string imu_type_, imu_interface_type_, imu_interface_;
    };
    struct MotorsCfg{
        int master_id_offset_;
        std::string motor_type_, motor_interface_type_;
        std::vector<std::string> motor_interface_;
        std::vector<long int> motor_id_, motor_model_, motor_num_;
        std::vector<double> motor_zero_offset_;
    };
    struct RobotCfg{
        std::vector<long int> close_chain_motor_id_, motor_sign_, urdf2motor_;
        std::vector<double> kp_, kd_, extrinsic_R_;
    };

    void apply_action(std::vector<float> action);
    void init_motors();
    void deinit_motors();
    void reset_joints(std::vector<double> joint_default_angle);
    void set_zeros();
    void clear_errors();
    void refresh_joints();
    std::vector<float> get_joint_q() {
        if (!is_init_.load()) {
            throw std::runtime_error("Motors not initialized");
        }
        std::unique_lock<std::mutex> lock(joint_mutex_);
        return joint_q_;
    }
    std::vector<float> get_joint_vel() {
        if (!is_init_.load()) {
            throw std::runtime_error("Motors not initialized");
        }
        std::unique_lock<std::mutex> lock(joint_mutex_);
        return joint_vel_;
    }
    std::vector<float> get_joint_tau() {
        if (!is_init_.load()) {
            throw std::runtime_error("Motors not initialized");
        }
        std::unique_lock<std::mutex> lock(joint_mutex_);
        return joint_tau_;
    }
    const std::vector<float>& get_quat() {
        if (!imu_) {
            throw std::runtime_error("IMU not initialized");
        }
        auto raw = imu_->get_quat();  // w, x, y, z
        q_body_ = Eigen::Quaternionf(raw[0], raw[1], raw[2], raw[3]) * extrinsic_q_inv_;
        q_body_.normalize();
        quat_buf_[0] = q_body_.w();
        quat_buf_[1] = q_body_.x();
        quat_buf_[2] = q_body_.y(); 
        quat_buf_[3] = q_body_.z();
        return quat_buf_;
    }
    const std::vector<float>& get_ang_vel() {
        if (!imu_) {
            throw std::runtime_error("IMU not initialized");
        }
        auto raw = imu_->get_ang_vel();  // in IMU frame
        Eigen::Map<const Eigen::Vector3f> omega_imu(raw.data());
        Eigen::Map<Eigen::Vector3f>(ang_vel_buf_.data()) = extrinsic_R_mat_ * omega_imu;
        return ang_vel_buf_;
    }

    std::atomic<bool> is_init_{false};

   private:
    std::shared_ptr<IMUCfg> imu_cfg_;
    std::shared_ptr<MotorsCfg> motors_cfg_;
    std::shared_ptr<RobotCfg> robot_cfg_;
    int offline_threshold_ = 25;
    std::shared_ptr<IMUDriver> imu_;
    std::shared_ptr<Decouple> ankle_decouple_;
    Eigen::Matrix3f extrinsic_R_mat_ = Eigen::Matrix3f::Identity();
    Eigen::Quaternionf extrinsic_q_inv_ = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf q_body_;
    std::vector<float> quat_buf_{0.f, 0.f, 0.f, 0.f};
    std::vector<float> ang_vel_buf_{0.f, 0.f, 0.f};
    std::vector<std::shared_ptr<MotorDriver>> motors_;
    std::unique_ptr<ThreadPool> thread_pool_;

    std::mutex motors_mutex_, joint_mutex_;
    std::vector<float> joint_q_, joint_vel_, joint_tau_, motor_target_;
    std::vector<int> close_chain_motor_idx_, close_chain_joint_idx_, motor2urdf_;

    void setup_motors();
    void setup_imu();

    void exec_motors_parallel(const std::function<void(std::shared_ptr<MotorDriver>&, int)>& cmd_func);
};