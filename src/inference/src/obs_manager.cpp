#include "inference_node.hpp"

namespace {
std::string trim_copy(const std::string& value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c) != 0; });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

ObsSourceSpec make_source_spec(const std::string& name, const ObsSourceDefinition& source, int size) {
    ObsSourceSpec spec;
    spec.name = name;
    spec.source = &source;
    spec.size = size;
    return spec;
}

std::vector<std::string> split_obs_layout_spec(const std::string& layout_spec) {
    std::vector<std::string> layout_specs;
    size_t start = 0;
    while (start < layout_spec.size()) {
        const size_t end = layout_spec.find(',', start);
        const std::string token = trim_copy(layout_spec.substr(start, end == std::string::npos ? std::string::npos : end - start));
        if (!token.empty()) {
            layout_specs.push_back(token);
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return layout_specs;
}

}

const std::vector<ObsSourceDefinition>& InferenceNode::obs_source_definitions() {
    static const std::vector<ObsSourceDefinition> definitions{
        {"motion_pos", &InferenceNode::get_motion_pos_obs},
        {"motion_vel", &InferenceNode::get_motion_vel_obs},
        {"ang_vel", &InferenceNode::get_ang_vel_obs},
        {"gravity_b", &InferenceNode::get_gravity_b_obs},
        {"cmd_vel", &InferenceNode::get_cmd_vel_obs},
        {"dof_pos", &InferenceNode::get_dof_pos_obs},
        {"dof_vel", &InferenceNode::get_dof_vel_obs},
        {"last_action", &InferenceNode::get_last_action_obs},
        {"interrupt", &InferenceNode::get_interrupt_obs},
        {"perception", &InferenceNode::get_perception_obs},
    };
    return definitions;
}

std::vector<ObsSourceSpec> InferenceNode::parse_obs_layout(
    const std::string& layout_spec,
    const std::string& layout_name) {
    const std::vector<std::string> layout_specs = split_obs_layout_spec(layout_spec);
    if (layout_specs.empty()) {
        throw std::runtime_error(layout_name + " must be explicitly configured");
    }

    std::vector<ObsSourceSpec> layout;
    layout.reserve(layout_specs.size());
    for (const std::string& raw_spec : layout_specs) {
        const std::string spec = trim_copy(raw_spec);
        const size_t separator = spec.find(':');
        if (separator == std::string::npos || separator == 0 || separator == spec.size() - 1) {
            throw std::runtime_error(layout_name + " entry must use 'name:size' format: " + raw_spec);
        }

        const std::string name = trim_copy(spec.substr(0, separator));
        const std::string size_text = trim_copy(spec.substr(separator + 1));
        if (name.empty() || size_text.empty()) {
            throw std::runtime_error(layout_name + " entry must use 'name:size' format: " + raw_spec);
        }
        if (!std::all_of(size_text.begin(), size_text.end(), [](unsigned char c) { return std::isdigit(c) != 0; })) {
            throw std::runtime_error(layout_name + " field size must be a positive integer: " + raw_spec);
        }

        const auto& definitions = obs_source_definitions();
        const auto source = std::find_if(definitions.begin(), definitions.end(), [&name](const ObsSourceDefinition& definition) {
            return name == definition.name;
        });
        if (source == definitions.end()) {
            throw std::runtime_error("Unsupported obs source: " + name);
        }

        layout.push_back(make_source_spec(name, *source, std::stoi(size_text)));
    }
    return layout;
}

bool InferenceNode::has_obs_source(const std::string& source_name) const {
    return std::any_of(policies_.begin(), policies_.end(), [this, &source_name](const PolicyRuntime& policy) {
        const auto source_matches = [&source_name](const ObsSourceSpec& spec) {
            return spec.name == source_name;
        };
        return std::any_of(policy.obs_layout.begin(), policy.obs_layout.end(), source_matches) ||
               std::any_of(policy.extra_obs_layout.begin(), policy.extra_obs_layout.end(), source_matches);
    });
}

void InferenceNode::update_obs_segments(std::vector<std::vector<float>>& segments, const std::vector<ObsSourceSpec>& layout) {
    for (size_t i = 0; i < layout.size(); i++) {
        (this->*(layout[i].source->get))(segments[i]);
    }
}

void InferenceNode::flatten_obs_segments(const std::vector<std::vector<float>>& segments,
                                         std::vector<float>::iterator output_begin) {
    int offset = 0;
    for (size_t i = 0; i < segments.size(); i++) {
        std::copy(segments[i].begin(), segments[i].end(), output_begin + offset);
        offset += static_cast<int>(segments[i].size());
    }
}

void InferenceNode::step_motion_frame() {
    auto& policy = active_policy();
    if (!policy.motion_loader) {
        return;
    }
    policy.motion_frame += 1;
    if (policy.motion_frame >= policy.motion_loader->get_num_frames()) {
        policy.motion_frame = policy.motion_loader->get_num_frames() - 1;
    }
}

void InferenceNode::get_motion_pos_obs(std::vector<float>& segment) {
    auto& policy = active_policy();
    const std::vector<float>& motion_pos = policy.motion_loader->get_pos(policy.motion_frame);
    std::copy(motion_pos.begin(), motion_pos.end(), segment.begin());
}

void InferenceNode::get_motion_vel_obs(std::vector<float>& segment) {
    auto& policy = active_policy();
    const std::vector<float>& motion_vel = policy.motion_loader->get_vel(policy.motion_frame);
    std::copy(motion_vel.begin(), motion_vel.end(), segment.begin());
}

void InferenceNode::get_ang_vel_obs(std::vector<float>& segment) {
    ang_vel_buffer_ = robot_->get_ang_vel();
    for (int i = 0; i < 3; i++) {
        segment[i] = ang_vel_buffer_[i] * obs_scales_ang_vel_;
    }
}

void InferenceNode::get_gravity_b_obs(std::vector<float>& segment) {
    quat_buffer_ = robot_->get_quat();
    Eigen::Quaternionf q_b2w(quat_buffer_[0], quat_buffer_[1], quat_buffer_[2], quat_buffer_[3]);
    Eigen::Vector3f gravity_w(0.0f, 0.0f, -1.0f);
    Eigen::Quaternionf q_w2b = q_b2w.inverse();
    Eigen::Vector3f gravity_b = q_w2b * gravity_w;
    if (gravity_b.z() > gravity_z_upper_){
        RCLCPP_FATAL(this->get_logger(), "Robot fell down! Shutting down...");
        rclcpp::shutdown();
        throw std::runtime_error("Robot fell down");
    }
    segment[0] = gravity_b.x() * obs_scales_gravity_b_;
    segment[1] = gravity_b.y() * obs_scales_gravity_b_;
    segment[2] = gravity_b.z() * obs_scales_gravity_b_;
}

void InferenceNode::get_cmd_vel_obs(std::vector<float>& segment) {
    std::unique_lock<std::mutex> lock(cmd_mutex_);
    segment[0] = cmd_vel_[0] * obs_scales_lin_vel_;
    segment[1] = cmd_vel_[1] * obs_scales_lin_vel_;
    segment[2] = cmd_vel_[2] * obs_scales_ang_vel_;
}

void InferenceNode::get_dof_pos_obs(std::vector<float>& segment) {
    joint_pos_buffer_ = robot_->get_joint_q();
    for (int i = 0; i < joint_num_; i++) {
        segment[i] = (joint_pos_buffer_[usd2urdf_[i]] - joint_default_angle_[usd2urdf_[i]]) * obs_scales_dof_pos_;
    }
    for(size_t i = 0; i < joint_limits_.size() / 2; i++){
        if(joint_pos_buffer_[i] < joint_limits_[i * 2] || joint_pos_buffer_[i] > joint_limits_[i * 2 + 1]){
            RCLCPP_FATAL(this->get_logger(), "Joint %zu out of limit! Shutting down...", i+1);
            rclcpp::shutdown();
            throw std::runtime_error("Joint out of limit");
        }
    }
}

void InferenceNode::get_dof_vel_obs(std::vector<float>& segment) {
    joint_vel_buffer_ = robot_->get_joint_vel();
    for (int i = 0; i < joint_num_; i++) {
        segment[i] = joint_vel_buffer_[usd2urdf_[i]] * obs_scales_dof_vel_;
    }
}

void InferenceNode::get_last_action_obs(std::vector<float>& segment) {
    const auto& policy = active_policy();
    for (int i = 0; i < joint_num_; i++) {
        segment[i] = policy.ctx->output_buffer[i];
    }
}

void InferenceNode::get_interrupt_obs(std::vector<float>& segment) {
    segment[0] = is_interrupt_.load() ? 1.0f : 0.0f;
}

void InferenceNode::get_perception_obs(std::vector<float>& segment) {
    std::unique_lock<std::mutex> lock(perception_mutex_);
    std::copy(perception_obs_buffer_.begin(), perception_obs_buffer_.begin() + segment.size(), segment.begin());
}
