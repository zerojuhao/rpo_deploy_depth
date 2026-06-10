#include "evo_motor_driver.hpp"

EVO_Limit_Param evo_limit_param[EVO_Num_Of_Model] = {
    {12.5, 20.0, 18.0, 500.0, 5.0},     // EVO431040
    {12.5, 10.0, 50.0, 250.0, 50.0},    // EVO811825
    {12.5, 10.0, 50.0, 250.0, 50.0},    // EVO811832
};

EvoMotorDriver::EvoMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                               EVO_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), can_(MotorsSocketCAN::get(can_interface)), motor_model_(motor_model) {
    if (interface_type != "can") {
        throw std::runtime_error("EVO driver only support CAN interface");
    }
    motor_id_ = motor_id;
    limit_param_ = evo_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;
    CanCbkFunc can_callback = std::bind(&EvoMotorDriver::can_rx_cbk, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, motor_id_);
}

EvoMotorDriver::~EvoMotorDriver() { can_->remove_can_callback(motor_id_); }

void EvoMotorDriver::lock_motor() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFC;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::unlock_motor() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFD;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

uint8_t EvoMotorDriver::init_motor() {
    // send disable command to enter read mode
    EvoMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    // send enable command to enter contorl mode
    EvoMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    switch (error_id_) {
        case EVOError::EVO_OVER_VOLTAGE:
            return EVOError::EVO_OVER_VOLTAGE;
            break;
        case EVOError::EVO_UNDER_VOLTAGE:
            return EVOError::EVO_UNDER_VOLTAGE;
            break;
        case EVOError::EVO_OVER_CURRENT:
            return EVOError::EVO_OVER_CURRENT;
            break;
        case EVOError::EVO_MOS_OVER_TEMP:
            return EVOError::EVO_MOS_OVER_TEMP;
            break;
        case EVOError::EVO_COIL_OVER_TEMP:
            return EVOError::EVO_COIL_OVER_TEMP;
            break;
        case EVOError::EVO_COMM_LOST:
            return EVOError::EVO_COMM_LOST;
            break;
        case EVOError::EVO_OVERLOAD:
            return EVOError::EVO_OVERLOAD;
            break;
        case EVOError::EVO_ENCODER_ERROR:
            return EVOError::EVO_ENCODER_ERROR;
            break;
        default:
            return error_id_;
    }
    return error_id_;
}

void EvoMotorDriver::deinit_motor() {
    EvoMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool EvoMotorDriver::write_motor_flash() { return true; }

bool EvoMotorDriver::set_motor_zero() {
    // send set zero command
    EvoMotorDriver::set_motor_zero_evo();
    Timer::sleep_for(setup_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);  // wait for motor to set zero
    logger_->info("motor_id: {0}\tposition: {1}\t", motor_id_, get_motor_pos());
    EvoMotorDriver::unlock_motor();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
    // disable motor
}

void EvoMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    
    pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
    spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
    t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
    error_id_ = rx_frame.data[6];
    if (error_id_ > 0) {
            if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, (uint32_t)error_id_);
        }
    }
    mos_temperature_ = rx_frame.data[7];
    
    motor_pos_ = range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), 
                          -limit_param_.PosMax, limit_param_.PosMax) + motor_zero_offset_;
    motor_spd_ = range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), 
                          -limit_param_.SpdMax, limit_param_.SpdMax);
    
    motor_current_ = range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), 
                                -limit_param_.TauMax, limit_param_.TauMax);
}

void EvoMotorDriver::get_motor_param(uint8_t param_cmd) {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;
    
    tx_frame.data[0] = 0x67;
    tx_frame.data[1] = param_cmd;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x00;
    tx_frame.data[4] = 0x00;
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x04;
    tx_frame.data[7] = 0x76;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void EvoMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;
    can_frame tx_frame;

    f_p -= motor_zero_offset_;
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);
    
    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(12));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = p >> 8;
    tx_frame.data[1] = p & 0xFF;
    tx_frame.data[2] = v >> 4;
    tx_frame.data[3] = (v & 0x0F) << 4 | kp >> 8;
    tx_frame.data[4] = kp & 0xFF;
    tx_frame.data[5] = kd >> 4;
    tx_frame.data[6] = (kd & 0x0F) << 4 | t >> 8;
    tx_frame.data[7] = t & 0xFF;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    write_register_evo(11, 0x02);
    motor_control_mode_ = motor_control_mode;
}

void EvoMotorDriver::set_motor_zero_evo() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFE;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::clear_motor_error_evo() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFD;
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::write_register_evo(uint8_t index, int32_t value) {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;
    
    tx_frame.data[0] = 0x67;
    tx_frame.data[1] = index;
    tx_frame.data[2] = *vbuf;
    tx_frame.data[3] = *(vbuf + 1);
    tx_frame.data[4] = *(vbuf + 2);
    tx_frame.data[5] = *(vbuf + 3);
    tx_frame.data[6] = 0x15;
    tx_frame.data[7] = 0x76;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::save_register_evo() {
    can_frame tx_frame;
    tx_frame.can_id = 0x600 + motor_id_;
    tx_frame.can_dlc = 0x08;
    
    tx_frame.data[0] = 0x67;
    tx_frame.data[1] = 0x00;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x00;
    tx_frame.data[4] = 0x00;
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x00;
    tx_frame.data[7] = 0x76;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::refresh_motor_status() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFC;
    
    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EvoMotorDriver::clear_motor_error() {
    clear_motor_error_evo();
}