//
// Created by martin on 04.03.25.
//

#ifndef DRILL_CONTROLLER_H

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "drill_logger.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "drill_interfaces/action/drill_calibration.hpp"
#include "drill_interfaces/action/drill_sample.hpp"
#include "drill_interfaces/action/store_sample.hpp"
#include "drill_interfaces/srv/get_sample_weight.hpp"

#define MAX_SLOT 4 //Total number of slots
#define DEF_SLOT 0 //Default slot, when the drilling is possible
#define SAFE_POS 20 // [mm] height when the turning with the storage is possible
#define STORING_POS 30 // [mm] height when the drill is the storage
#define MAX_TORQUE 2.5
#define MAX_HEIGHT 500 // [mm]
#define RANGE 1000 // to code the float number

enum state_machine
{
    stop,
    drilling,
    go_down,
    go_up,
    turn_right,
    turn_left,
    slot_select,
    get_weight,
    reset_weight,
    manual
};

class DrillController : public rclcpp::Node {
public:
    using DrillSample = drill_interfaces::action::DrillSample;
    using StoreSample = drill_interfaces::action::StoreSample;
    using DrillCalibration = drill_interfaces::action::DrillCalibration;
    using GetSampleWeight = drill_interfaces::srv::GetSampleWeight;

    using GoalHandleDrillSample = rclcpp_action::ServerGoalHandle<DrillSample>;
    using GoalHandleStoreSample = rclcpp_action::ServerGoalHandle<StoreSample>;
    using GoalHandleDrillCalibration = rclcpp_action::ServerGoalHandle<DrillCalibration>;

    // Constructor
    DrillController();

    // Destructor
    ~DrillController() override = default;

private:
    std::shared_ptr<DrillLogger> DrillLogger_;
    //Controller variables
    bool drill_is_busy;
    float actual_torque;
    int actual_height;
    int active_slot;
    int samples[4];

    // Publishers and subscribers
    // Publisher to publish state for drill
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drill_state_pub_;
    // Publisher to publish parameters for drill
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr drill_params_pub_;
    // Subscriber to subscribe data from drill
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr drill_data_sub_;

    //Action and service servers
    rclcpp_action::Server<DrillSample>::SharedPtr drill_sample_server_;
    rclcpp_action::Server<StoreSample>::SharedPtr store_sample_server_;
    rclcpp_action::Server<DrillCalibration>::SharedPtr drill_calibration_server_;
    rclcpp::Service<GetSampleWeight>::SharedPtr get_sample_weight_srv_;

    // Callback to handle output data from drill
    void drill_data_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received drill data.");
        actual_torque = float_decode(msg->data[0]);
        actual_height = msg->data[1];;
        active_slot = msg->data[2];;
        samples[0] = msg->data[3];
        samples[1] = msg->data[4];
        samples[2] = msg->data[5];
        samples[3] = msg->data[6];
    }

    // DRILL SAMPLE ACTION
    // If the drill is busy service will be refused
    rclcpp_action::GoalResponse handle_goal_drill_sample(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DrillSample::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received DrillSample request to drill %d mm with maximal torque %f .", goal->height, goal->torque);
        (void)uuid;

        if (drill_is_busy) {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or in manual mode. Rejecting request.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Drill is ready. Accepting request.");
        drill_is_busy = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Stop the drill when the action is canceled
    rclcpp_action::CancelResponse handle_cancel_drill_sample(const std::shared_ptr<GoalHandleDrillSample> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "DrillSample canceled.");
        (void)goal_handle;
        drill_is_busy = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Call the action execution
    void handle_accepted_drill_sample(const std::shared_ptr<GoalHandleDrillSample> goal_handle) {
        std::thread{std::bind(&DrillController::execute_drill_sample, this, std::placeholders::_1), goal_handle}.detach();
    }

    //executing the DrillSample service
    void execute_drill_sample(const std::shared_ptr<GoalHandleDrillSample> goal_handle);

    // STORE SAMPLE ACTION
    // If the drill is busy action will be refused
    rclcpp_action::GoalResponse handle_goal_store_sample(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StoreSample::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received StoreSample request to store sample on slot number %d." , goal->slot);
        (void)uuid;

        if (drill_is_busy) {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or in manual mode. Rejecting request.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Drill is ready. Accepting request.");
        drill_is_busy = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Stop the drill when the action is canceled
    rclcpp_action::CancelResponse handle_cancel_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "StoreSample canceled.");
        (void)goal_handle;
        drill_is_busy = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle) {
        std::thread{std::bind(&DrillController::execute_store_sample, this, std::placeholders::_1), goal_handle}.detach();
    }

    //executing the StoreSample action
    void execute_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle);

    // DRILL CALIBRATION ACTION
    // If the drill is busy action will be refused
    rclcpp_action::GoalResponse handle_goal_drill_calibration(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DrillCalibration::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received DrillCalibration request. Weights will be: %s", goal->reset_weights ? "erased" : "kept");
        (void)uuid;

        if (drill_is_busy) {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or in manual mode. Rejecting request.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Drill is ready. Accepting request.");
        drill_is_busy = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Stop the drill when the action is canceled
    rclcpp_action::CancelResponse handle_cancel_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "DrillCalibration canceled.");
        (void)goal_handle;
        drill_is_busy = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle) {
        std::thread{std::bind(&DrillController::execute_drill_calibration, this, std::placeholders::_1), goal_handle}.detach();
    }

    //executing the DrillCalibration service
    void execute_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle);

    void get_sample_weight_callback(const std::shared_ptr<GetSampleWeight::Request> request, std::shared_ptr<GetSampleWeight::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Getting sample weight for slot %d", request->slot);
        response->weight = samples[request->slot];
    }

    // Function for publish state to drill
    void publish_drill_state(const uint8_t state) const{
        auto message = std_msgs::msg::UInt8();
        message.data = state;
        drill_state_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published drill state: %d", state);
    }

    // Function to publish drill parameters
    void publish_drill_param(float torque, uint16_t speed, uint16_t height, uint16_t slot) const;

    static float float_decode(const uint16_t aNum) {
        constexpr  float constant = MAX_TORQUE/RANGE;
        const float result =  constant * static_cast<float>(aNum);
        return result;
    }

    static uint16_t float_code(const float aNum)
    {
        constexpr float constant = RANGE/MAX_TORQUE;
        const auto result =  static_cast<uint16_t>(constant * aNum);
        return result;
    }
};

#define DRILL_CONTROLLER_H

#endif //DRILL_CONTROLLER_H
