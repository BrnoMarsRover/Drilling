/******************************************************************************
 * @file     drill_controller.h
 * @author  Martin Kriz
 * @brief   Header file of node drill_controller
 * @date    2025-04-28
 *****************************************************************************/

#ifndef DRILL_CONTROLLER_H

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "drill_logger.h"
#include "drill_status.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "drill_interfaces/action/drill_calibration.hpp"
#include "drill_interfaces/action/drill_sample.hpp"
#include "drill_interfaces/action/store_sample.hpp"
#include "drill_interfaces/srv/get_sample_weight.hpp"
#include "drill_interfaces/srv/get_drill_status.hpp"
#include "drill_interfaces/srv/drill_reset.hpp"
#include "atomic"
#include "cmath"

// Hardcoded constants
namespace drill_constants {

    // [Hz] Main loop rate
    constexpr int LOOP_RATE = 5;

    // Maximum number of storage slots
    constexpr int MAX_SLOT = 4;

    // Default slot used when drilling is possible
    constexpr int DEF_SLOT = 0;

    // [mm] Safe height for rotating with the storage
    constexpr int SAFE_POS = 10;

    // [mm] Height when the drill reaches the storage
    constexpr int STORING_POS = 50;

    // [s] Time for dumping the material from the tube
    constexpr int DUMPING_TIME = 10;

    // [mm] Maximum height of the linear actuator
    constexpr int MAX_HEIGHT = 500;

}

enum class state_machine
{
    stop,
    drilling,
    goto_height,
    turn_right,
    turn_left,
    slot_select,
    tare_scale,
    get_weight,
    reset_weight,
    reset_subsystems,
    reset_pico
};

class DrillController : public rclcpp::Node
{
public:
    using DrillSample = drill_interfaces::action::DrillSample;
    using StoreSample = drill_interfaces::action::StoreSample;
    using DrillCalibration = drill_interfaces::action::DrillCalibration;
    using GetSampleWeight = drill_interfaces::srv::GetSampleWeight;
    using GetDrillStatus = drill_interfaces::srv::GetDrillStatus;
    using DrillReset = drill_interfaces::srv::DrillReset;

    using GoalHandleDrillSample = rclcpp_action::ServerGoalHandle<DrillSample>;
    using GoalHandleStoreSample = rclcpp_action::ServerGoalHandle<StoreSample>;
    using GoalHandleDrillCalibration = rclcpp_action::ServerGoalHandle<DrillCalibration>;

    // Constructor
    DrillController();

    // Destructor
    ~DrillController() override = default;

private:
    std::shared_ptr<DrillLogger> DrillLogger_;
    std::shared_ptr<DrillStatus> DrillStatus_;

    //Controller variables
    std::atomic<bool> drill_is_busy{};
    std::atomic<float> motorTorque{};
    std::atomic<float> motorRPS{};
    std::atomic<int> motorTemperature{};
    std::atomic<int> drillHeight{};
    std::atomic<int> drillDepth{};
    std::atomic<int> toGround{};
    std::atomic<int> activeSlot{};

    std::atomic<float> sampleWeight1{};
    std::atomic<float> sampleWeight2{};
    std::atomic<float> sampleWeight3{};
    std::atomic<float> sampleWeight4{};

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
    rclcpp::Service<GetDrillStatus>::SharedPtr get_drill_status_srv_;
    rclcpp::Service<DrillReset>::SharedPtr drill_reset_srv_;

    // DRILL SAMPLE ACTION
    // If the drill is busy service will be refused
    rclcpp_action::GoalResponse handle_goal_drill_sample(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DrillSample::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received DrillSample request to drill %d mm with maximal rps %f .", goal->depth, goal->max_rps);
        (void)uuid;

        if (drill_is_busy || !DrillStatus_->is_drill_connected()) {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or not connected. Rejecting request.");
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

        if (drill_is_busy || !DrillStatus_->is_drill_connected()) {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or not connected. Rejecting request.");
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
    void execute_store_sample(std::shared_ptr<GoalHandleStoreSample> goal_handle);

    // DRILL CALIBRATION ACTION
    // If the drill is busy action will be refused
    rclcpp_action::GoalResponse handle_goal_drill_calibration(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DrillCalibration::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received DrillCalibration request. Weights will be: %s", goal->reset_weights ? "erased" : "kept");
        (void)uuid;

        if (drill_is_busy || !DrillStatus_->is_drill_connected()) {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or not connected. Rejecting request.");
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
        response->weight = get_sampleWeight(request->slot);
    }

    void get_drill_status_callback(const std::shared_ptr<drill_interfaces::srv::GetDrillStatus::Request> request,
        std::shared_ptr<drill_interfaces::srv::GetDrillStatus::Response> response);

    void drill_reset_callback(const std::shared_ptr<drill_interfaces::srv::DrillReset::Request> request,
        std::shared_ptr<drill_interfaces::srv::DrillReset::Response> response);

    // Publishes the current drill state to the "drill_state" topic
    // @param state: The current state of the drill, represented by the state_machine enum
    void publish_drill_state(state_machine state) const;

    // Callback function that handles incoming drill data
    // @param msg: Shared pointer to the received UInt16MultiArray message
    void drill_data_callback(std_msgs::msg::UInt16MultiArray::SharedPtr msg);

    // Publishes drilling parameters (rotation speed, target height, storage slot)
    // @param rps: Drill rotation speed in RPS
    // @param height: Target drilling height in millimeters
    // @param slot: Storage slot number
    void publish_drill_param(float rps, int height, int slot) const;


    // Sets the drillDepth variable based on the current height and distance to ground
    // If calculated depth is negative, sets depth to zero
    void set_depth()
    {
        if (const auto tmp = drillHeight - toGround; tmp < 0)
            drillDepth = 0;
        else
            drillDepth = tmp;
    }

    // Calculates absolute height based on target drilling depth
    // @param depth: Desired drilling depth
    // @return: Absolute height from the ground
    int calculate_height(const int depth) const
    {
        return toGround + depth;
    }

    // Returns the sample weight for a given slot
    // @param slot: Storage slot number
    // @return: Sample weight in grams
    float get_sampleWeight(int slot);

    // Checks if the drill hits the target
    // @param goalHeight: Target height
    // @return: True if within tolerance, false otherwise
    bool height_inTolerance(const int goalHeight) const
    {
        if (goalHeight == drillHeight)
            return true;
        return false;
    }

    // Converts a linear speed percentage to a control code (0–255)
    // @param speed: Speed percentage (0–100%)
    // @return: Encoded speed as uint16_t
    static uint16_t speed_code(const int speed)
    {
        //formula for linear speed will be
        if (auto tmp = abs(speed) * (255 / 100); tmp < 256)
            return tmp;
        return 255;
    }

    // Decodes a sensor value to a float using a scale factor of 0.03
    // @param aNum: Raw sensor value
    // @return: Decoded float value
    static float float_decode(const uint16_t aNum) {
        const float result =  0.03f * static_cast<float>(aNum);
        return result;
    }

    // Encodes a float value into a scaled uint16_t value
    // @param aNum: Float value to encode
    // @return: Encoded uint16_t value
    static uint16_t float_code(const float aNum)
    {
        const auto result =  static_cast<uint16_t>(aNum / 0.03f);
        return result;
    }
};

#define DRILL_CONTROLLER_H

#endif //DRILL_CONTROLLER_H
