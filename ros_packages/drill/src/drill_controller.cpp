/******************************************************************************
* @file     drill_controller.cpp
 * @author  Martin Kriz
 * @brief   Drill controller node file
 * @date    2025-04-28
 *****************************************************************************/

#include "drill/drill_controller.h"

// Constructor
DrillController::DrillController(): Node("drill_controller") {

    // Publishers
    drill_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("drill_state", 10);
    drill_params_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("drill_parameters", 10);

    // Subscriber
    drill_data_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "drill_data", 10,
      std::bind(&DrillController::drill_data_callback, this, std::placeholders::_1));

    // Action Servers
    drill_sample_server_ = rclcpp_action::create_server<DrillSample>(
      this, "drill_sample",
      std::bind(&DrillController::handle_goal_drill_sample, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrillController::handle_cancel_drill_sample, this, std::placeholders::_1),
      std::bind(&DrillController::handle_accepted_drill_sample, this, std::placeholders::_1));

    store_sample_server_ = rclcpp_action::create_server<StoreSample>(
      this, "store_sample",
      std::bind(&DrillController::handle_goal_store_sample, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrillController::handle_cancel_store_sample, this, std::placeholders::_1),
      std::bind(&DrillController::handle_accepted_store_sample, this, std::placeholders::_1));

    drill_calibration_server_ = rclcpp_action::create_server<DrillCalibration>(
      this, "drill_calibration",
      std::bind(&DrillController::handle_goal_drill_calibration, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DrillController::handle_cancel_drill_calibration, this, std::placeholders::_1),
      std::bind(&DrillController::handle_accepted_drill_calibration, this, std::placeholders::_1));

    // Service Servers
    get_sample_weight_srv_ = this->create_service<GetSampleWeight>(
      "get_sample_weight",
      std::bind(&DrillController::get_sample_weight_callback, this, std::placeholders::_1, std::placeholders::_2));

    get_drill_status_srv_ = this->create_service<GetDrillStatus>(
        "get_drill_status",
        std::bind(&DrillController::get_drill_status_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

    DrillLogger_ = std::make_shared<DrillLogger>();
    DrillStatus_ = std::make_shared<DrillStatus>();


}

void DrillController::execute_drill_sample(const std::shared_ptr<GoalHandleDrillSample> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing DrillSample action...");
    DrillLogger_->logActionStamp(1);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DrillSample::Result>();
    auto feedback = std::make_shared<DrillSample::Feedback>();
    rclcpp::Rate loop_rate(drill_constants::LOOP_RATE);

    enum state_machine currentState = state_machine::stop;

    while(drillDepth < goal->depth-1) {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->final_depth = drillDepth;
            goal_handle->canceled(result);
            currentState = state_machine::stop;
            publish_drill_state(currentState);
            DrillLogger_->logDrillSampleResult(drillDepth);
            RCLCPP_INFO(this->get_logger(), "DrillSample action canceled.");
            return;
        }

        if (activeSlot != drill_constants::DEF_SLOT)
        {
            if (currentState != state_machine::slot_select) {
                currentState = state_machine::slot_select;
                publish_drill_param(0, 0, drill_constants::DEF_SLOT);
                publish_drill_state(currentState);
            }
        }
        else if(drillDepth == 0) {
            if (currentState != state_machine::goto_height) {
                currentState = state_machine::goto_height;
                publish_drill_param(0, calculate_height(goal->depth), 0);
                publish_drill_state(currentState);
            }
        }
        else
        {
            if (currentState != state_machine::drilling) {
                currentState = state_machine::drilling;
                publish_drill_param(goal->max_rps, calculate_height(goal->depth), 0);
                publish_drill_state(currentState);
            }
            else if (DrillStatus_->isMotorStucked())
                break;
        }

        // Sending feedback
        feedback->actual_torque = motorTorque;
        feedback->actual_rps = motorRPS;
        feedback->actual_height = drillHeight;
        goal_handle->publish_feedback(feedback);
        DrillLogger_->logDrillSampleData(motorTorque, motorRPS, motorTemperature, drillHeight);
        //RCLCPP_INFO(this->get_logger(), "Drilling progress torque: %f rps: %f temp: %d height: %d", motorTorque.load(), motorRPS.load(), motorTemperature.load(),drillHeight.load());
        loop_rate.sleep();
    }

    // Drilling complete
    result->final_depth = drillDepth;
    goal_handle->succeed(result);
    currentState = state_machine::stop;
    publish_drill_state(currentState);
    drill_is_busy = false;
    DrillLogger_->logDrillSampleResult(drillDepth);
    RCLCPP_INFO(this->get_logger(), "DrillSample action completed.");
}

void DrillController::execute_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing StoreSample action...");
    DrillLogger_->logActionStamp(2);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<StoreSample::Result>();
    auto feedback = std::make_shared<StoreSample::Feedback>();
    rclcpp::Rate loop_rate(drill_constants::LOOP_RATE);


    enum state_machine currentState = state_machine::stop;
    int state = 0;

    auto toDo = 0;
    auto storing_slot =  goal->slot;

    constexpr auto dumpingTicks = drill_constants::DUMPING_TIME * drill_constants::LOOP_RATE;
    auto ticks = 0;

    if (goal->slot > drill_constants::MAX_SLOT)
    {
        toDo = 1;
        storing_slot = 0;
    }
    else if (goal->slot == 0) { toDo = 2; }

    while(state < 8) {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->weight = 0;
            goal_handle->canceled(result);
            currentState = state_machine::stop;
            publish_drill_state(currentState);
            DrillLogger_->logStoreSampleResult(goal->slot, result->weight);
            RCLCPP_INFO(this->get_logger(), "StoreSample action canceled.");
            return;
        }

        switch (state) {
            case 0: //premisteni vrtacky nad zasobnik
                if(height_inTolerance(drill_constants::SAFE_POS))
                {
                    if (toDo == 0) {state = 1;}
                    else if (toDo == 1) {state = 3;}
                    else {state = 4;}
                }

                else
                {
                    if(currentState != state_machine::goto_height)
                    {
                        currentState = state_machine::goto_height;
                        publish_drill_param(0, drill_constants::SAFE_POS, drill_constants::DEF_SLOT);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 1: //nastaveni slotu pro zasobnik
                if(activeSlot == storing_slot)
                    state = 2;
                else
                {
                    if(currentState != state_machine::slot_select)
                    {
                        currentState = state_machine::slot_select;
                        publish_drill_param(0, drill_constants::SAFE_POS, storing_slot);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 2: //nastaveni pozice pro vysypani
                if(height_inTolerance(drill_constants::STORING_POS))
                    state = 3;
                else
                {
                    if(currentState != state_machine::goto_height)
                    {
                        currentState = state_machine::goto_height;
                        publish_drill_param(0, drill_constants::STORING_POS, storing_slot);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 3: //tare vahy
                if(DrillStatus_->isStorageTared() && currentState == state_machine::tare_scale)
                    state = 4;
                else
                {
                    if(currentState != state_machine::tare_scale)
                    {
                        currentState = state_machine::tare_scale;
                        publish_drill_param(1, drill_constants::STORING_POS, storing_slot);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 4: //vysypani
                if(ticks > dumpingTicks)
                    state = 5;
                else
                {
                    ++ticks;
                    if(currentState != state_machine::turn_right)
                    {
                        currentState = state_machine::turn_right;
                        publish_drill_param(1, drill_constants::STORING_POS, storing_slot);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 5: //zvednuti vrtaku, aby se mohlo tocit se zasobnikem
                if(height_inTolerance(drill_constants::SAFE_POS))
                    state = 6;
                else
                {
                    if(currentState != state_machine::goto_height)
                    {
                        currentState = state_machine::goto_height;
                        publish_drill_param(0, drill_constants::SAFE_POS, storing_slot);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 7: //zasobnik zpet na default slot
                if(activeSlot == drill_constants::DEF_SLOT)
                    state = 8;
                else
                {
                    if(currentState != state_machine::slot_select)
                    {
                        currentState = state_machine::slot_select;
                        publish_drill_param(0, drill_constants::SAFE_POS, drill_constants::DEF_SLOT);
                        publish_drill_state(currentState);
                    }
                }
            break;

            case 6: //vazeni
                if(!DrillStatus_->isStorageTared() == 1 && currentState == state_machine::get_weight)
                    state = 7;
                else
                {
                    if(currentState != state_machine::get_weight)
                    {
                        currentState = state_machine::get_weight;
                        publish_drill_param(0, drill_constants::SAFE_POS, storing_slot);
                        publish_drill_state(currentState);
                    }
                }
            break;

            default:
                state = 7;
            break;

        }

        // Sending feedback
        feedback->actual_height = drillHeight;
        goal_handle->publish_feedback(feedback);
        //RCLCPP_INFO(this->get_logger(), "Storing sample, drill height: %d", drillHeight.load());
        loop_rate.sleep();
    }

    // Drilling complete
    result->weight = get_sampleWeight(storing_slot);
    goal_handle->succeed(result);
    currentState = state_machine::stop;
    publish_drill_state(currentState);
    drill_is_busy = false;
    DrillLogger_->logStoreSampleResult(goal->slot, result->weight);
    RCLCPP_INFO(this->get_logger(), "StoreSample action completed.");
}

void DrillController::execute_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing DrillCalibration action...");
    DrillLogger_->logActionStamp(3);
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DrillCalibration::Result>();
    auto feedback = std::make_shared<DrillCalibration::Feedback>();
    rclcpp::Rate loop_rate(drill_constants::LOOP_RATE);

    enum state_machine currentState = state_machine::stop;
    int state = 0;

    while(state < 2)
    {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            currentState = state_machine::stop;
            publish_drill_state(currentState);
            DrillLogger_->logDrillCalibration(false);
            RCLCPP_INFO(this->get_logger(), "DrillCalibration action canceled.");
            return;
        }

        switch (state)
        {
            case 0:
                if (drillHeight == 0)
                    state = 1;
                else if (currentState != state_machine::goto_height) {
                    currentState = state_machine::goto_height;
                    publish_drill_state(currentState);
                    publish_drill_param(0, 0, drill_constants::DEF_SLOT);
                }
                break;
            case 1:
                if (!goal->reset_weights || (get_sampleWeight(1) == 0 && get_sampleWeight(2) == 0 && get_sampleWeight(3) == 0 && get_sampleWeight(4) == 0) )
                    state = 2;
                else if (currentState != state_machine::reset_weight) {
                    currentState = state_machine::reset_weight;
                    publish_drill_param(0, 0, drill_constants::DEF_SLOT);
                    publish_drill_state(currentState);
                }
                break;
        default:
            break;
        }
        // Sending feedback
        feedback->actual_height = drillHeight;
        goal_handle->publish_feedback(feedback);
        //RCLCPP_INFO(this->get_logger(), "Erasing the weights");
        loop_rate.sleep();
    }

    // Calibration complete
    result->success = true;
    goal_handle->succeed(result);
    currentState = state_machine::stop;
    publish_drill_state(currentState);
    drill_is_busy = false;
    DrillLogger_->logDrillCalibration(goal->reset_weights);
    RCLCPP_INFO(this->get_logger(), "DrillCalibration action completed.");
}

void DrillController::publish_drill_param(const float rps, const int height, const int slot) const{
    auto message = std_msgs::msg::UInt16MultiArray();
    message.data.resize(3);
    message.data[0] = float_code(rps);
    message.data[1] = height;
    message.data[2] = slot;
    drill_params_pub_->publish(message);
    //RCLCPP_INFO(this->get_logger(), "Published drill parameters rps %f, height %d, slot %d", rps, height, slot);
}

void DrillController::get_drill_status_callback(const std::shared_ptr<drill_interfaces::srv::GetDrillStatus::Request> request,
    std::shared_ptr<drill_interfaces::srv::GetDrillStatus::Response> response)
{
    (void)request;
    
    response->motor_i2c = DrillStatus_->getMotorI2CStatus();
    response->motor_stucked = DrillStatus_->getMotorStucked();
    response->motor_error = DrillStatus_->getMotorError();

    response->linear_i2c = DrillStatus_->getLinearI2CStatus();
    response->linear_error = DrillStatus_->getLinearError();

    response->storage_i2c = DrillStatus_->getStorageI2CStatus();
    response->storage_scale_tared = DrillStatus_->getStorageScaleTared();
    response->storage_error = DrillStatus_->getStorageError();
}

void DrillController::publish_drill_state(const state_machine state) const{
    auto message = std_msgs::msg::UInt8();
    message.data = static_cast<uint8_t>(state);
    drill_state_pub_->publish(message);
    //RCLCPP_INFO(this->get_logger(), "Published drill state: %d", static_cast<uint8_t>(state));
}

void DrillController::drill_data_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received drill data.");
    DrillStatus_->setStatus(msg->data[0]);
    motorRPS = float_decode(msg->data[1]);
    motorTorque = float_decode(msg->data[2]);
    motorTemperature = msg->data[3];
    drillHeight = msg->data[4];;
    if (drillHeight == 0) toGround = msg->data[5];
    set_depth();
    activeSlot = msg->data[6];;
    sampleWeight1 = roundf(static_cast<float>(msg->data[7]) / 10.0f * 10.0f) / 10.0f;
    sampleWeight2 = roundf(static_cast<float>(msg->data[8]) / 10.0f * 10.0f) / 10.0f;
    sampleWeight3 = roundf(static_cast<float>(msg->data[9]) / 10.0f * 10.0f) / 10.0f;
    sampleWeight4 = roundf(static_cast<float>(msg->data[10]) / 10.0f * 10.0f) / 10.0f;
}

float DrillController::get_sampleWeight(const int slot)
{
    switch (slot)
    {
    case 1:
        return sampleWeight1;
    case 2:
        return sampleWeight2;
    case 3:
        return sampleWeight3;
    case 4:
        return sampleWeight4;
    default:
        return 0;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrillController>());
    rclcpp::shutdown();
    return 0;
}