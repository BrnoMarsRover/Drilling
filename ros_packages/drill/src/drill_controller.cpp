#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "drill_interfaces/action/drill_calibration.hpp"
#include "drill_interfaces/action/drill_sample.hpp"
#include "drill_interfaces/action/store_sample.hpp"
#include "drill_interfaces/srv/get_sample_weight.hpp"

#define STORING_POS 30 // [mm]
#define MAX_TORQUE 2.5
#define RANGE 1000

enum state_machine 
{
    stop,
    drilling,
    go_down,
    go_up,
    turn_right,
    turn_left,
    slot_select,
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

    DrillController() : Node("drill_controller") {
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

      // Service Server
      get_sample_weight_srv_ = this->create_service<GetSampleWeight>(
          "get_sample_weight",
          std::bind(&DrillController::get_sample_weight_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:

  bool drill_is_busy;
  float_t actual_torque;
  uint16_t actual_height;
  uint16_t active_slot;
  uint16_t samples[4];

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drill_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr drill_params_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr drill_data_sub_;

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

  // If the drill is busy service will be refused
  rclcpp_action::GoalResponse handle_goal_drill_sample(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DrillSample::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received DrillSample request.");

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
      drill_is_busy = false;
      return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_drill_sample(const std::shared_ptr<GoalHandleDrillSample> goal_handle) {
      std::thread{std::bind(&DrillController::execute_drill_sample, this, std::placeholders::_1), goal_handle}.detach();
  }

  //executing the DrillSample service
  void execute_drill_sample(const std::shared_ptr<GoalHandleDrillSample> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing DrillSample action...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DrillSample::Result>();
    auto feedback = std::make_shared<DrillSample::Feedback>();
    rclcpp::Rate loop_rate(1);

    enum state_machine currentState;

    while(actual_height < goal->height)
    {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->finalheight = actual_height;
            goal_handle->canceled(result);
            currentState = stop;
            publish_drill_state(currentState);
            RCLCPP_INFO(this->get_logger(), "DrillSample action canceled.");
            return;
        }

        if(actual_height < 100)
        {
            if (currentState != go_down)
            {
                currentState = go_down;
                publish_drill_state(currentState);
                publish_drill_param(0, 100, goal->height, 0);
            }
        }
        else
        {
            if (currentState != drilling)
            {
                currentState = drilling;
                publish_drill_state(currentState);
                publish_drill_param(goal->torque, 70, goal->height, 0);
            }
        }

        // Sending feedback
        feedback->actual_torque = actual_torque;
        feedback->actual_height = actual_height;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Drilling progress torque: %f height: %d", actual_torque, actual_height);
        loop_rate.sleep();
    }

    
    // Drilling complete
    result->finalheight = actual_height;
    goal_handle->succeed(result);
    currentState = stop;
    publish_drill_state(currentState);
    RCLCPP_INFO(this->get_logger(), "DrillSample action completed.");
}

  // If the drill is busy service will be refused
  rclcpp_action::GoalResponse handle_goal_store_sample(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const StoreSample::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received StoreSample request.");

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
      drill_is_busy = false;
      return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle) {
      std::thread{std::bind(&DrillController::execute_store_sample, this, std::placeholders::_1), goal_handle}.detach();

  }

  //executing the StoreSample service
  void execute_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing StoreSample action...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<StoreSample::Result>();
    auto feedback = std::make_shared<StoreSample::Feedback>();
    rclcpp::Rate loop_rate(1);

    enum state_machine currentState;
    int state = 0;

    while(true)
    {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->finalheight = 0;
            goal_handle->canceled(result);
            currentState = stop;
            publish_drill_state(currentState);
            RCLCPP_INFO(this->get_logger(), "StoreSample action canceled.");
            return;
        }

        switch (state)
        {
            case 0: //jizda do pozice home
                if(actual_height = 0)
                    state = 1;
                else
                {
                    if(currentState != go_up)
                    {
                        currentState = go_up;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 100, 0);
                    }
                }
            break; 
            
            case 1: //nastaveni slotu pro zasobnik
                if(active_slot == goal->slot)
                    state = 2;
                else
                {
                    if(currentState != slot_select)
                    {
                        currentState = slot_select;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 0, 0, goal->slot);
                    }
                }    
            break;

            case 2: //nastaveni pozice pro vysypani
                if(actual_height == STORING_POS)
                    state = 3;
                else
                {
                    if(currentState != go_down)
                    {
                        currentState = go_down;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 50, STORING_POS, goal->slot);
                    }
                }
            break;

            case 3: //vysypani (budu muset vymyslet casovani)
                if(actual_height == STORING_POS)
                    state = 4;
                else
                {
                    if(currentState != turn_left)
                    {
                        currentState = turn_left;
                        publish_drill_state(currentState);
                        publish_drill_param(1, 0, STORING_POS, goal->slot);
                    }
                }
            break;

        }

        if(actual_height < 100)
        {
            if (currentState != go_down)
            {
                currentState = go_down;
                publish_drill_state(currentState);
                publish_drill_param(0, 100, goal->height);
            }
        }
        else
        {
            if (currentState != drilling)
            {
                currentState = drilling;
                publish_drill_state(currentState);
                publish_drill_param(goal->torque, 70, goal->height);
            }
        }
        
        // Sending feedback
        feedback->actual_torque = actual_torque;
        feedback->actual_height = actual_height;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Drilling progress torque: %f height: %d", actual_torque, actual_height);
        loop_rate.sleep();

    }

    
    // Drilling complete
    result->finalheight = actual_height;
    goal_handle->succeed(result);
    currentState = stop;
    publish_drill_state(currentState);
    RCLCPP_INFO(this->get_logger(), "DrillSample action completed.");
}


  rclcpp_action::GoalResponse handle_goal_drill_calibration(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DrillCalibration::Goal> goal) {
      RCLCPP_INFO(this->get_logger(), "Received DrillCalibration request.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "DrillCalibration canceled.");
      return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Executing DrillCalibration action...");
  }

  void get_sample_weight_callback(
      const std::shared_ptr<GetSampleWeight::Request> request,
      std::shared_ptr<GetSampleWeight::Response> response) {
      RCLCPP_INFO(this->get_logger(), "Getting sample weight for slot %d", request->slot);
      response->weight = samples[request->slot];
  }

  // Function for publish state to drill
  void publish_drill_state(uint8_t state) {
    auto message = std_msgs::msg::UInt8();
    message.data = state;
    drill_state_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published drill state: %d", state);
    }

    // Function to publish drill parameters
    void publish_drill_param(float torque, uint16_t speed, uint16_t height, uint16_t slot) {
        auto message = std_msgs::msg::UInt16MultiArray();
        message.data.resize(4);
        message.data[0] = float_code(torque);
        message.data[1] = speed;
        message.data[2] = height;
        message.data[3] = slot;


        drill_params_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published drill parameters torque %f, speed %d, height %d, slot %d" 
                    torque, speed, height, slot);
    }

    float float_decode(uint16_t aNum)
    {
        float constant = MAX_TORQUE/RANGE;
        float result =  constant * (float)aNum;
        return result;
    }

    uint16_t float_code(float aNum)
    {
        float constant = RANGE/MAX_TORQUE;
        uint16_t result =  (uint16_t)(constant * aNum);
        return result;
    }
    

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrillController>());
  rclcpp::shutdown();
  return 0;
}
