#include "rclcpp/rclcpp.hpp"
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
    drill_is_busy = false;
    RCLCPP_INFO(this->get_logger(), "DrillSample action completed.");
}

  // If the drill is busy service will be refused
  rclcpp_action::GoalResponse handle_goal_store_sample(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const StoreSample::Goal> goal) {
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

  //executing the StoreSample service
  void execute_store_sample(const std::shared_ptr<GoalHandleStoreSample> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing StoreSample action...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<StoreSample::Result>();
    auto feedback = std::make_shared<StoreSample::Feedback>();
    rclcpp::Rate loop_rate(1);

    enum state_machine currentState;
    int state = 0;

    auto storing_slot = goal->slot;
    if (storing_slot > MAX_SLOT)
    {
        RCLCPP_WARN(this->get_logger(), "Storage doesn't have that many slots, sample will be thrown away.");
        storing_slot = DEF_SLOT;
    }

    while(state < 7)
    {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->weight = 0;
            goal_handle->canceled(result);
            currentState = stop;
            publish_drill_state(currentState);
            RCLCPP_INFO(this->get_logger(), "StoreSample action canceled.");
            return;
        }

        switch (state)
        {
            case 0: //premisteni vrtacky nad zasobnik
                if(actual_height < SAFE_POS)
                    state = 1;
                else
                {
                    if(currentState != go_up)
                    {
                        currentState = go_up;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 100, 0, DEF_SLOT);
                    }
                }
            break; 
            
            case 1: //nastaveni slotu pro zasobnik
                if(active_slot == storing_slot)
                    state = 2;
                else
                {
                    if(currentState != slot_select)
                    {
                        currentState = slot_select;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 0, SAFE_POS, storing_slot);
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
                        publish_drill_param(0, 50, STORING_POS, storing_slot);
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
                        publish_drill_param(1, 0, STORING_POS, storing_slot);
                    }
                }
            break;

            case 4: //zvednuti vrtaku, aby se mohlo tocit se zasobnikem
                if(actual_height < SAFE_POS)
                    state = 5;
                else
                {
                    if(currentState != go_up)
                    {
                        currentState = go_up;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 100, SAFE_POS, storing_slot);
                    }
                }
            break;

            case 5: //zasobnik zpet na default slot
                if(active_slot == DEF_SLOT)
                    state = 6;
                else
                {
                    if(currentState != slot_select)
                    {
                        currentState = slot_select;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 0, SAFE_POS, DEF_SLOT);
                    }
                }    
            break;

            case 6: //vazeni
                if(samples[storing_slot] || storing_slot == DEF_SLOT)
                    state = 7;
                else
                {
                    if(currentState != get_weight)
                    {
                        currentState = get_weight;
                        publish_drill_state(currentState);
                        publish_drill_param(0, 0, SAFE_POS, storing_slot);
                    }
                }    
            break;

        }
        
        // Sending feedback
        feedback->actual_height = actual_height;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Storing sample, drill height: %d", actual_height);
        loop_rate.sleep();

    }

    
    // Drilling complete
    result->weight = samples[storing_slot];
    goal_handle->succeed(result);
    currentState = stop;
    publish_drill_state(currentState);
    drill_is_busy = false;
    RCLCPP_INFO(this->get_logger(), "StoreSample action completed.");
}


  rclcpp_action::GoalResponse handle_goal_drill_calibration(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const DrillCalibration::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received DrillCalibration request. Weights will be: %s", goal->reset_weights ? "erased" : "kept");      
        (void)uuid;

        if (drill_is_busy) 
        {
            RCLCPP_WARN(this->get_logger(), "Drill is busy or in manual mode. Rejecting request.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Drill is ready. Accepting request.");
        drill_is_busy = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  
    }

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
  void execute_drill_calibration(const std::shared_ptr<GoalHandleDrillCalibration> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing DrillCalibration action...");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DrillCalibration::Result>();
    auto feedback = std::make_shared<DrillCalibration::Feedback>();
    rclcpp::Rate loop_rate(1);

    enum state_machine currentState;

    while(actual_height != 0)
    {
        //Checking if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            currentState = stop;
            publish_drill_state(currentState);
            RCLCPP_INFO(this->get_logger(), "DrillCalibration action canceled.");
            return;
        }
        
        if (currentState != go_up)
            {
                currentState = go_up;
                publish_drill_state(currentState);
                publish_drill_param(0, 100, 0, DEF_SLOT);
            }
        

        // Sending feedback
        feedback->actual_height = actual_height;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Calibration progress height: %d mm", actual_height);
        loop_rate.sleep();
    }

    if(goal->reset_weights)
    {        
        while (samples[0] && samples[1] && samples[2] && samples[3])
        {
            if (currentState != reset_weight)
            {
                currentState = reset_weight;
                publish_drill_state(currentState);
                publish_drill_param(0, 0, 0, DEF_SLOT);
            }

            // Sending feedback
            feedback->actual_height = actual_height;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Reseting the weights");
            loop_rate.sleep();
        }
    }

    
    // Calibration complete
    result->success = true;
    goal_handle->succeed(result);
    currentState = stop;
    publish_drill_state(currentState);
    drill_is_busy = false;
    RCLCPP_INFO(this->get_logger(), "DrillCalibration action completed.");
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
        RCLCPP_INFO(this->get_logger(), "Published drill parameters torque %f, speed %d, height %d, slot %d", 
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
