#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <sensor_msgs/msg/joy.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "storage_driver.h"
#include "linear_driver.h"
#include "motor_driver.h"
#include "math.h"

#define I2C_PORT i2c0
#define BUFFER_SIZE 10
#define STRING_SIZE 30

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

enum state_machine currentState = stop;

struct storage storage;
struct linear linear;
struct motor motor;

const uint LED_PIN = 25;

rcl_subscription_t state_subscriber;
std_msgs__msg__UInt8 msg_state;

rcl_subscription_t parameters_subscriber;
std_msgs__msg__UInt16MultiArray msg_parameters; 

rcl_subscription_t joy_subscriber;
sensor_msgs__msg__Joy msg_joy;

rcl_publisher_t publisher;
std_msgs__msg__UInt16MultiArray msg_data; 

void timerPublisher_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    msg_data.data.data[0] = motor.rpsMeas;
    msg_data.data.data[1] = motor.torqueMeas;
    msg_data.data.data[2] = motor.temperature;
    msg_data.data.data[3] = linear.height;
    msg_data.data.data[4] = linear.toGround;
    msg_data.data.data[5] = storage.active_slot;
    msg_data.data.data[6] = storage.samples[0];
    msg_data.data.data[7] = storage.samples[1];
    msg_data.data.data[8] = storage.samples[2];
    msg_data.data.data[9] = storage.samples[3];
    msg_data.data.data[10] = storage.samples[4];
    rcl_ret_t ret = rcl_publish(&publisher, &msg_data, NULL);  
}

void timerMain_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    switch (currentState) 
    {
        case stop:
            motor_stop(&motor);
            linear_stop(&linear);
            break;

        case drilling:
            if(is_motor_stucked(&motor) || is_linear_stucked(&linear) || linear_reached_goal(&linear))
            {
                motor_stop(&motor);
                linear_stop(&linear);
            }
            else
            {
                motor_right(&motor);
                linear_go_down(&linear);
            }
            break;
        
        case go_down:
            motor_stop(&motor);
            if(!is_linear_stucked(&linear))
                linear_go_down(&linear);
            else
                linear_stop(&linear);
            break;    

        case go_up:
            motor_stop(&motor);
            if(is_linear_home(&linear))
                linear_stop(&linear);
            else
            {
                motor_unblock(&motor);
                linear_go_up(&linear);
            }
            break;
            
        case turn_left:
            linear_stop(&linear);
            motor_left(&motor);
            break;

        case turn_right:
            linear_stop(&linear);
            if(is_motor_stucked(&motor))
                motor_stop(&motor);
            else
                motor_right(&motor);
            break;

        case slot_select:
            motor_stop(&motor);
            linear_stop(&linear);
            if (is_linear_safe(&linear))
                storage_goto(&storage);
    
            break;

        case get_weight:
            motor_stop(&motor);
            linear_stop(&linear);
            storage_get_weight(&storage);
            break;

        case reset_weight:
            storage_wreset(&storage);
            break;

        default:
            // manual mode
            break;
    }

    //if(currentState != manual)
    //{
        motor_write(&motor);    
        linear_write(&linear);
        if (storage.command != storage.old_command)
            storage_write(&storage);
        
    //}
    motor_read(&motor);
    linear_read(&linear);
    storage_read(&storage);


}

void state_callback(const void * msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    currentState = msg->data;
}

void parameters_callback(const void * msgin)
{
    const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;

    motor.rpsGoal = msg->data.data[0];
    linear.speed = msg->data.data[1];
    linear.height = msg->data.data[2];
    storage.demand_pos = msg->data.data[3];
}

void joy_callback(sensor_msgs__msg__Joy* msgin)
{
    if (currentState == manual)
    {
        // Set linear state
        if (msgin->axes.data[1] == 0.0) { linear.command = 1; }      // stop linear
        else if (msgin->axes.data[1] > 0.0) { linear.command = 3; }  // up linear
        else { linear.command = 2; }                                 // down linear
    
        // Set the linear speed
        linear.speed = (uint8_t)(fabs(msgin->axes.data[1]) * 255.0f); //calculating speed
        if(linear.state == 0 && linear.command == 3) { linear.command = 1; } //kontrola koncaku
        //linear_write(&linear);
    
        // Set the motor
        if (msgin->axes.data[5] < 1)    //left 
        { 
            motor.rps =  (int8_t)((1 - msgin->axes.data[5]) / 2.0f * 66.7f); 
        }
        else    //right
        { 
            motor.rps =  (int8_t)(-(1 - msgin->axes.data[2]) / 2.0f * 66.7f);
        }
        //motor_write(&motor);
    
        // Set the storage command
        uint8_t old_command = storage.command;
        if (msgin->buttons.data[1] == 1) { storage.command = 31; }          //pos 1
        else if (msgin->buttons.data[2] == 1) {storage.command = 32;}       //pos 2
        else if (msgin->buttons.data[0] == 1) {storage.command = 33;}       //pos 3
        else if (msgin->buttons.data[3] == 1) {storage.command = 30;}       //pos 0
        else if (msgin->buttons.data[4] == 1) {storage.command = 20;}       //get weight
        else if (msgin->buttons.data[5] == 1) {storage.command = 40;}       //hold pos
        //if(old_command != storage.command) { storage_write(&storage); }  
    }
       
}

int main()
{
    printf("starting");
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    rcl_timer_t timerPublisher;
    rcl_timer_t timerMain;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "drill_rpiPico", "", &support);
    
    rclc_subscription_init_default(
        &state_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "drill_state");

    rclc_subscription_init_default(
        &parameters_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "drill_parameters");

    // alocation memory to msg_parameters
    int16_t buffer[BUFFER_SIZE] = {};
    msg_parameters.data.data = buffer;
    msg_parameters.data.size = 4;
    msg_parameters.data.capacity = BUFFER_SIZE;
        
    std_msgs__msg__MultiArrayDimension dim[BUFFER_SIZE] = {};
    msg_parameters.layout.dim.data = dim;
    msg_parameters.layout.dim.size = 0;
    msg_parameters.layout.dim.capacity = BUFFER_SIZE;
        
    char labels[BUFFER_SIZE][STRING_SIZE] = {};
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        msg_parameters.layout.dim.data[i].label.data = labels[i];
        msg_parameters.layout.dim.data[i].label.size = 0;
        msg_parameters.layout.dim.data[i].label.capacity = STRING_SIZE;
    }
        
    msg_parameters.layout.data_offset = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        snprintf(msg_parameters.layout.dim.data[i].label.data, STRING_SIZE, "label_%lu", i);
        msg_parameters.layout.dim.data[i].label.size = strlen(msg_parameters.layout.dim.data[i].label.data);
        msg_parameters.layout.dim.data[i].size = 42;
        msg_parameters.layout.dim.data[i].stride = 42;
    }

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "drill_data");
    
    // alocation memory to msg
    msg_data.data.data = buffer;
    msg_data.data.size = 11;
    msg_data.data.capacity = BUFFER_SIZE;
        
    msg_data.layout.dim.data = dim;
    msg_data.layout.dim.size = 0;
    msg_data.layout.dim.capacity = BUFFER_SIZE;
        
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        msg_data.layout.dim.data[i].label.data = labels[i];
        msg_data.layout.dim.data[i].label.size = 0;
        msg_data.layout.dim.data[i].label.capacity = STRING_SIZE;
    }
        
    msg_data.layout.data_offset = 0;
    for (size_t i = 0; i < BUFFER_SIZE; i++)
    {
        snprintf(msg_data.layout.dim.data[i].label.data, STRING_SIZE, "label_%lu", i);
        msg_data.layout.dim.data[i].label.size = strlen(msg_data.layout.dim.data[i].label.data);
        msg_data.layout.dim.data[i].size = 42;
        msg_data.layout.dim.data[i].stride = 42;
    }

    rclc_subscription_init_default(
        &joy_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joy");

     //Allocation for joy message
    msg_joy.axes.capacity=100;
    msg_joy.axes.size = 0;
    msg_joy.axes.data = (float*) malloc(msg_joy.axes.capacity * sizeof(float));

    msg_joy.buttons.capacity=100;
    msg_joy.buttons.size = 0;
    msg_joy.buttons.data = (int32_t*) malloc(msg_joy.buttons.capacity * sizeof(int32_t));

    msg_joy.header.frame_id.capacity = 100;
    msg_joy.header.frame_id.data = (char*) malloc(msg_joy.header.frame_id.capacity * sizeof(char));
    msg_joy.header.frame_id.size = 0;

    // Assigning value to the frame_id char sequence
    strcpy(msg_joy.header.frame_id.data, "Hello World");
    msg_joy.header.frame_id.size = strlen(msg_joy.header.frame_id.data);

    msg_joy.header.stamp.sec = 10;
    msg_joy.header.stamp.nanosec = 20;

    rclc_timer_init_default(
        &timerPublisher,
        &support,
        RCL_MS_TO_NS(1000),
        timerPublisher_callback);

    rclc_timer_init_default(
        &timerMain,
        &support,
        RCL_MS_TO_NS(100),
        timerMain_callback);

    rclc_executor_init(&executor, &support.context, 5, &allocator); //the number of executors
    rclc_executor_add_timer(&executor, &timerMain);
    
    rclc_executor_add_subscription(
        &executor, &state_subscriber, &msg_state,
        &state_callback, ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor, &parameters_subscriber, &msg_parameters,
        &parameters_callback, ON_NEW_DATA);

    rclc_executor_add_subscription(
        &executor, &joy_subscriber, &msg_joy,
        &joy_callback, ON_NEW_DATA);
        
    rclc_executor_add_timer(&executor, &timerPublisher);
    
    //init i2c
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    gpio_put(LED_PIN, 1);

    storage_init(&storage);
    linear_init(&linear);
    motor_init(&motor);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
