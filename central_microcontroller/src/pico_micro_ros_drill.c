/******************************************************************************
 * @file    pico_micro_ros_drill.c
 * @author  Martin Kriz
 * @brief   Main file containing the program's entry point and initialization routines.
 * @date    2025-04-26
 ******************************************************************************/

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
#include "storage_driver.h"
#include "linear_driver.h"
#include "motor_driver.h"
#include "math.h"
#include "hardware/watchdog.h"

#define MAIN_LOOP_TIME_MS 50    // Period of i2c communication
#define I2C_PORT i2c0
#define BUFFER_SIZE 10
#define STRING_SIZE 30

/**
 * @brief   Enum for /drill_state topic, must be same as enum in drill_controller node
 */
enum state_machine
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

uint64_t last_joy_topic = 0;
uint64_t last_state_topic = 0;

// State machine global variable
enum state_machine currentState = stop;

// Subsystems representation
struct storage storage;
struct linear linear;
struct motor motor;

// Green LED, RUN indicator
const uint LED_PIN = 25;

// Reset Pins
const uint TMP_RESET_PIN;
const uint MOTOR_RESET_PIN = 7;
const uint LINEAR_RESET_PIN = 8;
const uint STORAGE_RESET_PIN = 9;

// Output enable for logic converter
const uint OE_PIN = 3;
bool reset_done = false;

// Topics variables
rcl_subscription_t state_subscriber;
std_msgs__msg__UInt8 msg_state;

rcl_subscription_t parameters_subscriber;
std_msgs__msg__UInt16MultiArray msg_parameters; 

rcl_subscription_t joy_subscriber;
sensor_msgs__msg__Joy msg_joy;

rcl_publisher_t publisher;
std_msgs__msg__UInt16MultiArray msg_data; 

/**
 * @brief   Make status message for topic /drill_data index 0.
 * @param   linear Pointer to the linear actuator structure.
 * @param   motor Pointer to the motor structure.
 * @param   storage Pointer to the storage structure.
 * @return  Status message.
 */
uint16_t drill_message(struct motor* motor, struct linear* linear, struct storage* storage);

// Callback for publishing on /drill_data topic
void timerPublisher_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // Status
    msg_data.data.data[0] = drill_message(&motor, &linear, &storage);

    // Motor variables
    msg_data.data.data[1] = abs(motor.rpsMeas);
    msg_data.data.data[2] = abs(motor.torqueMeas);
    msg_data.data.data[3] = motor.temperature;
    
    // Linear variables
    msg_data.data.data[4] = linear.height; 
    msg_data.data.data[5] = linear.toGround;
    
    // Storage variables
    msg_data.data.data[6] = storage.active_slot;
    for (int i = 0; i < STORE_SLOTS; ++i) 
    {
        msg_data.data.data[7 + i] = storage.samples[i];
    }
    
    rcl_ret_t ret = rcl_publish(&publisher, &msg_data, NULL);  
}

// Main callback, i2c communication and state machine
void timerMain_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // Checking connection with /drill_controller
    uint64_t now = time_us_64();
    if (now - last_state_topic > 1000000)
        currentState = stop;
    
    if (currentState != reset_subsystems)
    {
        reset_done = false;
    }

    // Input reading
    if(motor_read(&motor) < 0) {motor.i2cStatus = false;}
    else {motor.i2cStatus = true;}

    if(linear_read(&linear) < 0) {linear.i2cStatus = false;}
    else {linear.i2cStatus = true;}

    if(storage_read(&storage) < 0) {storage.i2cStatus = false;}
    else {storage.i2cStatus = true;}

    // State machine
    switch (currentState) 
    {
        case stop:
            if (now - last_joy_topic > 1000000) // More then one second
            {
                motor_stop(&motor);
                linear_stop(&linear);
            }
            break;

        case drilling:
            if(is_motor_stucked(&motor) || is_linear_stucked(&linear) || linear_reached_goal(&linear))
            {
                motor_stop(&motor);
                linear_stop(&linear);
            }
            else
            {
                motor_left(&motor);
                set_drilling_speed(&linear, get_rps_error(&motor), MAIN_LOOP_TIME_MS/1000.0f);
            }
            break;
        
        case goto_height:
            motor_stop(&motor);

            if (can_linear_goDown(&linear))
            {
                if(!is_linear_stucked(&linear))
                {
                    linear_goto(&linear, MAIN_LOOP_TIME_MS/1000.0f);
                    if (linear.command == 3)
                        motor_unblock(&motor);
                }
                else
                    linear_stop(&linear);
            }
            else
            {
                if (is_storage_ok(&storage))
                {
                    if(!is_linear_stucked(&linear))
                    {
                        linear_goto(&linear, MAIN_LOOP_TIME_MS/1000.0f);
                        if (linear.command == 3)
                            motor_unblock(&motor);
                    }
                    else
                        linear_stop(&linear);
                }
                else
                {
                    linear_stop(&linear);
                }
            }

            break;    
            
        case turn_right:
            linear_stop(&linear);
            motor_right(&motor);
            break;

        case turn_left:
            linear_stop(&linear);
            if(is_motor_stucked(&motor))
                motor_stop(&motor);
            else
                motor_left(&motor);
            break;

        case slot_select:
            motor_stop(&motor);
            linear_stop(&linear);
            if (can_storage_move(&linear))
                storage_goto(&storage, storage.demand_pos);
            break;

        case tare_scale:
            motor_stop(&motor);
            linear_stop(&linear);
            if (!storage.scaleTared)
                storage_get_tared(&storage);
                            
            break;

        case get_weight:
            motor_stop(&motor);
            linear_stop(&linear);

            if (!storage.weight_recieved)
                storage_get_weight(&storage);
            else if (storage.scaleTared)
                    storage_reset(&storage);

            break;

        case reset_weight:
            storage_wreset(&storage);
            break;

        case reset_pico:
            watchdog_enable(1, 1);
            while (true);
            break;

        default:
            motor.stucked = false;
            //if (!reset_done && gpio_get(MOTOR_RESET_PIN) == true)
            if (!reset_done && gpio_get(TMP_RESET_PIN) == false)
            {
                ///////////////////////////////
                gpio_put(TMP_RESET_PIN, 1);
                ///////////////////////////////
                //gpio_put(MOTOR_RESET_PIN, 0);
                //gpio_put(LINEAR_RESET_PIN, 0);
                //gpio_put(STORAGE_RESET_PIN, 0);
                sleep_ms(50);
                reset_done = true;
            }
            ////////////////////////////
            gpio_put(TMP_RESET_PIN, 0);
            ////////////////////////////
            //gpio_put(MOTOR_RESET_PIN, 1);
            //gpio_put(LINEAR_RESET_PIN, 1);
            //gpio_put(STORAGE_RESET_PIN, 1);
            break;
    }

    // Writing outputs
    motor_write(&motor); 
    linear_write(&linear);
    if (storage.command != 0) {storage_write(&storage);}

}

// Callback for recieving /drill_state
void state_callback(const void * msgin)
{
    last_state_topic = time_us_64();
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    currentState = msg->data;
}

// Callback for recieving /drill_parameters
void parameters_callback(const void * msgin)
{
    const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;

    motor.rpsGoal = msg->data.data[0];
    linear.goalHeight = msg->data.data[1];
    storage.demand_pos = msg->data.data[2];
}

// Callback for recieving /drill_joy and handle manual controlling
void joy_callback(const void * msgin)
{
    const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;
    last_joy_topic = time_us_64();
    if (currentState == stop)
    {
        // Set linear state
        if (msg->axes.data[1] == 0.0) { linear.command = 1; }      // stop linear
        else if (msg->axes.data[1] > 0.0) { linear.command = 3; }  // up linear
        else { linear.command = 2; }                                 // down linear
    
        // Set the linear speed
        linear.speed = (uint8_t)(fabs(msg->axes.data[1]) * 255.0f); //calculating speed
        if(linear.state == 0 && linear.command == 3) { linear.command = 1; } //kontrola koncaku
    
        // Set the motor
        if (msg->axes.data[5] < 1)    //left 
        { 
            motor.rps =  (int8_t)((1 - msg->axes.data[5]) / 2.0f * 66.7f); 
        }
        else    //right
        { 
            motor.rps =  (int8_t)(-(1 - msg->axes.data[2]) / 2.0f * 66.7f);
        }
    
        // Set the storage command
        if (msg->buttons.data[0] == 1) { storage.demand_pos = 1; storage_goto(&storage, storage.demand_pos); }          //pos 1
        else if (msg->buttons.data[1] == 1) {storage.demand_pos = 2; storage_goto(&storage, storage.demand_pos);}       //pos 2
        else if (msg->buttons.data[2] == 1) {storage.demand_pos = 3; storage_goto(&storage, storage.demand_pos);}       //pos 2
        else if (msg->buttons.data[3] == 1) {storage.demand_pos = 4; storage_goto(&storage, storage.demand_pos);}       //pos 3
        else if (msg->buttons.data[10] == 1) {storage.demand_pos = 0; storage_goto(&storage, storage.demand_pos);}      //pos 0
        else if (msg->buttons.data[9] == 1) {storage.command = 10;}               //tare
        else if (msg->buttons.data[8] == 1) {storage.command = 50;}               //error clear
        else if (msg->buttons.data[4] == 1) {storage_get_weight(&storage);}       //get weight
        else if (msg->buttons.data[5] == 1) {storage_hold(&storage);}             //hold pos
        else {storage.command = 0;}
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

    gpio_init(MOTOR_RESET_PIN);
    gpio_set_dir(MOTOR_RESET_PIN, GPIO_OUT);

    gpio_init(LINEAR_RESET_PIN);
    gpio_set_dir(LINEAR_RESET_PIN, GPIO_OUT);

    gpio_init(STORAGE_RESET_PIN);
    gpio_set_dir(STORAGE_RESET_PIN, GPIO_OUT);

    gpio_init(OE_PIN);
    gpio_set_dir(OE_PIN, GPIO_OUT);

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
        // Unreachable agent, resetting pico.
        watchdog_enable(1, 1);
        while (true);     
    }

    // Setting up node
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

    // Alocation memory for msg_parameters
    int16_t buffer[BUFFER_SIZE] = {};
    msg_parameters.data.data = buffer;
    msg_parameters.data.size = 3;
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
    
    // Alocation memory for msg_data
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

     // Allocation memmory for joy message
    msg_joy.axes.capacity=100;
    msg_joy.axes.size = 0;
    msg_joy.axes.data = (float*) malloc(msg_joy.axes.capacity * sizeof(float));

    msg_joy.buttons.capacity=100;
    msg_joy.buttons.size = 0;
    msg_joy.buttons.data = (int32_t*) malloc(msg_joy.buttons.capacity * sizeof(int32_t));

    msg_joy.header.frame_id.capacity = 100;
    msg_joy.header.frame_id.data = (char*) malloc(msg_joy.header.frame_id.capacity * sizeof(char));
    msg_joy.header.frame_id.size = 0;

    strcpy(msg_joy.header.frame_id.data, "Hello World");
    msg_joy.header.frame_id.size = strlen(msg_joy.header.frame_id.data);

    msg_joy.header.stamp.sec = 10;
    msg_joy.header.stamp.nanosec = 20;

    // Setting timers
    rclc_timer_init_default(
        &timerPublisher,
        &support,
        RCL_MS_TO_NS(50),
        timerPublisher_callback);

    rclc_timer_init_default(
        &timerMain,
        &support,
        RCL_MS_TO_NS(MAIN_LOOP_TIME_MS),
        timerMain_callback);

    // Setting executor
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
    
    // Enable logic converter
    gpio_pull_up(OE_PIN);
    gpio_put(OE_PIN, 1);

    // Setting I2C
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    //////////////////////////////////
    gpio_pull_up(TMP_RESET_PIN);
    //////////////////////////////////
    gpio_pull_up(MOTOR_RESET_PIN);
    gpio_pull_up(LINEAR_RESET_PIN);
    gpio_pull_up(STORAGE_RESET_PIN);

    // GND on Arduinos resets pins
    //////////////////////////////////
    gpio_put(TMP_RESET_PIN, 1);
    //////////////////////////////////
    //gpio_put(MOTOR_RESET_PIN, 0);
    //gpio_put(LINEAR_RESET_PIN, 0);
    //gpio_put(STORAGE_RESET_PIN, 0);

    // Wait for robust reset
    sleep_ms(50);  

    // Clear reset
    //////////////////////////////////
    gpio_put(TMP_RESET_PIN, 0);
    //////////////////////////////////
    gpio_put(MOTOR_RESET_PIN, 1);
    gpio_put(LINEAR_RESET_PIN, 1);
    gpio_put(STORAGE_RESET_PIN, 1);

    // Make sure the subsystem booted successfully
    sleep_ms(1000);

    // Subsystems init
    storage_init(&storage);
    linear_init(&linear);
    motor_init(&motor);
    
    // Green LED on, starting executor spinning
    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // Periodically check agent
        rcl_ret_t ping_ret = rmw_uros_ping_agent(timeout_ms, 1);
        if (ping_ret != RCL_RET_OK)
        {
            // Reset if agent is lost
            watchdog_enable(1, 1);
            while (true);
        }
    }
    return 0;
}


uint16_t drill_message(struct motor* motor, struct linear* linear, struct storage* storage)
{
    if(!motor || !linear || !storage)
        return 0;

    uint16_t storage_status = 0;
    uint16_t linear_status = 0;
    uint16_t motor_status = 0;

    //MOTOR
    if (motor->i2cStatus) motor_status |= (1 << 3);
    else motor_status &= ~(1 << 3); 

    if (motor->stucked) motor_status |= (1 << 2);
    else motor_status &= ~(1 << 2); 

    motor_status |= (motor->error & 0b11);

    //LINEAR
    if (linear->i2cStatus) linear_status |= (1 << 3);
    else linear_status &= ~(1 << 3); 

    linear_status |= (linear->error & 0b111);

    linear_status = linear_status << 4;

    //STORAGE
    if (storage->i2cStatus) storage_status |= (1 << 3);
    else storage_status &= ~(1 << 3); 

    if (storage->scaleTared) storage_status |= (1 << 2);
    else storage_status &= ~(1 << 2); 

    storage_status |= (storage->error & 0b11);

    storage_status = storage_status << 8;

    return storage_status | linear_status | motor_status;
}
