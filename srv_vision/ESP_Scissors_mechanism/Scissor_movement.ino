#include <stdio.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int16.h>

rcl_node_t node;
rcl_timer_t timer;
rcl_publisher_t publisher;
rcl_allocator_t allocator;

rclc_support_t support;
rclc_executor_t executor;

std_msgs__msg__Int16 msg;

// Encoder
#define ENC_A 12
#define ENC_B 13

// PWM
#define IN1 25
#define IN2 26

// Micro Ros variables
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

// Servomotor
#define SERVO_PIN 27

// Interruptions variables
volatile bool Aset = 0;
volatile bool Bset = 0;

// Motor variables
int pos = 0;           // Motor position in degrees
int pwm = 30;          // PWM value (30 - 250) for motor movement
int max_pulses = 4560; // Pulses per revolution
float enc_res = 0.12;  // Encoder resolution

// Others
int revolutions = 0;
int current_pulse = 0;
float current_pos = 0;
float linear_vel = 0.0;

/* --- DC Motor Functions ---*/

void reset()
{

    current_pulse = 0;
    revolutions = 0;
    linear_vel = 0.0;
}

void stop()
{

    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
}

void turn_right()
{

    analogWrite(IN1, pwm);
    analogWrite(IN2, 0);
}

void turn_left()
{

    analogWrite(IN1, 0);
    analogWrite(IN2, pwm);
}

// Encoder interruption routine
void IRAM_ATTR Encoder()
{

    Aset = digitalRead(ENC_A);
    Bset = digitalRead(ENC_B);

    if (Bset == Aset)
    {

        current_pulse++;
    }
    else
    {

        current_pulse--;
    }

    pos = (current_pulse * 360) / max_pulses;
    current_pos = pos;

    if (current_pulse >= max_pulses || current_pulse <= -max_pulses)
    {
        revolutions++;
        current_pulse = 0;
    }
}

/* --- Micro Ros Functions ---*/

void error_loop()
{

    while (1)
    {

        stop();
        delay(100);
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);

    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

void setup()
{

    set_microros_transports();

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_A), Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), Encoder, CHANGE);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "scissor_movement", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "scissor_movement_pub"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;
}

void loop()
{

    delay(100);
}