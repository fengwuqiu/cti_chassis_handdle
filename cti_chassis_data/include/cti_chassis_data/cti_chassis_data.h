/*
 * This file is part of cti_chassis_data.
 * Author : wangpeng
 * Date: 2019-12-12
 * describe: .h file for cti_chassis_data.cpp
 */
#ifndef CTI_CHASSIS_DATA_H
#define CTI_CHASSIS_DATA_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <deque>
#include <string>
#include <limits>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "cti_msgs/msg/battery_state.hpp"
#include "cti_msgs/msg/target_pose.hpp"
#include "cti_msgs/msg/range.hpp"
#include "cti_msgs/msg/battery_cell.hpp"
#include "cti_msgs/msg/battery_cells_state.hpp"
#include "cti_msgs/msg/vehicle_ctl_run_info.hpp"
#include "cti_msgs/msg/robot_version_display.hpp"
#include "cti_msgs/msg/tab_state.hpp"
#include "cti_msgs/msg/box_state.hpp"
#include "cti_msgs/msg/sins.hpp"
#include "cti_msgs/msg/state.hpp"
#include <Eigen/Dense>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"
#include "time.h"
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <iomanip>
#include <vector>
#include <pthread.h>
#include <std_msgs/msg/u_int16.hpp>
#include <cti_msgs/msg/auto_transmission.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cti_chassis_msgs/msg/navigation_log.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
//#include <cti_monitor/node_status_publisher.h>
#include <cti_chassis_msgs/msg/vehicle_state.hpp>
#include <cti_rblite_msgs/msg/box_ask_response.hpp>
#include <cti_rblite_msgs/msg/box_info.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cti_chassis_msgs/msg/ult_v30_datas.hpp>
#include <deque>

#define cmin(x,y) ((x<y)?(x):(y))
#define MAX_FILTER_NUM 4
#define PI 3.14159265358979323846
constexpr char const* kN = "fpga-data";
using namespace cti::log;

//--????????????????????????????????????
typedef struct {
    std::deque<uint16_t> data_buffer;
    float wight[MAX_FILTER_NUM];
    uint16_t data_output;
}ULT_DATA_ST;

class CtiFpgaData : public rclcpp::Node 
{
public:
    CtiFpgaData();
    void boxload_Callback(const cti_msgs::msg::State::SharedPtr msg);
    void rfidsingle_Callback(const cti_msgs::msg::BoxState::SharedPtr msg);
    void chassiserror_Callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    void navigationlog_Callback(const cti_chassis_msgs::msg::NavigationLog::SharedPtr msg);
    void chassisInfo_Callback(const cti_chassis_msgs::msg::VehicleState::SharedPtr msg);
    void boxinfo_Callback(const cti_rblite_msgs::msg::BoxAskResponse::SharedPtr msg);
    void initLog(const std::string name);
    void lin_ult_data_Callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    void lin_ult_data_v3_0_Callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    void dustbox_lin_ult_data_Callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    void new_lin_ult_data_v3_0_Callback(const cti_chassis_msgs::msg::UltV30Datas::SharedPtr msg);
    void kalman_init_v3_0();
    void kalman_filter_v3_0(uint16_t ult_data[] , int len);


    //--ros param
    rclcpp::Subscription<cti_msgs::msg::State>::SharedPtr boxload_sub;
    rclcpp::Subscription<cti_msgs::msg::BoxState>::SharedPtr rfid_sub;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr  chassis_error_sub;
    rclcpp::Subscription<cti_chassis_msgs::msg::NavigationLog>::SharedPtr navigation_log_sub;
    rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr  rfid_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr  box_exist_pub;
    rclcpp::Subscription<cti_chassis_msgs::msg::VehicleState>::SharedPtr chassis_info_sub;
    rclcpp::Subscription<cti_rblite_msgs::msg::BoxAskResponse>::SharedPtr box_info_sub;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr lin_ult_data_sub;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr lin_ult_data_v3_0_sub;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr dustbox_lin_ult_data_sub;
    rclcpp::Subscription<cti_chassis_msgs::msg::UltV30Datas>::SharedPtr new_lin_ult_data_v3_0_sub;
    

    //--rfid param
    std::vector<cti_msgs::msg::TabState> tabstate_rfid1; 
    std::vector<cti_msgs::msg::TabState> tabstate_rfid2;
    std::vector<cti_msgs::msg::TabState> tabstate_rfid3;
    std::vector<cti_msgs::msg::TabState> tabstate_rfid4;
    //--?????????2?????????RFID?????????????????????
    std::string boxloadid_old ;//????????????id??????
    int boxloadstatus_old ;//????????????????????????
    std::string rfid4th_old ;//?????????????????????RFID??????????????????
    std::string rfid4thid_old ;//????????????????????????
    //--??????rfid????????????
    double oldtime_rfid1 = 0;
    double oldtime_rfid2 = 0;
    double oldtime_rfid3 = 0;
    double oldtime_rfid4 = 0;
    bool recv_rfid1_timeout = false;
    bool recv_rfid2_timeout = false;
    bool recv_rfid3_timeout = false;
    bool recv_rfid4_timeout = false;
    //--rfid??????????????????
    uint8_t rfid1_empty_cnt;
    uint8_t rfid2_empty_cnt;
    uint8_t rfid3_empty_cnt;
    uint8_t rfid4_empty_cnt;
    //--rfid read lock
    uint8_t rfid2_read_lock;

    //--????????????
    cti_msgs::msg::BoxState rfid_all;//????????????????????????rfid??????????????????
    std_msgs::msg::UInt8 box_exist;
    //--???????????????????????????????????????
    typedef enum module_error_enum{
        motion_control_board = 1,
        left_front_driver = 2,
        right_front_driver = 3,
        left_rear_driver = 4,
        right_rear_driver = 5,
        front_turn_driver = 6,
        rear_turn_driver = 7,
        front_brake_driver = 8,
        rear_brake_driver = 9,        
        battery_board = 10,
    }module_error_t;
//????????????
    typedef enum{
        warning_level = 1,
        serious_level = 2,
        deadly_level  = 3
    }chassis_error_level_enum;
    //lin???????????????????????????
    typedef enum ult_type_enum{
        front_left_ult,
        front_right_ult,
        right_front_down_ult,
        right_front_up_ult,
        right_center_ult,
        rear_right_ult,
        rear_left_ult,
        left_center_ult,
        left_front_down_ult,
        left_front_up_ult,
        back_center_ult,
        unused,
        max_type_ult   
    }ult_type_t;
        //lin?????????????????????
    std::string ult_name[max_type_ult]={
        "front_left_ult",
        "front_right_ult",
        "right_front_down_ult",
        "right_front_up_ult",
        "right_center_ult",
        "rear_right_ult",
        "rear_left_ult",
        "left_center_ult",
        "left_front_down_ult",
        "left_front_up_ult",
        "back_center_ult",
        "unused"
    };

    //new_lin???????????????????????????_v3.0
    typedef enum new_ult_type_enum_v3_0{
        new_front_right_ult_v3_0,
        new_right_front_down_ult_v3_0,
        new_left_front_down_ult_v3_0,
        new_front_left_ult_v3_0,
        new_front_ult_v3_0,
        new_back_ult_v3_0,
        new_max_type_ult_v3_0
    }new_ult_type_enum_v3_0_t;

    //new_lin?????????????????????_v3.0
    std::string new_ult_name_v3_0[new_max_type_ult_v3_0]={
        "front_right_ult",
        "rear_right_down_ult",
        "left_front_down_ult",
        "front_left_ult",
        "front_center_ult",
        "back_center_ult"
    };


    //lin???????????????????????????_v3.0
    typedef enum ult_type_enum_v3_0{
        front_left_ult_v3_0,
        front_right_ult_v3_0,
        right_front_ult_v3_0,
        right_rear_ult_v3_0,
        rear_right_ult_v3_0,
        rear_left_ult_v3_0,
        left_rear_ult_v3_0,
        left_front_ult_v3_0,
        max_type_ult_v3_0   
    }ult_type_enum_v3_0_t;
    //lin?????????????????????_v3.0
    std::string ult_name_v3_0[max_type_ult_v3_0]={
        "front_left_ult_v3_0",
        "front_right_ult_v3_0",
        "right_front_ult_v3_0",
        "right_rear_ult_v3_0",
        "rear_right_ult_v3_0",
        "rear_left_ult_v3_0",
        "left_rear_ult_v3_0",
        "left_front_ult_v3_0"
    };

    //??????lin???????????????????????????
    typedef enum dustbox_ult_type_enum{
        dustbox_rear_right_ult,
        dustbox_rear_left_ult,
        dustbox_bottom_ult,
        dustbox_max_type_ult   
    }dustbox_ult_type_t;

    
    double VEHICLE_WIDTH;
    double REAR_ULT_DIST;
    double FRONT_ULT_DIST;
    double min_range;
    double max_range;
    double rear_ult_detect_offset;
    double rear_ult_detect_max;
    double rear_ult_detect_min;
    double front_ult_detect_offset;
    double front_ult_detect_max;
    double front_ult_detect_min;
    double vehicle_width;
    double rear_ult_dist;
    double front_ult_dist;
    ULT_DATA_ST ult_state[max_type_ult];




    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lin_ult_pub[max_type_ult];
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lin_ult_front_center_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lin_ult_rear_center_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_0;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_1;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_2;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_3;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lin_ult_before_kalman_v3_0_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lin_ult_after_kalman_v3_0_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr dustbox_ult_rear_center_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr dustbox_ult_bottom_pub;
    ULT_DATA_ST dustbox_ult_state[dustbox_max_type_ult];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ult_topic_names_pub;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lin_ult_v3_0_pub[max_type_ult_v3_0];
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lin_ult_front_center_v3_0_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr lin_ult_rear_center_v3_0_pub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr new_lin_ult_v3_0_pub[new_max_type_ult_v3_0];
     
private:
    uint8_t RFIDTIMEOUT_DUR;
    double RFID_EMPTY_LIMIT;
    int dustbox_LIN_ult_installed;
    double msg_max_range;


    //?????????????????????????????????
    typedef struct Kalman
    {
        float lastP; //??????????????????
        float nowP;  //??????????????????
        float x_hat; //????????????????????????????????????????????????
        float Kg;    //?????????????????????
        float Q;     //????????????
        float R;     //????????????
    }Kalman;
    std::vector<Kalman> ult_Kalman_list_v3_0_;
    bool kalman_enable_; //?????????????????????
    double kalman_R_;    //?????????????????????
    double kalman_Q_;    //??????????????????????????????????????????

    //std::shared_ptr<cti_monitor::NodeStatusPublisher> node_status_publisher_ptr_;
};

#endif
