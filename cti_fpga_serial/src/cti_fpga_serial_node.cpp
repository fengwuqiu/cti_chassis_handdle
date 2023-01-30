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
#include <geometry_msgs/msg/twist_stamped.hpp>
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
#include "cti_msgs/msg/rtcm.hpp"
#include "cti_fpga_serial/user_cmd.hpp"
#include "cti_fpga_serial_msgs/msg/update_info.hpp"
#include "cti_fpga_serial_msgs/msg/cmd_answer.hpp"
#include "cti_fpga_serial_msgs/msg/firm_ware_info.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <Eigen/Dense>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"
#include <sys/time.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <pthread.h>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cti_msgs/msg/auto_transmission.hpp>
#include <cti_fpga_serial_msgs/msg/navigation_log.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <cti_fpga_serial_msgs/msg/serial_status.hpp>
#include <cti_msgs/msg/gnss_rtk.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cti_msgs/msg/robot_localizer_state.hpp>
#include <cti_msgs/msg/dustbin_control.hpp>
#include <cti_msgs/msg/dustbin_state.hpp>
#include <std_msgs/msg/int8.hpp>
//#include <cti_monitor/node_status_publisher.h>
#include <cti_fpga_serial_msgs/msg/vehicle_state.hpp>
#include <std_msgs/msg/int64.hpp>
#include "cti_fpga_serial_msgs/msg/dustbinid_state.hpp"
#include "jsoncpp/json/json.h"
#include <cstring>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <cti_msgs/msg/dustbin_control_new.hpp>
#include <cti_msgs/msg/dustbin_state_new.hpp>
#include <cti_msgs/msg/dustbox_control.hpp>
#include <cti_msgs/msg/dustbox_state.hpp>
#include <cti_msgs/msg/data.hpp>
#include <cti_msgs/msg/data_array.hpp>
#include <cti_fpga_serial_msgs/msg/range_raw_data.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <queue>
#include <cti_fpga_serial_msgs/msg/ult_v30_datas.hpp>

using namespace std::chrono_literals;
#define DEBUG_PRINT 0
#define PI 3.1415926

// static std::shared_ptr<cti_monitor::NodeStatusPublisher>
// node_status_publisher_ptr_;
//*********************************** 全局变量声明
// thread of reading data
pthread_t th_serial_recv;
// KN:name in log
constexpr char const *kN = "fpga-serial";
using namespace cti::log;
#define cmax(x, y) ((x > y) ? (x) : (y))
#define cmin(x, y) ((x < y) ? (x) : (y))
#define PI 3.14159265358979323846
#define GRAV 9.7803
#define SWEEPER_ULT_FILTER_NUM 2
serial_frame_type user_frame = {0};

//通讯端口标志
#define PORT_SERIAL 1
#define PORT_UDP 2
int port_type = 0;
std::string port_type_name;
//串口通讯端口
std::string serial_port;
// udp通信端口
std::string udp_ip;
int udp_port;

std::string udp_ip_dest;
int udp_port_dest;

// std::string serial_port;
double car_wheel_base;
//--data buf
std::deque<serial_frame_include_id_type> databuf;
const unsigned int DATABUF_NUM_MAX = 5;
bool stm32_update_flag = false;
unsigned int TIMEOUT = 0;
uint32_t seq_num = 0;
//--里程计
double odom_x = 0, odom_x_4wd = 0;
double odom_y = 0, odom_y_4wd = 0;
double odom_th = 0, odom_th_4wd = 0;
static nav_msgs::msg::Odometry odom, odom_4wd;
//--IMU
static cti_msgs::msg::Sins sins_info;
static sensor_msgs::msg::Imu imu_info;
//--灯控
uint16_t light_type = 0;
//--路程
double mileage = 0;
//--电池电量
static cti_msgs::msg::BatteryState batteryState;
static cti_msgs::msg::BatteryCellsState BatCellsState;
//--版本信息
static cti_msgs::msg::RobotVersionDisplay RobotVersionDisplay;
//--超声波数据
static cti_msgs::msg::Range rangeDatas;
static sensor_msgs::msg::Range rangeData;
//--运行信息
static cti_msgs::msg::VehicleCtlRunInfo runinfo;
//--命令ID全局计数变量
uint16_t cmd_id_global = 0;
//--已发送命令的储存map，map的大小限制。
std::map<uint16_t, double> cmd_send_map;
const int SENDBUF_SIZE_MAXMUM = 1;
//--应答成功/失败的命令计数
uint32_t cmd_answer_success_cnt = 0;
uint32_t cmd_answer_fail_cnt = 0;
//--cmd send count
uint32_t cmd_withid_send_cnt = 0;
uint32_t cmd_withid_recv_cnt = 0;
//--消息应答情况
static cti_fpga_serial_msgs::msg::CmdAnswer cmd_answer_cnt;
//--timer3 计时器回调函数变量
uint8_t sendtime_loop_num = 50;
//--查询到版本标志位
bool get_version_flag = false;
//--顶升、锁箱、命令应答全局变量
int8_t global_up_down_flag = 0;
int8_t global_switch_flag = 0;
double cmd_answer_timeout = 0;
// position global temp
float global_pose[3] = {0};
float global_q[4] = {0};
// rfid 信息
cti_msgs::msg::BoxState rfid_all;  //一次发送车上所有rfid读卡器的信息
cti_msgs::msg::BoxState rfid_single;  //单个方式每一个raid的信息
std::vector<cti_msgs::msg::TabState> oldtabstate_rfid1;
std::vector<cti_msgs::msg::TabState> oldtabstate_rfid2;
std::vector<cti_msgs::msg::TabState> oldtabstate_rfid3;
std::vector<cti_msgs::msg::TabState> oldtabstate_rfid4;
double oldtime_rfid1 = 0;
double oldtime_rfid2 = 0;
double oldtime_rfid3 = 0;
double oldtime_rfid4 = 0;
bool recv_rfid1_timeout = false;
bool recv_rfid2_timeout = false;
bool recv_rfid3_timeout = false;
bool recv_rfid4_timeout = false;
const double RFIDTIMEOUT_DUR = 4;
//--通讯状态信息
uint8_t recvrfid_loop_num = 0;
uint16_t recv_rfid_cnt = 1;
uint16_t recv_rfid_cnt_old = 0;

uint8_t recvctrl_loop_num = 0;
uint16_t recv_ctrl_cnt = 1;
uint16_t recv_ctrl_cnt_old = 0;

uint8_t recvctrl_rate_loop_num = 0;
uint16_t recv_ctrl_rate_cnt = 1;
uint16_t recv_ctrl_rate_cnt_old = 0;

uint8_t recvult_loop_num = 0;
uint16_t recv_ult_cnt = 1;
uint16_t recv_ult_cnt_old = 0;

uint8_t recvcmd_loop_num = 0;
uint16_t recv_cmd_cnt = 1;
uint16_t recv_cmd_cnt_old = 0;

uint8_t recv_pthread_loop_num = 0;
uint16_t recv_pthread_cnt = 1;
uint16_t recv_pthread_cnt_old = 0;
uint16_t recv_pthread_crc_status = 0;  // crc校验，0：正确 1：错误

const double RECVTIMEOUT_DUR = 0.02;
const uint8_t RECV_CTRL_RATE_MIN =
    15;  //控制信息上传频率控制，少于15hz串口状态报错
//--通讯状态消息
static cti_fpga_serial_msgs::msg::SerialStatus serial_status;
uint16_t old_serial_status = 65535;
//-- 障碍物信息
int16_t global_k = 0;       //前障碍物信息
int16_t global_k_back = 0;  //后障碍物信息
//--激光对箱消息
static cti_msgs::msg::Rtcm box_laser;
//--底盘错误码消息
// static cti_fpga_serial_msgs::msg::ChassisErrorCode
// chassis_error;//提供给fpga_data_node使用,v3.7以后停用
//--底盘导航log消息
static cti_fpga_serial_msgs::msg::NavigationLog navigation_log;
//--气压计数据储存全局变量
float baro_raw_old = 0;
//--气压计原始数据值检测消息
static std_msgs::msg::UInt16 baro_status;
//--底盘错误码全局变量
uint8_t module_type_global = 0;
uint32_t module_error_code_global = 0;
uint8_t module_error_level_global = 0;
//--portIndex全局变量
uint8_t send_ctl_portIndex_cnt = 0;
uint8_t send_contraposition_portIndex_cnt = 0;
uint8_t send_poweroff_portIndex_cnt = 0;
uint8_t send_timenow_portIndex_cnt = 0;
//--sd卡格式化结果消息
static std_msgs::msg::UInt8 formatsdcard_result;
//--gps消息
static cti_msgs::msg::GnssRTK gps_data;
//--定位状态全局变量
static int32_t robotlocalizerstate_global = 0;
//--版本号全局变量
static int8_t control_version_right =
    0;  //运控版本兼容状态 0：运控版本可用; 1：运控版本过高; -1：运控版本过低;
static int max_control_board_version_head_ = 0;
static int min_control_board_version_head_ = 0;
static int max_control_board_version_mid_ = 0;
static int min_control_board_version_mid_ = 0;
static int max_control_board_version_end_ = 0;
static int min_control_board_version_end_ = 0;
//--ptp时间同步全局变量
bool need_resend_rough = false;
time_sync_rough_msg_type sync_rough_msg_saved;
bool check_rough_sync_res_timeoout;
uint8_t check_rough_sync_res_cnt;
uint8_t send_fine_sync_cnt = 0;
bool need_send_rough = true;
bool need_send_fine_sync = false;
//--清扫箱控制变量
bool check_clean_mechine_state;
uint8_t recv_clean_mechine_motor_status;
motion_to_dustbin_t resend_to_dustbin_cmd;
//--清扫车超声波消息
static sensor_msgs::msg::Range sweeperRangeData;
//--顶升测试标志位
bool lift_test_switch_ = false;
bool lift_test_flag = false;
uint16_t recv_lift_test_num = 0;
//--运控版本兼容状态消息
static std_msgs::msg::Int8 firmwareVersionCheck;  //
//--车上的箱子类型状态
int32_t box_type;
//--清扫箱箱号设置状态
dusbin_rf_set_state_t dustbin_set_state;
//--清扫箱在车上设置标志位
bool set_dustbin_on_car = false;  // false:未进行设置；true:设置中；
//--读取清扫箱id标志位
bool read_dustbin_id = false;
//--默认清扫箱箱号设置
int default_sweeper_id = 0;
//--清扫箱id设置接受
cti_fpga_serial_msgs::msg::DustbinidState dustbinidstate;
int pre_recv_dustbin_id;
//--发送到集尘箱命令
motion_to_dust_box_t send_to_dust_box_cmd;
//--发送到清扫箱命令
motion_to_dustbin_t send_to_dustbin_cmd;
//--环卫车清扫命令
motion_to_clean_t send_to_clean_cmd;
motion_to_clean_t resend_to_clean_cmd;
//--机器人类型
int robot_type = 0;
//--机器人版本号
std::string cti_run_ver;
//--默认的边刷伸展比例和速度..针对没有加这两个的旧结构体的兼容
int default_side_brush_transform;
int default_side_brush_speed;
//--是否安装的lin通信的超声波
int LIN_ult_installed;
//--dataarray类型控制话题的发送结构体
motion_to_clean_t_new send_to_clean_cmd_new;
//--存放水量计算的结构体
vehicle_water_box_status vehicle_water_status;
std::string config_file_path;
std::deque<uint8_t> vehicle_water_tank_top;
std::deque<uint8_t> vehicle_water_tank_bottom;
int max_vehicle_water_tank_restore = 6;
//--取消定位和障碍物对底盘速度限制的开关
bool localization_limit = true;
bool obstatle_disobs_limit = true;
//--通讯检测超时时间
double box_chat_timeout_secs;
double chassis_chat_timeout_secs;
chat_state chassis_chat_state;
chat_state box_chat_state;
int dustbox_lora_id = -1;  // 清扫箱设定id号, -1:取消id

//--需要构建控制指令的顺序和lin通讯的超声波序号之间的对应表
//在cpp文件中构建（-1,代表不存在）
int8_t linult_order_to_control[] = {2,  -1, 1, 11, -1, 7, 6,
                                    -1, 10, 9, 4,  3,  8, 5};
//----------------------------------------;
lin_ult_mode_union
    vehicle_defalt_linult_mode;  // 2号前右，3号右前下，6号后右，9号左前下
                                 // 箱子2号（后左）只收不发，其余即收也发
lin_ult_mode_union
    vehicle_all_stop_work_mode;  //关闭所有超声波，有效位置为0,无效位全部为1
ultmode_set_state vehicle_ult_set_state;
uint64_t vehicle_ult_cmd =
    0xffffffffffffffff;  //接收到的超声波控制命令用于打开，关闭单个超声波
uint8_t vehicle_ult_check_resend_time = 0;
uint8_t vehicle_ult_set_resend_time = 0;

lin_ult_mode_union
    dustbox_defalt_linult_mode;  // 2号前右，3号右前下，6号后右，9号左前下
                                 // 箱子2号（后左） 只收不发，其余即收也发
lin_ult_mode_union
    dustbox_all_stop_work_mode;  //关闭所有超声波，有效位置为0,无效位全部为1
ultmode_set_state dustbox_ult_set_state;
uint64_t dustbox_ult_cmd =
    0xffffffffffffffff;  //接收到的超声波控制命令用于打开，关闭单个超声波
uint8_t dustbox_ult_check_resend_time = 0;
uint8_t dustbox_ult_set_resend_time = 0;
double msg_max_range;

msg_light_cmd_3_0 light_cmd_v3_0;

int recv_data_cnt = 0;
int recv_ret = 0;
int recv_crc_status = 0;

battery_board_cmd_t send_battery_board;

//*********************************** ros发布器声明
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_4wd;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_calc;
rclcpp::Publisher<cti_fpga_serial_msgs::msg::UpdateInfo>::SharedPtr stm32_pub;
// rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr steer_pub;
// //2020年4月2日停用
rclcpp::Publisher<cti_msgs::msg::BatteryState>::SharedPtr battery_pub;
rclcpp::Publisher<cti_msgs::msg::BatteryCellsState>::SharedPtr batcell_pub;
// rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
// info_pub;//2020年4月2日停用
rclcpp::Publisher<cti_msgs::msg::VehicleCtlRunInfo>::SharedPtr ctlinfo_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
rclcpp::Publisher<cti_msgs::msg::RobotVersionDisplay>::SharedPtr firmvion_pub;
rclcpp::Publisher<cti_msgs::msg::Range>::SharedPtr ranges_pub;
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
    alt_3_0_pub[ULT_3_0_TYPE_NUM];
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub[max_type_ult];
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
    sweeper_range_pub[sweeper_max_type_ult];
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
    new_sweeper_range_pub[SWEEPER_ULT_MODULE_NUM][new_sweeper_max_type_ult];
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
    dustbox_range_pub[dustbox_max_type_ult];
rclcpp::Publisher<cti_fpga_serial_msgs::msg::RangeRawData>::SharedPtr
    raw_lin_range_pub;  // lin通信超声波原始数据发布
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr dustbox_bottom_range_pub;
rclcpp::Publisher<cti_fpga_serial_msgs::msg::CmdAnswer>::SharedPtr
    cmd_answer_pub;
// rclcpp::Publisher<cti_msgs::msg::Sins>::SharedPtr ult_ver_three_pub;
rclcpp::Publisher<cti_msgs::msg::Sins>::SharedPtr sins_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imudata_pub;
rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr boxrfid_pub_all;
rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr boxrfid_pub_single;
rclcpp::Publisher<cti_fpga_serial_msgs::msg::SerialStatus>::SharedPtr
    serial_status_pub;
rclcpp::Publisher<cti_msgs::msg::Rtcm>::SharedPtr
    box_laser_pub;  //激光对箱信息发布
rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr
    chassis_error_pub;  //底盘错误信息发布
rclcpp::Publisher<cti_fpga_serial_msgs::msg::NavigationLog>::SharedPtr
    navigation_log_pub;  //底盘重要信息上传
rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr
    baro_status_pub;  //气压计状态发布
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    formatsdcard_pub;  // SD卡格式化结果回复
rclcpp::Publisher<cti_msgs::msg::GnssRTK>::SharedPtr gps_pub;  // gps信息发布
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
    compass_pub;  // compass信息发布
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    node_status_pub;  //节点状态标志话题
rclcpp::Publisher<cti_msgs::msg::DustbinState>::SharedPtr
    dustbin_state_pub;  //清扫箱状态发布
rclcpp::Publisher<cti_msgs::msg::DustbinStateNew>::SharedPtr
    dustbin_state_pub_new;  //清扫箱状态发布,环卫车的清扫状态发布,新的消息类型
rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
    firmware_version_status_pub;  //运控版本兼容状态发布
rclcpp::Publisher<cti_fpga_serial_msgs::msg::FirmWareInfo>::SharedPtr
    firmware_version_check_pub;  //查询地盘固件版本信息的返回详细信息
rclcpp::Publisher<cti_fpga_serial_msgs::msg::VehicleState>::SharedPtr
    recv_chassis_info_pub;  //收到运控上传所有信息发布
rclcpp::Publisher<cti_msgs::msg::TabState>::SharedPtr
    set_dustbin_id_state_pub;  //清扫箱id设置状态发布
rclcpp::Publisher<cti_fpga_serial_msgs::msg::DustbinidState>::SharedPtr
    recv_dustbin_id_state_pub;  //接受到清扫箱id信息发布
rclcpp::Publisher<cti_msgs::msg::DustbinState>::SharedPtr
    dust_box_state_pub;  //集尘箱状态发布
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    dust_box_autopush_pub;  //集尘箱自动倒垃圾发布
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    dust_box_fanspeed_pub;  //集尘箱风机速度发布
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    dustbin_damboard_pub;  //清扫箱挡板状态
rclcpp::Publisher<cti_msgs::msg::BatteryState>::SharedPtr
    dustbox_wireless_charge_state_pub;  //集尘箱无线充电状态
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    boxlock_state_pub;  //顶升磁吸锁状态
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    dust_box_state_pub_json;  //集尘箱状态发布json类型
rclcpp::Publisher<cti_msgs::msg::DustboxState>::SharedPtr
    dust_box_state_pub_new;  //集尘箱状态发布,新的消息类型
rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
    rain_sensor_pub;  //雨水传感器数据发布
rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
    smart_trash_state_pub;  //智能垃圾箱状态发布

rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
    dust_box_state_info_pub;  // 垃圾箱状态发布 cti_msgs/DataArray类型
rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
    dust_vehicle_state_info_pub;  // 环卫车清扫状态发布 cti_msgs/DataArray类型

rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
    dustbox_5g_state_pub;  //吸尘箱5g状态发布
rclcpp::Publisher<cti_msgs::msg::BatteryCellsState>::SharedPtr
    dustbox_batterycell_pub;  //吸尘箱电池电量

rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
    chat_statue_pub;  //通讯状态发布(箱子和底盘)
rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
    lin_ult_data_pub;  // lin通信超声波原始数据发布
rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
    dustbox_rear_range_pub;  //吸尘箱超声波数据原始数据发布
rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
    lin_ult_data_v3_0_pub;  // lin通信超声波原始数据发布_3.0的车　
rclcpp::Publisher<cti_fpga_serial_msgs::msg::UltV30Datas>::SharedPtr
    new_lin_ult_data_v3_0_pub;  // 新lin通信超声波原始数据发布_3.0的车　

//************************************** 初始函数
static void SigsHandler(int sig) { rclcpp::shutdown(); }
void init_signal() {
  ::signal(SIGCHLD, SIG_IGN);      // SIGCHLD
  ::signal(SIGINT, SigsHandler);   // ^ + C
  ::signal(SIGTERM, SigsHandler);  // 请求中断
  ::signal(SIGKILL, SigsHandler);  // 强制中断
}
void initLog(const std::string name) {
  // set the logger file name, defaut is "logger.log"
  Logger::setDefaultLogger(name);
  // set log output mode {CoutOrCerr,File,Both}
  Logger::getLogger().setOutputs(Logger::Output::File);
  // set log level mode {Fata,Erro,Warn,Note,Info,Trac,Debu,Deta}
  Logger::getLogger().setLogLevel(LogLevel::Info);
  // enable display thread id , defaut is "false"
  Logger::getLogger().enableTid(false);
  // enable display line id number, defaut is "true"
  Logger::getLogger().enableIdx(true);
  // 1M  默认最小8*1024 默认最大sizeof(long)*32*1024*1024
  Logger::getLogger().setMaxSize(200 * 1024 * 1024);
}

//************************************** 由四元数计算yaw
double calcYawFromQuaternion(const tf2::Quaternion &q) {
  return tf2::impl::getYaw(q);
}
//************************************** 把结构体放进缓冲区
void pushData(serial_frame_include_id_type *data) {
  if (data != NULL) {
    if (databuf.size() > DATABUF_NUM_MAX) {
      // Info("CMD_DROP: "<<"id: "<< int(databuf.front().id) <<"cmd:
      // "<<int(databuf.front().cmd));
      databuf.pop_front();
    }
    //时间同步命令优先发送
    if (data->cmd == TIME_SYNC_FINE_CMD || data->cmd == TIME_SYNC_DELAY_CMD) {
      databuf.push_front(*data);
    } else {
      databuf.push_back(*data);
    }
  }
}

//************************************** 发送数据
void sendData(void) {
  if (cmd_send_map.empty() && !databuf.empty()) {
    if (databuf.front().need_id == 1) {
      double starttime = rclcpp::Clock().now().seconds();
      uint16_t id = databuf.front().id;
      std::pair<std::map<uint16_t, double>::iterator, bool> ret;
      ret = cmd_send_map.insert(std::pair<uint16_t, double>(id, starttime));
      if (ret.second == false) {
        // Info("CMD_EXITED: "<<"id: "<< int(ret.first->first));
      }
      cmd_withid_send_cnt++;
    }
    send_single_serial_frame(&(databuf.front().frame), 0);
    // Info("cmd_send_id: "<< int(databuf.front().id)<<"
    // cmd:"<<int(databuf.front().cmd));
    // printf("send id :%d\n",databuf.front().id);
    databuf.pop_front();
  }
}

//************************************** 16进制转string函数
std::string hex2string(uint8_t *data, unsigned int Len) {
  int i = 0;
  std::stringstream ss;
  for (i = 0; i < Len; i++) {
    ss << std::setw(2) << std::setfill('0') << std::hex << (int)data[i];
  }
  std::string str(ss.str());
  return str;
}

//************************************** 浅睡眠话题回调函数
void poweroff_cmd_type_callback(
    const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
  
  //int x=0b1001;
  send_to_poweroff_cmd_type_new send_to_poweroff_cmd_type_new_;
  send_to_poweroff_cmd_type_new_.cmd_id = cmd_id_global++;
  send_to_poweroff_cmd_type_new_.power_off_flag[0] = msg->data[0];
  send_to_poweroff_cmd_type_new_.power_control = msg->data[1];

  construct_serial_frame_ex(&user_frame, SEND_TO_POWEROFF_CMD,
                            sizeof(send_to_poweroff_cmd_type_new),
                            &send_to_poweroff_cmd_type_new_);

  serial_frame_include_id_type user_frame_include_id;
  user_frame_include_id.id = send_to_poweroff_cmd_type_new_.cmd_id;
  user_frame_include_id.cmd = SEND_TO_POWEROFF_CMD;
  user_frame_include_id.frame = user_frame;
  user_frame_include_id.need_id = 1;

  pushData(&user_frame_include_id);
}

//************************************** 软急停控制回调函数
void soft_stop_callback(const std_msgs::msg::Int8::SharedPtr msg) {
  send_battery_board.soft_stop = msg->data;

  construct_serial_frame_ex(&user_frame, SEND_BATTERY_CMD,
                            sizeof(send_battery_board), &send_battery_board);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_BATTERY_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  send_battery_board.soft_stop = 0;  //复位后为０
}

//************************************** 控制命令话题回调函数
void cmd_vel_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr twistStamped) {
  // node_status_publisher_ptr_->CHECK_RATE("/topic/cti_fpga_serial/cmd_vel",20,15,"topic
  // /cmd_vel rate is too slow");

  if (stm32_update_flag) {
    return;
  }
  if (control_version_right != 0) {
    Info("运控版本对应不上，超前或滞后");
    return;
  }
  double msgs_time = rclcpp::Time(twistStamped->header.stamp).seconds();
  double now_time = rclcpp::Clock().now().seconds();
  if ((now_time - msgs_time) > 1) {
    return;
  }
  rclcpp::Time start_time = rclcpp::Clock().now();
  send_to_control_cmd_type control_cmd;
  send_ctl_portIndex_cnt++;
  send_ctl_portIndex_cnt %= 255;
  control_cmd.portIndex = send_ctl_portIndex_cnt;
  control_cmd.linkCnt = 0;
  control_cmd.cmd_id = cmd_id_global++;
  control_cmd.linkFlag = 1;
  control_cmd.cmd_vel_Vx = twistStamped->twist.linear.x;
  // node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/cti_fpga_serial/control_cmd/vx",control_cmd.cmd_vel_Vx,4.5,5,"value
  // control_cmd:vx is too high");
  control_cmd.cmd_vel_Vy = twistStamped->twist.angular.y;
  control_cmd.cmd_vel_W = twistStamped->twist.angular.z;
  control_cmd.cmd_turn_mode = abs(twistStamped->twist.angular.x);
  control_cmd.cmd_break_flag = abs(twistStamped->twist.linear.z);
  control_cmd.switch_flag = global_switch_flag;
  control_cmd.up_down_flag = global_up_down_flag;
  control_cmd.light_type = light_type;
  control_cmd.pose[0] = global_pose[0];
  control_cmd.pose[1] = global_pose[1];
  control_cmd.pose[2] = global_pose[2];
  control_cmd.q[0] = global_q[0];
  control_cmd.q[1] = global_q[1];
  control_cmd.q[2] = global_q[2];
  control_cmd.q[3] = global_q[3];
  control_cmd.imu_flag = 10;
  control_cmd.front_obstacle_distance = global_k;
  control_cmd.rear_obstacle_distance = global_k_back;
  if (0 != robotlocalizerstate_global) {
    control_cmd.robot_status.bits.localizer = 1;
  } else {
    control_cmd.robot_status.bits.localizer = 0;
  }
  construct_serial_frame_ex(&user_frame, SEND_TO_CONTROL_CMD,
                            sizeof(control_cmd), &control_cmd);
  serial_frame_include_id_type user_frame_include_id;
  user_frame_include_id.id = control_cmd.cmd_id;
  user_frame_include_id.cmd = SEND_TO_CONTROL_CMD;
  user_frame_include_id.need_id = 1;
  user_frame_include_id.frame = user_frame;
  pushData(&user_frame_include_id);

  serial_status.send_localizer = control_cmd.robot_status.bits.localizer;

  Info("S_CT_PI: " << control_cmd.cmd_id
                   << " S_CT_VX: " << twistStamped->twist.linear.x
                   << " S_CT_VY: " << twistStamped->twist.linear.y
                   << " S_CT_TA: " << twistStamped->twist.angular.z
                   << " S_CT_TM: " << abs(twistStamped->twist.angular.x)
                   << " S_CT_LT: " << light_type << " S_CT_K: " << global_k
                   << " S_CT_KB: " << global_k_back
                   << " S_CT_BK: " << abs(twistStamped->twist.linear.z)
                   << " S_CT_LP: " << (int)global_up_down_flag
                   << " S_CT_LL: " << robotlocalizerstate_global
                   << " S_CT_BL: " << (int)control_cmd.switch_flag);
  static int send_loop_cnt = 0;
  send_loop_cnt++;
  if (send_loop_cnt >= 20) {
    Info(" S_CT_PS: " << global_pose[0] << " " << global_pose[1] << " "
                      << global_pose[2] << " S_CT_Q: " << global_q[0] << " "
                      << global_q[1] << " " << global_q[2] << " "
                      << global_q[3]);
    send_loop_cnt = 0;
  }
  // send light cmd for 3.0
  construct_serial_frame_ex(&user_frame, SEND_TO_LIGHT_CMD,
                            sizeof(light_cmd_v3_0), &light_cmd_v3_0);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_LIGHT_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
}

//************************************** 对箱命令话题回调函数
void boxContraposition_Callback(
    const cti_msgs::msg::TargetPose::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_control_contraposition_type control_contraposition_cmd;
  send_contraposition_portIndex_cnt++;
  send_contraposition_portIndex_cnt %= 255;
  control_contraposition_cmd.portIndex = send_contraposition_portIndex_cnt;
  double q_x = msg->pose.pose.orientation.x;
  double q_y = msg->pose.pose.orientation.y;
  double q_z = msg->pose.pose.orientation.z;
  double q_w = msg->pose.pose.orientation.w;
  tf2::Quaternion q(q_x, q_y, q_z, q_w);
  double q_yaw = calcYawFromQuaternion(q);
  //---
  if (msg->command == cti_msgs::msg::TargetPose::LOAD) {  //装箱
    control_contraposition_cmd.dock_flag = 1;
  } else if (msg->command == cti_msgs::msg::TargetPose::UNLOAD) {  //卸箱
    control_contraposition_cmd.dock_flag = 2;
  } else if (msg->command == cti_msgs::msg::TargetPose::LOAD + 10) {  //前方障碍
    control_contraposition_cmd.dock_flag = 3;
  } else {
    control_contraposition_cmd.dock_flag = 0;
  }

  control_contraposition_cmd.current_X = msg->pose.pose.position.x;
  control_contraposition_cmd.current_Y = msg->pose.pose.position.y;
  control_contraposition_cmd.current_Yaw = q_yaw;
  construct_serial_frame_ex(&user_frame, SEND_TO_CONTROL_CONTRAPOSITION,
                            sizeof(control_contraposition_cmd),
                            &control_contraposition_cmd);
  serial_frame_include_id_type user_frame_include_id;
  user_frame_include_id.id = 0;
  user_frame_include_id.cmd = SEND_TO_CONTROL_CONTRAPOSITION;
  user_frame_include_id.need_id = 0;
  user_frame_include_id.frame = user_frame;
  pushData(&user_frame_include_id);
}

//************************************** 升降箱命令话题回调函数
void boxUpDown_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (stm32_update_flag || lift_test_flag) {
    return;
  }
  global_up_down_flag = msg->data;
  static int old_data = std::numeric_limits<int>::min();
  if (msg->data != old_data) {
    old_data = msg->data;
  }
}

//************************************** 顶升测试升降命令回调
void lifttest_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  global_up_down_flag = msg->data;
  recv_lift_test_num %= 5000;
  recv_lift_test_num++;
  lift_test_flag = true;
}
//************************************** 锁箱命令话题回调函数
void boxLock_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  global_switch_flag = msg->data;  // 0：磁吸锁松开（销子弹出，箱子被锁住）
                                   // 1：磁吸锁吸合（销子下降，箱子不被锁住）
  static int old_data = std::numeric_limits<int>::min();
  if (msg->data != old_data) {
    old_data = msg->data;
  }
}

//************************************** 掉电命令话题回调函数
void powerOff_Callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_poweroff_cmd_type poweroff_cmd;
  // send_poweroff_portIndex_cnt++;
  // send_poweroff_portIndex_cnt %= 255;
  // poweroff_cmd.portIndex = send_poweroff_portIndex_cnt;
  poweroff_cmd.power_off_flag[0] = msg->data[0];
  poweroff_cmd.power_off_flag[1] = 0;  // msg->data[1];
  poweroff_cmd.power_off_flag[2] = 0;  // msg->data[2];
  poweroff_cmd.power_off_flag[3] = 0;  // msg->data[3];
  poweroff_cmd.cmd_id = cmd_id_global++;
  construct_serial_frame_ex(&user_frame, SEND_TO_POWEROFF_CMD,
                            sizeof(send_to_poweroff_cmd_type), &poweroff_cmd);
  serial_frame_include_id_type user_frame_include_id;
  user_frame_include_id.id = poweroff_cmd.cmd_id;
  user_frame_include_id.cmd = SEND_TO_POWEROFF_CMD;
  user_frame_include_id.frame = user_frame;
  user_frame_include_id.need_id = 1;
  pushData(&user_frame_include_id);
  static int old_data = std::numeric_limits<int>::min();
  // if(msg->data != old_data)
  // {
  //     old_data = msg->data;
  //     //Info("S_PW: "<<msg->data);
  // }
  Info("S_PW: " << msg->data[0]);
}

//************************************** 固件升级命令话题回调函数
void stmUpdate_Callback(
    const cti_fpga_serial_msgs::msg::UpdateInfo::SharedPtr msg) {
  TIMEOUT = 0;  //清空定时器
  if (msg->seq_num >= 1) {
    stm32_update_flag = true;
    send_update_serial_frame(msg->data.data(), msg->data.size());
  } else {
    stm32_update_flag = false;
    seq_num = 0;
  }
}
//************************************** 灯控命令话题回调函数
void lightType_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  light_type = msg->data;
}

//**************************************3.0 light control callback
void lightType_v3_0_Callback(const cti_msgs::msg::DataArray::SharedPtr msg) {
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "warning_light")
      light_cmd_v3_0.warning_light = atoi(info_msg.data.c_str());
    if (info_msg.name == "headlight_right")
      light_cmd_v3_0.headlight_right = atoi(info_msg.data.c_str());
    if (info_msg.name == "headlight_left")
      light_cmd_v3_0.headlight_left = atoi(info_msg.data.c_str());
    if (info_msg.name == "backlight_right")
      light_cmd_v3_0.backlight_right = atoi(info_msg.data.c_str());
    if (info_msg.name == "backlight_left")
      light_cmd_v3_0.backlight_left = atoi(info_msg.data.c_str());
    if (info_msg.name == "headlight_circle")
      light_cmd_v3_0.headlight_circle = atoi(info_msg.data.c_str());
    if (info_msg.name == "backlight_circle")
      light_cmd_v3_0.backlight_circle = atoi(info_msg.data.c_str());
    if (info_msg.name == "turn_light_right")
      light_cmd_v3_0.turn_light_right = atoi(info_msg.data.c_str());
    if (info_msg.name == "turn_light_left")
      light_cmd_v3_0.turn_light_left = atoi(info_msg.data.c_str());
    if (info_msg.name == "break_light")
      light_cmd_v3_0.break_light = atoi(info_msg.data.c_str());
    if (info_msg.name == "beep")
      light_cmd_v3_0.beep = atoi(info_msg.data.c_str());
  }
  // std::cout<<"in light_control_v3 callback:_________________________
  // "<<std::endl; std::cout<<"warning_light:
  // "<<(int)light_cmd_v3_0.warning_light<<std::endl;
  // std::cout<<"headlight_right:
  // "<<(int)light_cmd_v3_0.headlight_right<<std::endl;
  // std::cout<<"headlight_left:
  // "<<(int)light_cmd_v3_0.headlight_left<<std::endl;
  // std::cout<<"backlight_right:
  // "<<(int)light_cmd_v3_0.backlight_right<<std::endl;
  // std::cout<<"backlight_left:
  // "<<(int)light_cmd_v3_0.backlight_left<<std::endl;
  // std::cout<<"headlight_circle:
  // "<<(int)light_cmd_v3_0.headlight_circle<<std::endl;
  // std::cout<<"backlight_circle:
  // "<<(int)light_cmd_v3_0.backlight_circle<<std::endl;
  // std::cout<<"turn_light_right:
  // "<<(int)light_cmd_v3_0.turn_light_right<<std::endl;
  // std::cout<<"turn_light_left:
  // "<<(int)light_cmd_v3_0.turn_light_left<<std::endl; std::cout<<"break_light:
  // "<<(int)light_cmd_v3_0.break_light<<std::endl; std::cout<<"beep:
  // "<<(int)light_cmd_v3_0.beep<<std::endl;
}

//************************************** 位置命令话题回调函数
void position_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  global_pose[0] = msg->pose.position.x;
  global_pose[1] = msg->pose.position.y;
  global_pose[2] = msg->pose.position.z;
  global_q[0] = msg->pose.orientation.w;
  global_q[1] = msg->pose.orientation.x;
  global_q[2] = msg->pose.orientation.y;
  global_q[3] = msg->pose.orientation.z;
}

//************************************** 障碍物命令话题回调函数
void disObs_Callback(const cti_msgs::msg::AutoTransmission::SharedPtr msg) {
  if (obstatle_disobs_limit) {
    double msgs_time = msg->header.stamp.sec;
    double now_time = rclcpp::Clock().now().seconds();
    if ((now_time - msgs_time) > 1) {
      return;
    }
    global_k = (msg->k) * 100;
    global_k_back = (msg->k_back) * 100;
  } else {
    global_k = 10.0 * 100;
    global_k_back = 10.0 * 100;
  }
}

//************************************** 查询固件版本号命令话题回调函数
void checkversion_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_check_version_type check_version_cmd;
  update_info_type update_info_struct;
  update_info_struct.src = MODULE_CHECK_UPD;
  update_info_struct.dest = msg->data;
  check_version_cmd.upd_info = update_info_struct;
  check_version_cmd.check = 01;
  construct_serial_frame_ex(&user_frame, SEND_TO_CHECK_PROGRAM_VERSION,
                            sizeof(check_version_cmd), &check_version_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_CHECK_PROGRAM_VERSION;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
}

//************************************** 格式化SD卡命令话题回调函数
void formatsdcard_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_format_sd_card_cmd_type format_sdcard_cmd;
  format_sdcard_cmd.portIndex = msg->data;
  construct_serial_frame_ex(&user_frame, SEND_FORMAT_SD_CARD_CMD,
                            sizeof(format_sdcard_cmd), &format_sdcard_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_FORMAT_SD_CARD_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
}

//************************************** 清扫箱控制回调函数
void dustbinControl_Callback(
    const cti_msgs::msg::DustbinControl::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_dustbin_cmd.port = 0;
  send_to_dustbin_cmd.engin_start = msg->engin_start;
  send_to_dustbin_cmd.lift_motor = msg->lift_motor;
  send_to_dustbin_cmd.main_brush = msg->main_brush;
  send_to_dustbin_cmd.spray_motor = msg->spray_motor;
  send_to_dustbin_cmd.dust_suppresion = msg->dust_suppresion;
  send_to_dustbin_cmd.side_brush = msg->side_brush;
  send_to_dustbin_cmd.led = msg->led;
  send_to_dustbin_cmd.control_mode = 2;  // 2：导航控制
  if (4 == box_type) {
    send_to_dustbin_cmd.dustbin_on_car = 1;  //清扫箱在车上
  } else {
    send_to_dustbin_cmd.dustbin_on_car = 0;
  }

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(send_to_dustbin_cmd), &send_to_dustbin_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DBI_ES: " << (int)msg->engin_start
                    << " S_DBI_LM: " << (int)msg->lift_motor
                    << " S_DBI_MB: " << (int)msg->main_brush
                    << " S_DBI_SM: " << (int)msg->spray_motor
                    << " S_DBI_DS: " << (int)msg->dust_suppresion
                    << " S_DBI_SB: " << (int)msg->side_brush
                    << " S_DBI_LED: " << (int)msg->led << " S_DBI_DOC: "
                    << (int)send_to_dustbin_cmd.dustbin_on_car);
}

//************************************** 环卫车清扫控制回调函数
void cleanControl_Callback(const cti_msgs::msg::DustbinControl::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  serial_status.callback_dustbin_cnt_old_cnt++;
  serial_status.callback_dustbin_cnt_old_cnt %= 32000;
  send_to_clean_cmd.port = 0;
  send_to_clean_cmd.engin_start = msg->engin_start;
  send_to_clean_cmd.lift_motor = msg->lift_motor;
  send_to_clean_cmd.main_brush = msg->main_brush;
  send_to_clean_cmd.spray_motor = msg->spray_motor;
  send_to_clean_cmd.dust_suppresion = msg->dust_suppresion;
  send_to_clean_cmd.side_brush = msg->side_brush;
  send_to_clean_cmd.led = msg->led;
  send_to_clean_cmd.control_mode = 2;// ：导航控制
  if (4 == box_type) {
    send_to_clean_cmd.dustbin_on_car = 1;  //清扫箱在车上
  } else {
    send_to_clean_cmd.dustbin_on_car = 0;
  }
  if (msg->engin_start == 1) {
    //如果是开启清扫
    if (default_side_brush_transform >= 0 &&
        default_side_brush_transform <= 100) {
      send_to_clean_cmd.unused0 = default_side_brush_transform;
    }
    if (default_side_brush_speed >= 0 && default_side_brush_speed <= 100) {
      send_to_clean_cmd.unused1 = default_side_brush_speed;
    }
  }
  if (msg->engin_start == 0) {
    //如果是开启清扫
    send_to_clean_cmd.unused0 = 0;
    send_to_clean_cmd.unused1 = 0;
  }

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(send_to_clean_cmd), &send_to_clean_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_CC_ES: " << (int)msg->engin_start
                   << " S_CC_LM: " << (int)msg->lift_motor
                   << " S_CC_MB: " << (int)msg->main_brush
                   << " S_CC_SM: " << (int)msg->spray_motor
                   << " S_CC_DS: " << (int)msg->dust_suppresion << " S_CC_SB: "
                   << (int)msg->side_brush << " S_CC_LED: " << (int)msg->led
                   << " S_CC_DOC: " << (int)send_to_dustbin_cmd.dustbin_on_car);
}
void cleanControlNew_Callback(
    const cti_msgs::msg::DustbinControlNew::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  //测试模式　定时过滤１０分钟数据
  if (msg->control_mode == process_work_mode::process_work_mode_test) {
    process_twork_mode_t.cur_cnt =
        process_work_mode::PROCESS_WORK_MODE_INTERVAL;
    process_twork_mode_t.process_work_mode_enum_ =
        process_work_mode::process_work_mode_test;
  }
  // else if (msg->control_mode == process_work_mode::process_work_mode_normal) {
  //   process_twork_mode_t.process_work_mode_enum_ =
  //       process_work_mode::process_work_mode_normal;
  // }

  serial_status.callback_dustbin_cnt_new_cnt++;
  serial_status.callback_dustbin_cnt_new_cnt %= 32000;
  motion_to_clean_t_new send_to_clean_cmd_new;
  send_to_clean_cmd_new.port = 0;

  //借用灯的位置测下水泵
  send_to_clean_cmd_new.lift_pump_switch = msg->led;
  send_to_clean_cmd_new.shake_dust_motor_speed = msg->dust_suppresion;
  //借用灯的位置测下水泵 集尘电机

  send_to_clean_cmd_new.engin_start = msg->engin_start;
  send_to_clean_cmd_new.side_brush = msg->side_brush;
  send_to_clean_cmd_new.main_brush = msg->main_brush;
  send_to_clean_cmd_new.spray_motor = msg->spray_motor;
  //send_to_clean_cmd_new.dust_suppresion = msg->dust_suppresion;
  send_to_clean_cmd_new.lift_motor = msg->lift_motor;
  // send_to_clean_cmd_new.led = msg->led;
  send_to_clean_cmd_new.control_mode = msg->control_mode;  // 2：导航控制
  if (4 == box_type) {
    send_to_clean_cmd.dustbin_on_car = 1;  //清扫箱在车上
  } else {
    send_to_clean_cmd.dustbin_on_car = 0;
  }
  send_to_clean_cmd_new.dam_board = msg->dam_board;
  send_to_clean_cmd_new.side_brush_transform = msg->side_brush_transform;
  send_to_clean_cmd_new.side_brush_speed = msg->side_brush_speed;
  send_to_clean_cmd_new.unused1 = msg->unused1;
  send_to_clean_cmd_new.unused2 = msg->unused2;
  send_to_clean_cmd_new.unused3 = msg->unused3;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(send_to_clean_cmd_new),
                            &send_to_clean_cmd_new);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_CC_ES: " << (int)msg->engin_start
                   << " S_CC_LM: " << (int)msg->lift_motor
                   << " S_CC_MB: " << (int)msg->main_brush
                   << " S_CC_SM: " << (int)msg->spray_motor
                   << " S_CC_DS: " << (int)msg->dust_suppresion << " S_CC_SB: "
                   << (int)msg->side_brush << " S_CC_LED: " << (int)msg->led
                   << " S_CC_DOC: " << (int)send_to_clean_cmd_new.dustbin_on_car
                   << " S_CC_DB: " << (int)msg->dam_board
                   << " S_CC_SBT: " << (int)msg->side_brush_transform
                   << " S_CC_SBS: " << (int)msg->side_brush_speed
                   << " S_CC_UN1: " << (int)msg->unused1 << " S_CC_UN2: "
                   << (int)msg->unused2 << " S_CC_UN3: " << (int)msg->unused3);
}

//
void cleanControlInfo_Callback(const cti_msgs::msg::DataArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  //测试模式下不受到其他上游控制
  if (process_twork_mode_t.process_work_mode_enum_ ==
      process_work_mode::process_work_mode_test) {
    return;
  }

  //数据解析
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "engin_start")
      //一键开启 uint8_t  1    0
      send_to_clean_cmd_new.engin_start = atoi(info_msg.data.c_str());
    else if (info_msg.name == "spray_motor")
    //喷水电机 uint8_t 1/0    0
    {
      if (atoi(info_msg.data.c_str()) != 0) {
        send_to_clean_cmd_new.spray_motor = 8;
      } else {
        send_to_clean_cmd_new.spray_motor = 0;
      }
    } else if (info_msg.name == "sidebrush_lift")
      //边刷升降 int8_t 1   0
      send_to_clean_cmd_new.lift_motor = atoi(info_msg.data.c_str());
    else if (info_msg.name == "led")
      // led灯 uint8_t 1   0
      send_to_clean_cmd_new.led = atoi(info_msg.data.c_str());
    else if (info_msg.name == "dam_board")
      //挡板控制 uint8_t 0   0
      send_to_clean_cmd_new.dam_board = atoi(info_msg.data.c_str());
    else if (info_msg.name == "sidebrush_transform")
      //边刷伸展 uint8_t 100   0
      send_to_clean_cmd_new.side_brush_transform = atoi(info_msg.data.c_str());
    else if (info_msg.name == "sidebrush_speed")
      //边刷转速 uint8_t 100  0
      send_to_clean_cmd_new.side_brush_speed = atoi(info_msg.data.c_str());
    else if (info_msg.name == "decorate_light")
      //装饰灯 uint8_t 0:关 1:开
      send_to_clean_cmd_new.decorate_light = atoi(info_msg.data.c_str());
    else if (info_msg.name == "lift_motor")
      //边刷升降 int8_t 0:停止 1:上升 -1：下降
      send_to_clean_cmd_new.lift_motor = atoi(info_msg.data.c_str());
    else if (info_msg.name == "control_mode")
      //
      send_to_clean_cmd_new.control_mode = atoi(info_msg.data.c_str());
    else if (info_msg.name == "lift_pump_switch")
      //水泵开关
      send_to_clean_cmd_new.lift_pump_switch = atoi(info_msg.data.c_str());
    else if (info_msg.name == "shake_dust_motor_speed")
      //震尘速度百分比
      send_to_clean_cmd_new.shake_dust_motor_speed = atoi(info_msg.data.c_str());
    else
      continue;
  }

  send_to_clean_cmd_new.port = 0;
  send_to_clean_cmd_new.side_brush = 0;
  send_to_clean_cmd_new.main_brush = 0;
  send_to_clean_cmd_new.dust_suppresion = 0;
  // send_to_clean_cmd_new.control_mode = 2;  // 2：导航控制
  if (4 == box_type) {
    send_to_clean_cmd.dustbin_on_car = 1;  //清扫箱在车上
  } else {
    send_to_clean_cmd.dustbin_on_car = 0;
  }
  send_to_clean_cmd_new.unused1 = 0;
  send_to_clean_cmd_new.unused2 = 0;
  send_to_clean_cmd_new.unused3 = 0;

  send_to_clean_cmd.port = 0;
  send_to_clean_cmd.engin_start = send_to_clean_cmd_new.engin_start;
  send_to_clean_cmd.lift_motor = send_to_clean_cmd_new.lift_motor;
  send_to_clean_cmd.main_brush = 0;
  send_to_clean_cmd.spray_motor = send_to_clean_cmd_new.spray_motor;
  send_to_clean_cmd.dust_suppresion = 0;
  send_to_clean_cmd.side_brush = 0;
  send_to_clean_cmd.led = send_to_clean_cmd_new.led;
  send_to_clean_cmd.dam_board = send_to_clean_cmd_new.dam_board;
  // send_to_clean_cmd.control_mode = 2;  // 2：导航控制
  if (4 == box_type) {
    send_to_clean_cmd.dustbin_on_car = 1;  //清扫箱在车上
  } else {
    send_to_clean_cmd.dustbin_on_car = 0;
  }

  std::string clean_flag = "v6.0";
  std::string::size_type idx = cti_run_ver.find(clean_flag);
  if (idx != std::string::npos) {
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                              sizeof(send_to_clean_cmd), &send_to_clean_cmd);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
  } else {
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                              sizeof(send_to_clean_cmd_new),
                              &send_to_clean_cmd_new);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
  }

  Info("S_CC_ES: "
       << (int)send_to_clean_cmd_new.engin_start
       << " S_CC_LM: " << (int)send_to_clean_cmd_new.lift_motor
       << " S_CC_MB: " << (int)send_to_clean_cmd_new.main_brush
       << " S_CC_SM: " << (int)send_to_clean_cmd_new.spray_motor
       << " S_CC_DS: " << (int)send_to_clean_cmd_new.dust_suppresion
       << " S_CC_SB: " << (int)send_to_clean_cmd_new.side_brush
       << " S_CC_LED: " << (int)send_to_clean_cmd_new.led
       << " S_CC_DOC: " << (int)send_to_clean_cmd_new.dustbin_on_car
       << " S_CC_DB: " << (int)send_to_clean_cmd_new.dam_board
       << " S_CC_SBT: " << (int)send_to_clean_cmd_new.side_brush_transform
       << " S_CC_SBS: " << (int)send_to_clean_cmd_new.side_brush_speed
       << " S_CC_DE_LG: " << (int)send_to_clean_cmd_new.decorate_light
       << " S_CC_UN1: " << (int)send_to_clean_cmd_new.unused1
       << " S_CC_UN2: " << (int)send_to_clean_cmd_new.unused2
       << " S_CC_UN3: " << (int)send_to_clean_cmd_new.unused3);
}

//************************************** ３。０喷水控制
void sprayControl_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  send_to_clean_cmd_new.engin_start = 0;
  send_to_clean_cmd_new.lift_motor = 0;
  send_to_clean_cmd_new.main_brush = 0;
  send_to_clean_cmd_new.dust_suppresion = 0;
  send_to_clean_cmd_new.side_brush = 0;
  send_to_clean_cmd_new.led = 0;
  send_to_clean_cmd_new.dustbin_on_car = 0;
  send_to_clean_cmd_new.dam_board = 0;
  send_to_clean_cmd_new.side_brush_transform = 0;
  send_to_clean_cmd_new.side_brush_speed = 0;
  send_to_clean_cmd_new.decorate_light = 0;
  send_to_clean_cmd_new.unused1 = 0;
  send_to_clean_cmd_new.unused2 = 0;
  send_to_clean_cmd_new.unused3 = 0;

  //0x00// 关闭喷水
  //0x01// 后端球头雷达
  //0x02// 超声波满溢探头清洗
  //0x04// 头顶16线雷达zx
  //0x08// 抑尘喷头
  //0x10// 前端球头雷达
  send_to_clean_cmd_new.spray_motor = msg->data;
  send_to_clean_cmd_new.control_mode = 2;  // 2：一般控制模式

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(send_to_clean_cmd_new),
                            &send_to_clean_cmd_new);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
}

//************************************** 挡板控制回调函数
void dustbinDamboard_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_clean_cmd.dam_board = msg->data;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(send_to_clean_cmd), &send_to_clean_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_CC_DB: " << (int)msg->data);
}

//************************************** 边刷伸缩控制函数
void dustbinSideBrushTrans_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_clean_cmd.unused0 = msg->data;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(send_to_clean_cmd), &send_to_clean_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_CC_SBT: " << (int)msg->data);
}

//************************************** 智能垃圾箱控制回调函数
void smartTrash_Callback(const cti_msgs::msg::DataArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }

  smart_trash_cmd send_to_smart_trash;
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "lift_mode")
      // 1:上升 2：下降：3:急停  uint8_t
      send_to_smart_trash.lift_mode = atoi(info_msg.data.c_str());
    else if (info_msg.name == "reaction_control")
      // 1:关闭感应开盖 0:允许感应开盖
      send_to_smart_trash.reaction_control = atoi(info_msg.data.c_str());
    else
      continue;
  }
  send_to_smart_trash.unuse2 = 0;
  send_to_smart_trash.unuse3 = 0;
  send_to_smart_trash.unuse4 = 0;

  construct_serial_frame_ex(&user_frame, SMART_TRASH_SEND,
                            sizeof(send_to_smart_trash), &send_to_smart_trash);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SMART_TRASH_SEND;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_ST_LM: " << (int)send_to_smart_trash.lift_mode);
}

//************************************** 吸尘箱5g状态查询回调函数
void dustbox5GCheck_Callback(const cti_msgs::msg::DataArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }

  dustbox_5g_check_cmd_t dustbox_5g_check;
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "port")
      dustbox_5g_check.port = atoi(info_msg.data.c_str());
    else if (info_msg.name == "msg1")
      dustbox_5g_check.msg1 = atoi(info_msg.data.c_str());
    else if (info_msg.name == "unused1")
      dustbox_5g_check.unused1 = atoi(info_msg.data.c_str());
    else if (info_msg.name == "unused2")
      dustbox_5g_check.unused2 = atoi(info_msg.data.c_str());
    else
      continue;
  }

  construct_serial_frame_ex(&user_frame, DUST_BOX_5G_CHECK_CMD,
                            sizeof(dustbox_5g_check), &dustbox_5g_check);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = DUST_BOX_5G_CHECK_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DOB_5G_CH_PO: " << (int)dustbox_5g_check.port << "S_DOB_5G_CH_MSG1: "
                          << (int)dustbox_5g_check.msg1);
}

//************************************** 大屏控制回调函数
void ledshow_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  Info("LED: " << (int)msg->data);
}
//************************************** 集尘箱控制回调函数
void dustBoxControl_Callback(
    const cti_msgs::msg::DustbinControl::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  serial_status.callback_dustbox_cnt_old_cnt++;
  serial_status.callback_dustbox_cnt_old_cnt %= 32000;
  send_to_dust_box_cmd.port = 0;
  send_to_dust_box_cmd.engin_start = msg->engin_start;
  send_to_dust_box_cmd.lift_motor = msg->lift_motor;
  send_to_dust_box_cmd.main_brush = msg->main_brush;
  send_to_dust_box_cmd.spray_motor = msg->spray_motor;
  send_to_dust_box_cmd.dust_suppresion = msg->dust_suppresion;
  send_to_dust_box_cmd.side_brush = msg->side_brush;
  send_to_dust_box_cmd.led = msg->led;
  send_to_dust_box_cmd.control_mode = 2;  // 2：导航控制

  construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD,
                            sizeof(send_to_dust_box_cmd),
                            &send_to_dust_box_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DBO_ES: " << (int)msg->engin_start);
}

//**************************************
//集尘箱控制回调函数,新的消息类型,把控制信息集成到一起
void dustBoxControlNew_Callback(
    const cti_msgs::msg::DustboxControl::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  serial_status.callback_dustbox_cnt_new_cnt++;
  serial_status.callback_dustbox_cnt_new_cnt %= 32000;
  //结构体并没有改变只是把风机速度和风机启动都放在了这里
  send_to_dust_box_cmd.port = 0;
  send_to_dust_box_cmd.engin_start = msg->engin_start;
  send_to_dust_box_cmd.lift_motor = msg->lift_motor;
  send_to_dust_box_cmd.main_brush = msg->main_brush;
  send_to_dust_box_cmd.spray_motor = msg->spray_motor;
  send_to_dust_box_cmd.dust_suppresion = msg->dust_suppresion;
  send_to_dust_box_cmd.side_brush = msg->side_brush;
  send_to_dust_box_cmd.led = msg->led;
  send_to_dust_box_cmd.control_mode = 2;  // 2：导航控制
  send_to_dust_box_cmd.fan_speed = msg->fan_speed;
  send_to_dust_box_cmd.unused0 = msg->unused0;
  send_to_dust_box_cmd.unused1 = msg->unused1;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD,
                            sizeof(send_to_dust_box_cmd),
                            &send_to_dust_box_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DBO_ES: " << (int)msg->engin_start
                    << "S_DBO_FS: " << (int)msg->fan_speed);
}

//解析 cti_msgs::msg::Data消息类型
parsed_ChassisInfo_t parse_chassisInfo(cti_msgs::msg::Data msg) {
  parsed_ChassisInfo_t parsed_data;
  switch (msg.type) {
    case cti_msgs::msg::Data::TYPE_UINT8:
      parsed_data.uint8_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_INT8:
      parsed_data.int8_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_UINT16:
      parsed_data.uint16_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_INT16:
      parsed_data.int16_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_UINT32:
      parsed_data.uint32_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_INT32:
      parsed_data.int32_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_UINT64:
      parsed_data.uint64_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_INT64:
      parsed_data.int64_data = atoi(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_FLOAT:
      parsed_data.float_data = atof(msg.data.c_str());
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_DOUBLE:
      parsed_data.double_data = strtod(msg.data.c_str(), NULL);
      return parsed_data;
      break;
    case cti_msgs::msg::Data::TYPE_STRING:
      parsed_data.string_data = msg.data;
      return parsed_data;
      break;
    default:
      break;
  }
}

//**************************************
//集尘箱控制回调函数,cti_msgs::msg::DataArray 消息类型
void dustBoxControlInfo_Callback(
    const cti_msgs::msg::DataArray::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  //数据解析
  for (int i = 0; i < msg->datas.size(); i++) {
    cti_msgs::msg::Data info_msg = msg->datas[i];
    if (info_msg.name == "engin_start")
      //吸尘箱启动 uint8_t
      send_to_dust_box_cmd.engin_start = atoi(info_msg.data.c_str());
    else if (info_msg.name == "fanspeed")
      //风机速度  uint8_t
      send_to_dust_box_cmd.fan_speed = atoi(info_msg.data.c_str());
    else if (info_msg.name == "led")
      // led灯 uint8_t
      send_to_dust_box_cmd.led = atoi(info_msg.data.c_str());
    else
      continue;
  }
  send_to_dust_box_cmd.control_mode = 2;  // 2：导航控制
  send_to_dust_box_cmd.port = 0;
  send_to_dust_box_cmd.lift_motor = 0;
  send_to_dust_box_cmd.main_brush = 0;
  send_to_dust_box_cmd.spray_motor = 0;
  send_to_dust_box_cmd.dust_suppresion = 0;
  send_to_dust_box_cmd.side_brush = 0;
  send_to_dust_box_cmd.unused0 = 0;
  send_to_dust_box_cmd.unused1 = 0;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD,
                            sizeof(send_to_dust_box_cmd),
                            &send_to_dust_box_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DBO_ES: " << (int)send_to_dust_box_cmd.engin_start
                    << "S_DBO_FS: " << (int)send_to_dust_box_cmd.fan_speed);
}

//************************************** 集尘箱自动倒垃圾控制回调函数
void dustBoxAutopushControl_Callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_dust_box_cmd.auto_push = msg->data;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD,
                            sizeof(send_to_dust_box_cmd),
                            &send_to_dust_box_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DBO_AP: " << (int)msg->data);
}
//************************************** 集尘箱风机速度控制回调函数
void dustBoxFanSpeed_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  send_to_dust_box_cmd.fan_speed = msg->data;

  construct_serial_frame_ex(&user_frame, SEND_TO_DUST_BOX_CMD,
                            sizeof(send_to_dust_box_cmd),
                            &send_to_dust_box_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUST_BOX_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("S_DBO_FS: " << (int)msg->data);
}
//************************************** 清扫箱控制(带重发和超时)回调函数
void dustbinControlResend_Callback(
    const cti_msgs::msg::DustbinControl::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  check_clean_mechine_state = true;

  resend_to_dustbin_cmd.port = 0;
  resend_to_dustbin_cmd.engin_start = msg->engin_start;
  resend_to_dustbin_cmd.lift_motor = msg->lift_motor;
  resend_to_dustbin_cmd.main_brush = msg->main_brush;
  resend_to_dustbin_cmd.spray_motor = msg->spray_motor;
  resend_to_dustbin_cmd.dust_suppresion = msg->dust_suppresion;
  resend_to_dustbin_cmd.side_brush = msg->side_brush;
  resend_to_dustbin_cmd.led = msg->led;
  resend_to_dustbin_cmd.control_mode = 2;  // 2：导航控制

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(resend_to_dustbin_cmd),
                            &resend_to_dustbin_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("RS_DBI_ES: " << (int)msg->engin_start
                     << " RS_DBI_LM: " << (int)msg->lift_motor
                     << " RS_DBI_MB: " << (int)msg->main_brush
                     << " RS_DBI_SM: " << (int)msg->spray_motor
                     << " RS_DBI_DS: " << (int)msg->dust_suppresion
                     << " RS_DBI_SB: " << (int)msg->side_brush
                     << " RS_DBI_LED: " << (int)msg->led << " RS_DBI_DOC: "
                     << (int)send_to_dustbin_cmd.dustbin_on_car);
}

//************************************** 环卫车清扫控制(带重发和超时)回调函数
void cleanControlResend_Callback(
    const cti_msgs::msg::DustbinControl::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  check_clean_mechine_state = true;

  resend_to_clean_cmd.port = 0;
  resend_to_clean_cmd.engin_start = msg->engin_start;
  resend_to_clean_cmd.lift_motor = msg->lift_motor;
  resend_to_clean_cmd.main_brush = msg->main_brush;
  resend_to_clean_cmd.spray_motor = msg->spray_motor;
  resend_to_clean_cmd.dust_suppresion = msg->dust_suppresion;
  resend_to_clean_cmd.side_brush = msg->side_brush;
  resend_to_clean_cmd.led = msg->led;
  resend_to_clean_cmd.control_mode = 2;  // 2：导航控制

  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(resend_to_clean_cmd), &resend_to_clean_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  Info("RS_CC_ES: " << (int)msg->engin_start
                    << " RS_CC_LM: " << (int)msg->lift_motor
                    << " RS_CC_MB: " << (int)msg->main_brush
                    << " RS_CC_SM: " << (int)msg->spray_motor
                    << " RS_CC_DS: " << (int)msg->dust_suppresion
                    << " RS_CC_SB: " << (int)msg->side_brush
                    << " RS_CC_LED: " << (int)msg->led << " RS_CC_DOC: "
                    << (int)send_to_dustbin_cmd.dustbin_on_car);
}

//************************************** 车上箱子状态回调函数
void boxtype_Callback(const std_msgs::msg::Int32::SharedPtr msg) {
  set_dustbin_on_car = true;
  box_type = msg->data;
  Info("CB_BT: " << (int)box_type);
}

//**************************************清扫箱id设定命令回调函数
void sweeperBoxID_Callback(const std_msgs::msg::Int64::SharedPtr msg) {
  dustbox_lora_id = msg->data;
  if (stm32_update_flag) {
    return;
  }
  Info("CB_SID: " << (int)msg->data);
  if (msg->data == -1) {             //取消配对
    if (pre_recv_dustbin_id != 1) {  //没有取消成功才发送
      if (pre_recv_dustbin_id != -1) {
        dustbin_set_state.reset();
        dustbin_set_state.if_send = true;
      }
    }
  } else if (msg->data == -2) {  //读取id
    dustbin_rf_set_cmd_t send_set_dustbin_id;
    send_set_dustbin_id.rw = 0;
    send_set_dustbin_id.baud =
        -1;  //一定要设置成-1 否则会修改运控和lora通信的波特率

    construct_serial_frame_ex(&user_frame, DUSBIN_RF_SET_READ_CMD,
                              sizeof(send_set_dustbin_id),
                              &send_set_dustbin_id);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = DUSBIN_RF_SET_READ_CMD;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
    read_dustbin_id = true;
    Info("S_SID:" << (int)msg->data);
  } else {
    //如果收到的id和已经设定的不同才设置
    if (pre_recv_dustbin_id != msg->data) {
      read_dustbin_id = false;
      dustbin_set_state.reset();
      dustbin_set_state.if_send = true;
      if (default_sweeper_id != 0) {
        dustbin_set_state.send_id = default_sweeper_id;
      } else {
        dustbin_set_state.send_id = msg->data;
      }
      Info("S_SID: " << (int)dustbin_set_state.send_id);
      dustbin_set_state.send_id %= 10000;
      dustbin_set_state.send_id_loop_cnt = 0;
    }
  }
}
void wirelessCharge_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  Info("S_WC: " << (int)msg->data);
  send_battery_board.soft_stop = 0;  //　软急停　０：无作为　１：复位　
  send_battery_board.wireless_charge = msg->data;

  construct_serial_frame_ex(&user_frame, SEND_BATTERY_CMD,
                            sizeof(send_battery_board), &send_battery_board);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_BATTERY_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
}

void exit_charging_Callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (stm32_update_flag) {
    return;
  }
  Info("S_EX_CG: " << (int)msg->data);
  send_battery_board.exit_charging_state = msg->data;
  std::cout << "退出充电状态" << std::endl;

  construct_serial_frame_ex(&user_frame, SEND_BATTERY_CMD,
                            sizeof(send_battery_board), &send_battery_board);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_BATTERY_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);

  send_battery_board.exit_charging_state = 0;
}

//***************************************lin通讯超声波控制接收
void ultCmdCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  //车身超声波处理
  if (!LIN_ult_installed) {
    return;
  }
  if (cti_run_ver == "v7.0") {
    //如果是环卫车3.0先不设置模式
    return;
  }
  vehicle_ult_cmd = 0x0000000000003fff & msg->data;
  vehicle_ult_set_state.set_mode = vehicle_ult_cmd;
  //如果是关闭或者打开整体超声波需要设置，如果只是设置单个的超声波就不需要发送设置
  if (vehicle_ult_set_state.set_mode == 0) {
    vehicle_ult_set_state.state = 1;  //开始查询
    vehicle_ult_set_state.set_mode =
        vehicle_all_stop_work_mode.data;  //设置全部休眠
  } else if (vehicle_ult_set_state.set_mode != 0 &&
             vehicle_ult_set_state.recv_mode !=
                 vehicle_ult_set_state.set_mode) {
    vehicle_ult_set_state.state = 1;  //开始查询
    vehicle_ult_set_state.set_mode =
        vehicle_defalt_linult_mode.data;  //设置默认模式
  } else {
    vehicle_ult_set_state.state = 0;  //空闲
    vehicle_ult_set_state.set_mode =
        vehicle_defalt_linult_mode.data;  //默认模式
  }

  //箱子超声波处理
  dustbox_ult_cmd = 0x000000000001c000 & msg->data;
  dustbox_ult_set_state.set_mode = dustbox_ult_cmd;
  //如果是关闭或者打开整体超声波需要设置，如果只是设置单个的超声波就不需要发送设置
  if (dustbox_ult_set_state.set_mode == 0) {
    dustbox_ult_set_state.state = 1;  //开始查询
    dustbox_ult_set_state.set_mode =
        dustbox_all_stop_work_mode.data;  //设置全部休眠
  } else if (dustbox_ult_set_state.set_mode != 0 &&
             dustbox_ult_set_state.recv_mode !=
                 dustbox_ult_set_state.set_mode) {
    dustbox_ult_set_state.state = 1;  //开始查询
    dustbox_ult_set_state.set_mode =
        dustbox_defalt_linult_mode.data;  //设置默认模式
  } else {
    dustbox_ult_set_state.state = 0;  //空闲
    dustbox_ult_set_state.set_mode =
        dustbox_defalt_linult_mode.data;  //默认模式
  }

#if DEBUG_PRINT
    RCLCPP_INFO(this->get_logger(), "recv ult cmd: %d", msg->data));
#endif
    Info("V_ULT_CMD: " << (int)msg->data);
}

// (x,y),(0,0),(-2,0)  三点求角
double getAngleByThreeP(double x, double y) {
  double pointx[] = {x, 0, -2};
  double pointy[] = {y, 0, 0};
  double a_b_x = pointx[0] - pointx[1];
  double a_b_y = pointy[0] - pointy[1];
  double c_b_x = pointx[2] - pointx[1];
  double c_b_y = pointy[2] - pointy[1];
  double ab_mul_cb = a_b_x * c_b_x + a_b_y * c_b_y;
  double dist_ab = sqrt(a_b_x * a_b_x + a_b_y * a_b_y);
  double dist_cd = sqrt(c_b_x * c_b_x + c_b_y * c_b_y);
  double cosValue = ab_mul_cb / (dist_ab * dist_cd);
  return 90 - acos(cosValue) * 180 / PI;
}

//************************************** ３．０超声波发布话题
void pub_alt_3_0_(const msg_upa_pos_data_t *data) {
  cti_fpga_serial_msgs::msg::UltV30Datas ult_datas;
  ult_datas.ori_datas.data.push_back((double)data->upa_data[2]);
  ult_datas.ori_datas.data.push_back((double)data->upa_data[3]);
  ult_datas.ori_datas.data.push_back((double)data->upa_data[6]);
  ult_datas.ori_datas.data.push_back((double)data->upa_data[7]);

  ult_datas.pos_front_datas.push_back((double)data->pos_dat[0][0]);
  ult_datas.pos_front_datas.push_back((double)data->pos_dat[0][1]);

  ult_datas.pos_back_datas.push_back((double)data->pos_dat[1][0]);
  ult_datas.pos_back_datas.push_back((double)data->pos_dat[1][1]);

  new_lin_ult_data_v3_0_pub->publish(ult_datas);

  Info("T_UL_PF1: " << (double)data->pos_flag[0]
                    << " T_UL_PF2: " << (double)data->pos_flag[1]
                    << " T_UL_SS1: " << (double)data->upa_data[0]
                    << " T_UL_SS2: " << (double)data->upa_data[1]
                    << " T_UL_SS3: " << (double)data->upa_data[2]
                    << " T_UL_SS4: " << (double)data->upa_data[3]
                    << " T_UL_SS5: " << (double)data->upa_data[4]
                    << " T_UL_SS6: " << (double)data->upa_data[5]
                    << " T_UL_SS7: " << (double)data->upa_data[6]
                    << " T_UL_SS8: " << (double)data->upa_data[7]
                    << " T_UL_PSX1: " << (double)data->pos_dat[0][0]
                    << " T_UL_PSY1: " << (double)data->pos_dat[0][1]
                    << " T_UL_PSX2: " << (double)data->pos_dat[1][0]
                    << " T_UL_PSY2: " << (double)data->pos_dat[1][1]);

  ////////////////////////////////////////////////////
  // sensor_msgs::msg::Range alt_3_0_msg;
  // alt_3_0_msg.radiation_type = 0;
  // alt_3_0_msg.min_range = 0.3;
  // alt_3_0_msg.max_range = 1.5;
  // alt_3_0_msg.field_of_view = 90;

  // alt_3_0_msg.range = (double)data->upa_data[2];
  // alt_3_0_pub[ult_3_0_right_front_down]->publish(alt_3_0_msg);

  // alt_3_0_msg.range = (double)data->upa_data[3];
  // alt_3_0_pub[ult_3_0_right_down]->publish(alt_3_0_msg);

  // alt_3_0_msg.range = (double)data->upa_data[6];
  // alt_3_0_pub[ult_3_0_left_down]->publish(alt_3_0_msg);

  // alt_3_0_msg.range = (double)data->upa_data[7];
  // alt_3_0_pub[ult_3_0_left_front_down]->publish(alt_3_0_msg);

  // alt_3_0_msg.field_of_view = getAngleByThreeP((double)data->pos_dat[0][0],
  //                                              (double)data->pos_dat[0][1]);
  // alt_3_0_msg.range = (double)data->pos_dat[0][1];
  // alt_3_0_pub[ult_3_0_front]->publish(alt_3_0_msg);

  // alt_3_0_msg.field_of_view = getAngleByThreeP((double)data->pos_dat[1][0],
  //                                              (double)data->pos_dat[1][1]);
  // alt_3_0_msg.range = (double)data->pos_dat[1][1];
  // alt_3_0_pub[ult_3_0_back]->publish(alt_3_0_msg);

  // std::cout << "３．０超声波发布话题 test" << std::endl;
  // std::cout << "dat_seq:" << (int)data->dat_seq << std::endl;
  // std::cout << "com_seq:" << (int)data->com_seq << std::endl;
  // std::cout << "pos_flag0:" << data->pos_flag[0] << std::endl;
  // std::cout << "pos_flag1:" << data->pos_flag[1] << std::endl;
  // for (auto data_ : data->upa_data) {
  //   std::cout << "upa_data:" << data_ << std::endl;
  // }
  // for (auto data_ : data->pos_dat) {
  //   for (auto data_bottom : data->pos_dat) {
  //     std::cout << "pos_dat:" << data_bottom << std::endl;
  //   }
  // }
  // std::cout << "unused:" << data->unused << std::endl;
  // std::cout << "３．０超声波发布话题 test end" << std::endl;
}

//************************************** 接收到控制状态后发布话题
void pub_odom(const recv_from_control_status_type *data) {
  if (!data) {
    return;
  }
  //-------------------通讯检测------------------
  chassis_chat_state.module_id = 0;
  chassis_chat_state.recv_state = true;
  chassis_chat_state.time_recv = rclcpp::Clock().now().seconds();
  //---------------------------odom_4wd publish
  tf2::Quaternion q_tf;
  q_tf.setRPY(0.0, 0.0, data->angle_yaw_radian);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q_tf);
  odom_4wd.header.stamp = rclcpp::Clock().now();
  odom_4wd.header.frame_id = "odom";
  // set the position
  odom_4wd.pose.pose.position.x = data->pos.x;
  odom_4wd.pose.pose.position.y = data->pos.y;
  odom_4wd.pose.pose.position.z = 0.0;
  odom_4wd.pose.pose.orientation = odom_quat;
  // set the velocity
  odom_4wd.child_frame_id = "base_link";
  odom_4wd.twist.twist.linear.x =
      data->vel_liner_x;  // vx == 100时！是底盘有问题！！！！！！！！！
  odom_4wd.twist.twist.linear.y = data->vel_liner_y;
  odom_4wd.twist.twist.angular.z = data->vel_angular;
  // publish the message
  odom_pub_4wd->publish(odom_4wd);
  //---------------------------ctlinfo publish
  runinfo.state_vehicle = data->state_vehicle;    //底盘状态
  runinfo.drivers_enable = data->drivers_enable;  //电机驱动使能
  runinfo.control_mode = data->control_mode;  //控制方式 1:遥控 2：导航
  runinfo.state_brake = data->state_brake;    //刹车
  runinfo.vel_liner_x =
      data->vel_liner_x;  // x轴线速度    vx ==
                          // 100时！是底盘有问题！！！！！！！！！
  runinfo.vel_liner_y = data->vel_liner_y;            // Y轴线速度
  runinfo.vel_angular = data->vel_angular;            //角速度
  runinfo.angle_front_turn = data->angle_front_turn;  //前轮转角
  runinfo.angle_rear_turn = data->angle_rear_turn;    //后轮转角
  //霍尔信号
  uint8_t box_sw = 0;
  box_sw = box_sw | (data->sw_status.bits.box_pos_reach1 & 0x01);
  box_sw = box_sw | ((data->sw_status.bits.box_pos_reach2 << 1) & 0x02);
  box_sw = box_sw | ((data->sw_status.bits.box_pos_reach3 << 2) & 0x04);
  //触边
  box_sw = box_sw | ((!(data->sw_status.bits.lift_pressure) << 3) & 0x08);
  runinfo.box_sw = box_sw;                                //霍尔信号+触边
  runinfo.front_bar = !(data->sw_status.bits.front_bar);  //前防撞杆
  runinfo.rear_bar = !(data->sw_status.bits.rear_bar);    //后防撞杆
  runinfo.lift_position = data->lift_position;            //顶升位置
  runinfo.odometer = data->odometer;                      //里程
  ctlinfo_pub->publish(runinfo);
  //---------------------------Imu publish

  // imu 时间戳
  imu_info.header.stamp = rclcpp::Clock().now();
  imu_info.header.frame_id = "/fpga_serial/imu";
  //四元数位姿
  imu_info.orientation.w = data->q[0];
  imu_info.orientation.x = data->q[1];
  imu_info.orientation.y = data->q[2];
  imu_info.orientation.z = data->q[3];
  if (std::isnan(data->q[0]) || std::isnan(data->q[1]) ||
      std::isnan(data->q[2]) || std::isnan(data->q[3])) {
    serial_status.data =
        serial_status.data | 0x0020;  //如果四元数出现NAN，状态位倒数第六位置1
    serial_status.recv_imu_nan = -1;
  } else {
    serial_status.data =
        serial_status.data & 0xFFDF;  //如果四元数没有NAN，状态位倒数第六位置0
    serial_status.recv_imu_nan = 0;
  }
  //线加速度
  imu_info.linear_acceleration.x = data->acc[0] * GRAV;
  imu_info.linear_acceleration.y = data->acc[1] * GRAV;
  imu_info.linear_acceleration.z = -data->acc[2] * GRAV;
  //角速度
  imu_info.angular_velocity.x = data->gyro[0] * PI / 180.0;
  imu_info.angular_velocity.y = data->gyro[1] * PI / 180.0;
  imu_info.angular_velocity.z = -data->gyro[2] * PI / 180.0;
  //发布
  imudata_pub->publish(imu_info);
  // sins时间戳
  sins_info.header.stamp = imu_info.header.stamp;
  sins_info.header.frame_id = "/fpga_serial/sins";
  // imu
  sins_info.imu_data = imu_info;
  //位置
  sins_info.position_x = data->position[0];
  sins_info.position_y = data->position[1];
  sins_info.position_z = data->position[2];
  //线速度
  sins_info.linear_velocity_x = data->linear_velocity[0];
  sins_info.linear_velocity_y = data->linear_velocity[1];
  sins_info.linear_velocity_z = data->linear_velocity[2];
  //气压计
  sins_info.barometer_rawdata = data->baro_raw;
  //判断气压计数据突然出现大于100 的跳变
  if (abs(data->baro_raw - baro_raw_old) > 100) {
    baro_status.data = 1;
    baro_status_pub->publish(baro_status);
    Info("BARO_RAW_ERROR: "
         << " rawdata_before: " << baro_raw_old
         << " rawdata_now: " << data->baro_raw);
    baro_status.data = 0;
  }
  baro_raw_old = data->baro_raw;
  sins_info.barometer_height = data->baro_height;
  //标志位
  sins_info.state_flag = data->state_flag;
  //发布
  sins_pub->publish(sins_info);

  //---------------------------box_laser publish
  box_laser.header.stamp = rclcpp::Clock().now();
  box_laser.header.frame_id = "/fpga_serial/box_laser";
  uint8_t id_data[6];
  for (int i = 0; i < 6; i++) {
    id_data[i] = data->laser_id[i];
  }
  std::string id_str;
  id_str = hex2string(&(id_data[0]), 6);
  box_laser.rtcm_type = id_str;
  box_laser.data.push_back(data->laser_data[0]);
  box_laser.data.push_back(data->laser_data[1]);
  box_laser_pub->publish(box_laser);
  box_laser.data.clear();
  //------------------------磁吸锁状态发布
  std_msgs::msg::UInt8 boxlockstate;
  boxlockstate.data = data->laser_id[0];
  boxlock_state_pub->publish(boxlockstate);

  //--------------------------磁力计发布
  std_msgs::msg::Int32MultiArray compass_msg;
  compass_msg.data.push_back(data->compass1_str[0]);
  compass_msg.data.push_back(data->compass1_str[1]);
  compass_msg.data.push_back(data->compass1_str[2]);
  compass_msg.data.push_back(data->compass2_str[0]);
  compass_msg.data.push_back(data->compass2_str[1]);
  compass_msg.data.push_back(data->compass2_str[2]);
  compass_pub->publish(compass_msg);

  //---------------------------发布到话题"/cti/fpga_serial/serial_status"
  serial_status.recv_hall_turn_front_left =
      data->sw_status.bits.turn_front_left;
  serial_status.recv_hall_turn_front_center =
      data->sw_status.bits.turn_front_center;
  serial_status.recv_hall_turn_front_right =
      data->sw_status.bits.turn_front_right;
  serial_status.recv_hall_turn_rear_left = data->sw_status.bits.turn_rear_left;
  serial_status.recv_hall_turn_rear_center =
      data->sw_status.bits.turn_rear_center;
  serial_status.recv_hall_turn_rear_right =
      data->sw_status.bits.turn_rear_right;
  serial_status.recv_box_pos_reach1 = data->sw_status.bits.box_pos_reach1;
  serial_status.recv_box_pos_reach2 = data->sw_status.bits.box_pos_reach2;
  serial_status.recv_box_pos_reach2 = data->sw_status.bits.box_pos_reach2;
  serial_status.recv_lift_pressure = !(data->sw_status.bits.lift_pressure);
  //-------------------------接受到的所有信息全部发布出去,用于记log
  cti_fpga_serial_msgs::msg::VehicleState chassis_info_msg;
  chassis_info_msg.port_index = data->portIndex;
  chassis_info_msg.state_vehicle = data->state_vehicle;
  chassis_info_msg.drivers_enable = data->drivers_enable;
  chassis_info_msg.control_mode = data->control_mode;
  chassis_info_msg.state_brake = data->state_brake;
  chassis_info_msg.vel_liner_x = data->vel_liner_x;
  chassis_info_msg.vel_liner_y = data->vel_liner_y;
  chassis_info_msg.vel_angular = data->vel_angular;
  chassis_info_msg.angle_front_turn = data->angle_front_turn;
  chassis_info_msg.angle_rear_turn = data->angle_rear_turn;
  chassis_info_msg.sw_status_data = data->sw_status.data;
  chassis_info_msg.odometer = data->odometer;
  chassis_info_msg.pos_x = data->pos.x;
  chassis_info_msg.pos_y = data->pos.y;
  chassis_info_msg.angle_yaw_radian = data->angle_yaw_radian;
  chassis_info_msg.lift_position = data->lift_position;
  for (int i = 0; i < (sizeof(data->acc) / sizeof(data->acc[0])); i++) {
    chassis_info_msg.acc.push_back(data->acc[i]);
  }
  for (int i = 0; i < (sizeof(data->gyro) / sizeof(data->gyro[0])); i++) {
    chassis_info_msg.gyro.push_back(data->gyro[i]);
  }
  for (int i = 0; i < (sizeof(data->q) / sizeof(data->q[0])); i++) {
    chassis_info_msg.q.push_back(data->q[i]);
  }
  for (int i = 0; i < (sizeof(data->position) / sizeof(data->position[0]));
       i++) {
    chassis_info_msg.position.push_back(data->position[i]);
  }
  for (int i = 0;
       i < (sizeof(data->linear_velocity) / sizeof(data->linear_velocity[0]));
       i++) {
    chassis_info_msg.linear_velocity.push_back(data->linear_velocity[i]);
  }
  chassis_info_msg.baro_raw = data->baro_raw;
  chassis_info_msg.baro_height = data->baro_height;
  for (int i = 0;
       i < (sizeof(data->compass1_str) / sizeof(data->compass1_str[0])); i++) {
    chassis_info_msg.compass1_str.push_back(data->compass1_str[i]);
  }
  for (int i = 0;
       i < (sizeof(data->compass2_str) / sizeof(data->compass2_str[0])); i++) {
    chassis_info_msg.compass2_str.push_back(data->compass2_str[i]);
  }
  chassis_info_msg.state_flag = data->state_flag;
  for (int i = 0; i < (sizeof(data->laser_id) / sizeof(data->laser_id[0]));
       i++) {
    chassis_info_msg.laser_id.push_back(data->laser_id[i]);
  }
  for (int i = 0; i < (sizeof(data->laser_data) / sizeof(data->laser_data[0]));
       i++) {
    chassis_info_msg.laser_data.push_back(data->laser_data[i]);
  }
  recv_chassis_info_pub->publish(chassis_info_msg);
}

//************************************** 接收到电池状态后发布话题 不带无线充电
void pub_battery(const recv_battery_4_to_1_active_report_status_type *data) {
  if (!data) {
    return;
  }
  batteryState.header.stamp = rclcpp::Clock().now();
  batteryState.voltage_all =
      cmax(cmax(data->Bat_1_Volt, data->Bat_2_Volt), data->Bat_3_Volt);
  // batteryState.power =
  // cmax(cmax(data->Bat_1_Soc,data->Bat_2_Soc),data->Bat_3_Soc);
  if (robot_type == 0) {
    //阳光物流车
    batteryState.power =
        (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc) / 3;
  } else if (robot_type == 1) {
    //阳光环卫车
    if (cti_run_ver == "v7.0") {  //如果是硬件7.0版本
      batteryState.power = data->Bat_1_Soc;
    } else {
      batteryState.power = (data->Bat_1_Soc + data->Bat_2_Soc) / 2;
    }
  }
  if (batteryState.power > 100) {
    batteryState.power = 100;
  }
  batteryState.temperature =
      cmax(cmax(data->Bat_1_temp, data->Bat_2_temp), data->Bat_3_temp);
  batteryState.power_supply_status = data->ChargeStatus;
  batteryState.wireless_install_state = 0;
  batteryState.wireless_voltage = 0;
  batteryState.wireless_current = 0;
  batteryState.wireless_reserve = 0;
  batteryState.wireless_state = 0;
  batteryState.wireless_temp = 0;
  batteryState.wireless_changer = 0;
  battery_pub->publish(batteryState);
  BatCellsState.batcells.clear();
  BatCellsState.header.stamp = rclcpp::Clock().now();
  BatCellsState.soc_all = batteryState.power;
  BatCellsState.volt_all =
      cmax(cmax(data->Bat_1_Volt, data->Bat_2_Volt), data->Bat_3_Volt);
  cti_msgs::msg::BatteryCell BatCell;
  if (data->Bat_1_Volt > 5) {
    BatCell.bat_volt = data->Bat_1_Volt;
    BatCell.bat_temp = data->Bat_1_temp;
    BatCell.bat_curr = data->Bat_1_curr;
    BatCell.bat_soc = data->Bat_1_Soc;
    BatCell.bat_comm_state = data->Bat_1_comm_state;
    BatCell.bat_cell_num = data->Bat_1_cell_num;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->Bat_2_Volt > 5) {
    BatCell.bat_volt = data->Bat_2_Volt;
    BatCell.bat_temp = data->Bat_2_temp;
    BatCell.bat_curr = data->Bat_2_curr;
    BatCell.bat_soc = data->Bat_2_Soc;
    BatCell.bat_comm_state = data->Bat_2_comm_state;
    BatCell.bat_cell_num = data->Bat_2_cell_num;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->Bat_3_Volt > 5) {
    BatCell.bat_volt = data->Bat_3_Volt;
    BatCell.bat_temp = data->Bat_3_temp;
    BatCell.bat_curr = data->Bat_3_curr;
    BatCell.bat_soc = data->Bat_3_Soc;
    BatCell.bat_comm_state = data->Bat_3_comm_state;
    BatCell.bat_cell_num = data->Bat_3_cell_num;
    BatCellsState.batcells.push_back(BatCell);
  }
  //--
  BatCellsState.bat_backup_volt = data->Bat_backup_Volt;
  BatCellsState.bus_volt = data->Bus_Volt;
  BatCellsState.charger_volt = data->Charger_Volt;

  BatCellsState.board_temp.clear();
  BatCellsState.board_temp.push_back(data->Board_temp1);
  BatCellsState.board_temp.push_back(data->Board_temp2);
  BatCellsState.board_temp.push_back(data->Board_temp3);
  BatCellsState.board_temp.push_back(data->Board_temp4);
  BatCellsState.board_temp.push_back(data->Board_temp5);
  BatCellsState.board_temp.push_back(data->Board_temp6);
  BatCellsState.board_temp.push_back(data->Board_temp7);
  BatCellsState.board_temp.push_back(data->Board_temp8);
  BatCellsState.board_curr.clear();
  BatCellsState.board_curr.push_back(data->Board_curr1);
  BatCellsState.board_curr.push_back(data->Board_curr2);
  BatCellsState.board_curr.push_back(data->Board_curr3);
  BatCellsState.board_curr.push_back(data->Board_curr4);
  BatCellsState.board_curr.push_back(data->Board_curr5);
  BatCellsState.board_curr.push_back(data->Board_curr6);
  BatCellsState.board_curr.push_back(data->Board_curr7);
  BatCellsState.board_curr.push_back(data->Board_curr8);
  //--
  BatCellsState.charge_reverse = data->charge_reverse;
  BatCellsState.v_leakage = data->v_leakage;
  BatCellsState.charge_status = data->ChargeStatus;
  BatCellsState.lock_status = data->Lock_status;
  BatCellsState.errorinfo = data->errorInfo;
  //充电状态
  BatCellsState.charge_type = 0;  // 0:undefined 1:wireless_charge
                                  // 2:wired_charge
  batcell_pub->publish(BatCellsState);
  Info("R_WC_IS: " << (int)batteryState.wireless_install_state
                   << " R_BA_VT: " << batteryState.voltage_all
                   << " R_BA_PW: " << (int)batteryState.power << " R_BA_1_VO: "
                   << data->Bat_1_Volt << " R_BA_2_VO: " << data->Bat_2_Volt
                   << " R_BA_3_VO: " << data->Bat_3_Volt
                   << " R_BA_1_SOC: " << (int)data->Bat_1_Soc
                   << " R_BA_2_SOC: " << (int)data->Bat_2_Soc
                   << " R_BA_3_SOC: " << (int)data->Bat_3_Soc
                   << " R_BA_VO: " << BatCellsState.volt_all
                   << " R_BA_SOC: " << (int)BatCellsState.soc_all);
}

//************************************** 接收到电池状态后发布话题,带有无线充电
void pub_battery_with_wireless(
    const recv_battery_4_to_1_active_report_status_with_wireless_type *data) {
  if (!data) {
    return;
  }
  batteryState.header.stamp = rclcpp::Clock().now();
  batteryState.voltage_all =
      cmax(cmax(data->Bat_1_Volt, data->Bat_2_Volt), data->Bat_3_Volt);
  // batteryState.power =
  // cmax(cmax(data->Bat_1_Soc,data->Bat_2_Soc),data->Bat_3_Soc);
  if (robot_type == 0) {
    //阳光物流车
    batteryState.power =
        (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc) / 3;
  } else if (robot_type == 1) {
    //阳光环卫车
    batteryState.power = (data->Bat_1_Soc + data->Bat_2_Soc) / 2;
  }
  if (batteryState.power > 100) {
    batteryState.power = 100;
  }
  batteryState.temperature =
      cmax(cmax(data->Bat_1_temp, data->Bat_2_temp), data->Bat_3_temp);
  batteryState.power_supply_status = data->ChargeStatus;
  batteryState.wireless_install_state = 1;
  batteryState.wireless_voltage = data->wireless_voltage;
  batteryState.wireless_current = data->wireless_current;
  batteryState.wireless_reserve = data->wireless_reserve;
  batteryState.wireless_state = data->wireless_state;
  batteryState.wireless_temp = data->wireless_temp;
  batteryState.wireless_changer = data->wireless_changer;
  battery_pub->publish(batteryState);

  //新的电池信息发布
  BatCellsState.batcells.clear();
  BatCellsState.header.stamp = rclcpp::Clock().now();
  BatCellsState.soc_all = batteryState.power;
  BatCellsState.volt_all =
      cmax(cmax(data->Bat_1_Volt, data->Bat_2_Volt), data->Bat_3_Volt);
  cti_msgs::msg::BatteryCell BatCell;
  if (data->Bat_1_Volt > 5) {
    BatCell.bat_volt = data->Bat_1_Volt;
    BatCell.bat_temp = data->Bat_1_temp;
    BatCell.bat_curr = data->Bat_1_curr;
    BatCell.bat_soc = data->Bat_1_Soc;
    BatCell.bat_comm_state = data->Bat_1_comm_state;
    BatCell.bat_cell_num = data->Bat_1_cell_num;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->Bat_2_Volt > 5) {
    BatCell.bat_volt = data->Bat_2_Volt;
    BatCell.bat_temp = data->Bat_2_temp;
    BatCell.bat_curr = data->Bat_2_curr;
    BatCell.bat_soc = data->Bat_2_Soc;
    BatCell.bat_comm_state = data->Bat_2_comm_state;
    BatCell.bat_cell_num = data->Bat_2_cell_num;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->Bat_3_Volt > 5) {
    BatCell.bat_volt = data->Bat_3_Volt;
    BatCell.bat_temp = data->Bat_3_temp;
    BatCell.bat_curr = data->Bat_3_curr;
    BatCell.bat_soc = data->Bat_3_Soc;
    BatCell.bat_comm_state = data->Bat_3_comm_state;
    BatCell.bat_cell_num = data->Bat_3_cell_num;
    BatCellsState.batcells.push_back(BatCell);
  }
  //--
  BatCellsState.bat_backup_volt = data->Bat_backup_Volt;
  BatCellsState.bus_volt = data->Bus_Volt;
  BatCellsState.charger_volt = data->Charger_Volt;

  BatCellsState.board_temp.clear();
  BatCellsState.board_temp.push_back(data->Board_temp1);
  BatCellsState.board_temp.push_back(data->Board_temp2);
  BatCellsState.board_temp.push_back(data->Board_temp3);
  BatCellsState.board_temp.push_back(data->Board_temp4);
  BatCellsState.board_temp.push_back(data->Board_temp5);
  BatCellsState.board_temp.push_back(data->Board_temp6);
  BatCellsState.board_temp.push_back(data->Board_temp7);
  BatCellsState.board_temp.push_back(data->Board_temp8);
  BatCellsState.board_curr.clear();
  BatCellsState.board_curr.push_back(data->Board_curr1);
  BatCellsState.board_curr.push_back(data->Board_curr2);
  BatCellsState.board_curr.push_back(data->Board_curr3);
  BatCellsState.board_curr.push_back(data->Board_curr4);
  BatCellsState.board_curr.push_back(data->Board_curr5);
  BatCellsState.board_curr.push_back(data->Board_curr6);
  BatCellsState.board_curr.push_back(data->Board_curr7);
  BatCellsState.board_curr.push_back(data->Board_curr8);
  //--
  BatCellsState.charge_reverse = data->charge_reverse;
  BatCellsState.v_leakage = data->v_leakage;
  BatCellsState.charge_status = data->ChargeStatus;
  BatCellsState.lock_status = data->Lock_status;
  BatCellsState.errorinfo = data->errorInfo;
  //充电状态
  BatCellsState.charge_type = 1;  // 0:undefined 1:wireless_charge
                                  // 2:wired_charge
  BatCellsState.charge_voltage = data->wireless_voltage;
  BatCellsState.charge_current = data->wireless_current;
  BatCellsState.charge_reserve = data->wireless_reserve;
  BatCellsState.charge_state = data->wireless_state;
  BatCellsState.charge_temp = data->wireless_temp;
  //对充电标志位进行窗口滤波
  static std::deque<uint8_t> vehicle_charger_queue_;
  vehicle_charger_queue_.push_back(data->wireless_changer);
  if (vehicle_charger_queue_.size() > 3) vehicle_charger_queue_.pop_front();
  uint8_t wireless_changer = 1;
  if (vehicle_charger_queue_[0] == 0 && vehicle_charger_queue_[1] == 0 &&
      vehicle_charger_queue_[2] == 0)
    wireless_changer = 0;
  BatCellsState.charge_charger = wireless_changer;

  batcell_pub->publish(BatCellsState);

  Info("R_WC_IS: " << (int)batteryState.wireless_install_state
                   << " R_WC_VT: " << (int)data->wireless_voltage
                   << " R_WC_CU: " << (int)data->wireless_current
                   << " R_WC_RE: " << (int)data->wireless_reserve
                   << " R_WC_ST: " << (int)data->wireless_state
                   << " R_WC_TP: " << (int)data->wireless_temp
                   << " R_WC_CH: " << (int)data->wireless_changer
                   << " R_WC_CH_RE: " << (int)wireless_changer
                   << " R_BA_VT: " << batteryState.voltage_all
                   << " R_BA_PW: " << (int)batteryState.power << " R_BA_1_VO: "
                   << data->Bat_1_Volt << " R_BA_2_VO: " << data->Bat_2_Volt
                   << " R_BA_3_VO: " << data->Bat_3_Volt
                   << " R_BA_1_SOC: " << (int)data->Bat_1_Soc
                   << " R_BA_2_SOC: " << (int)data->Bat_2_Soc
                   << " R_BA_3_SOC: " << (int)data->Bat_3_Soc
                   << " R_BA_SOC: " << (int)BatCellsState.soc_all
                   << " R_BA_VO: " << BatCellsState.volt_all);
}

//**************************************
//接收到电池状态后发布话题,3.0车辆的点车状态
void pub_battery_3_0(
    const recv_battery_4_to_1_active_report_status_type_3_0_ *data) {
  //数据处理 to_do
  // std::cout << "recv battery v3_0" << std::endl;
  if (!data) {
    return;
  }
  // batteryState.header.stamp = rclcpp::Clock().now();
  // batteryState.voltage_all = data->Bat_1_Volt;
  // batteryState.power = (double)data->Bat_1_Soc;
  // batteryState.temperature = data->Bat_1_temp;
  // batteryState.power_supply_status = data->ChargeStatus;
  // batteryState.wireless_install_state = 0;
  // batteryState.wireless_voltage = 0;
  // batteryState.wireless_current = 0;
  // batteryState.wireless_reserve = 0;
  // batteryState.wireless_state = 0;
  // batteryState.wireless_temp = 0;
  // batteryState.wireless_changer = 0;

  // battery_pub->publish(batteryState);
  //////////////////////////////////////////////////////////////////
  // std::cout<<"------------------------11------------------------"<<std::endl;
  // std::cout<<"------------------------------------------------"<<std::endl;
  // std::cout<<"------------------------------------------------"<<std::endl;
  //电池电量发布
  cti_msgs::msg::BatteryCellsState BatCellsState;
  BatCellsState.header.stamp = rclcpp::Clock().now();
  BatCellsState.soc_all = (double)data->Bat_1_Soc;
  BatCellsState.volt_all = data->Bat_1_Volt;
  BatCellsState.charger_volt = data->battery_charger_voltage;

  // BatCellsState.board_temp[0] = data->Board_temp1;  // Bat1 MOS
  // BatCellsState.board_temp[1] = data->Board_temp2;  // MCU
  // BatCellsState.board_temp[2] = data->Board_temp3;  //
  // BatCellsState.board_temp[3] = data->Board_temp4;  // front left motor
  // driver MOS BatCellsState.board_temp[4] = data->Board_temp5;  // charger In
  // MOS BatCellsState.board_temp[5] = data->Board_temp6;  // clean board port
  // BatCellsState.board_temp[6] = data->Board_temp7;  // back left motor driver
  // MOS BatCellsState.board_temp[7] = data->Board_temp8;  // inverter port

  // BatCellsState.board_curr[0] = data->Board_curr1;
  // BatCellsState.board_curr[1] = data->Board_curr2;
  // BatCellsState.board_curr[2] = data->Board_curr3;
  // BatCellsState.board_curr[3] = data->Board_curr4;
  // BatCellsState.board_curr[4] = data->Board_curr5;
  // BatCellsState.board_curr[5] = data->Board_curr6;
  // BatCellsState.board_curr[6] = data->Board_curr7;
  // BatCellsState.board_curr[7] = data->Board_curr8;

  BatCellsState.board_temp.clear();
  BatCellsState.board_temp.push_back(data->Board_temp1);
  BatCellsState.board_temp.push_back(data->Board_temp2);
  BatCellsState.board_temp.push_back(data->Board_temp3);
  BatCellsState.board_temp.push_back(data->Board_temp4);
  BatCellsState.board_temp.push_back(data->Board_temp5);
  BatCellsState.board_temp.push_back(data->Board_temp6);
  BatCellsState.board_temp.push_back(data->Board_temp7);
  BatCellsState.board_temp.push_back(data->Board_temp8);
  BatCellsState.board_curr.clear();
  BatCellsState.board_curr.push_back(data->Board_curr1);
  BatCellsState.board_curr.push_back(data->Board_curr2);
  BatCellsState.board_curr.push_back(data->Board_curr3);
  BatCellsState.board_curr.push_back(data->Board_curr4);
  BatCellsState.board_curr.push_back(data->Board_curr5);
  BatCellsState.board_curr.push_back(data->Board_curr6);
  BatCellsState.board_curr.push_back(data->Board_curr7);
  BatCellsState.board_curr.push_back(data->Board_curr8);

  BatCellsState.charge_reverse = data->charge_reverse;
  BatCellsState.v_leakage = data->v_leakage;
  BatCellsState.charge_status = data->ChargeStatus;
  BatCellsState.lock_status = data->brake_status;
  BatCellsState.errorinfo = data->ErrorInfo;

  // BatCellsState.bat_curr = data->Bat_1_curr;
  // BatCellsState.bat_temp = data->Bat_1_temp;

  cti_msgs::msg::BatteryCell BatCell;
  BatCell.bat_soc = data->Bat_1_Soc;
  BatCell.bat_volt = data->Bat_1_Volt;
  BatCell.bat_curr = data->Bat_1_curr;
  BatCell.bat_temp = data->Bat_1_temp;
  BatCell.bat_comm_state = data->Bat_1_comm_state;
  BatCell.bat_cell_num = 1;
  BatCellsState.batcells.push_back(BatCell);

  BatCellsState.charge_type = 2;  // 0:undefined 1:wireless_charge
                                  // 2:wired_charge
  BatCellsState.charge_voltage = data->battery_charger_voltage;
  BatCellsState.charge_current = data->battery_charger_current;
  BatCellsState.charge_reserve = data->charge_reverse;
  BatCellsState.charge_state = data->ChargeStatus;
  BatCellsState.charge_temp = data->Bat_1_temp;
  // BatCellsState.charge_charger = data->battery_charger_state;
  BatCellsState.charge_charger = data->battery_charger_flag;

  batcell_pub->publish(BatCellsState);

  Info(" R_WC_VT: " << (int)data->Bat_1_Volt
                    << " R_WC_CU: " << (int)data->Bat_1_curr
                    << " R_WC_TP: " << (int)data->Bat_1_temp
                    << " R_BA_1_SOC: " << (int)data->Bat_1_Soc
                    << " R_BT_CG_VO: " << (int)data->battery_charger_voltage
                    << " R_BT_CG_CU: " << (int)data->battery_charger_current
                    << " R_BT_CG_CS: " << (int)data->ChargeStatus
                    << " R_BT_CG_FL: " << (int)data->battery_charger_flag
                    << " R_BT_CG_EI: " << (int)data->ErrorInfo);

  ////////////////////////////////////////////////////////////////////////////////

  // std::cout << "portIndex:         " << data->portIndex << std::endl;
  // std::cout << "Bat_1_Volt         " << data->Bat_1_Volt << std::endl;
  // std::cout << "Bat_1_Soc          " << (double)data->Bat_1_Soc << std::endl;
  // std::cout << "Bat_1_temp          " << data->Bat_1_temp << std::endl;
  // std::cout << "Bat_1_temp          " << data->Bat_1_temp << std::endl;
  // std::cout << "Bat_1_comm_state          " << data->Bat_1_comm_state
  //           << std::endl;
  // std::cout << "Bat_1_cell_num          " << data->Bat_1_cell_num <<
  // std::endl; std::cout << "full_capacity          " << data->full_capacity <<
  // std::endl; std::cout << "surplus_capacity          " <<
  // data->surplus_capacity
  //           << std::endl;
  // std::cout << " Bat_1_Soh         " << data->Bat_1_Soh << std::endl;

  // std::cout << "cell_state          " << data->cell_state << std::endl;
  // std::cout << " ntc_state         " << data->ntc_state << std::endl;
  // std::cout << "volt_curr_state          " << data->volt_curr_state
  //           << std::endl;

  // std::cout << "Board_temp1          " << data->Board_temp1 << std::endl;
  // std::cout << "Board_temp2          " << data->Board_temp2 << std::endl;
  // std::cout << "Board_temp3          " << data->Board_temp3 << std::endl;
  // std::cout << "Board_temp4          " << data->Board_temp4 << std::endl;
  // std::cout << "Board_temp5          " << data->Board_temp5 << std::endl;
  // std::cout << "Board_temp6          " << data->Board_temp6 << std::endl;
  // std::cout << "Board_temp7          " << data->Board_temp7 << std::endl;
  // std::cout << "Board_temp8          " << data->Board_temp8 << std::endl;

  // std::cout << "Board_curr1          " << data->Board_curr1 << std::endl;
  // std::cout << "Board_curr2          " << data->Board_curr2 << std::endl;
  // std::cout << "Board_curr3          " << data->Board_curr3 << std::endl;
  // std::cout << "Board_curr4          " << data->Board_curr4 << std::endl;
  // std::cout << "Board_curr5          " << data->Board_curr5 << std::endl;
  // std::cout << "Board_curr6          " << data->Board_curr6 << std::endl;
  // std::cout << "Board_curr7          " << data->Board_curr7 << std::endl;
  // std::cout << "Board_curr8          " << data->Board_curr8 << std::endl;

  // std::cout << "charge_reverse         " << data->charge_reverse <<
  // std::endl; std::cout << "v_leakage          " << data->v_leakage <<
  // std::endl; std::cout << "ChargeStatus          " << data->ChargeStatus <<
  // std::endl; std::cout << "brake_status          " << data->brake_status <<
  // std::endl; std::cout << "ErrorInfo          " << data->ErrorInfo <<
  // std::endl; std::cout << "battery_charger_voltage          "
  //           << data->battery_charger_voltage << std::endl;
  // std::cout << "battery_charger_current          "
  //           << data->battery_charger_current << std::endl;
  // std::cout << "battery_charger_flag          " << data->battery_charger_flag
  //           << std::endl;
  // std::cout << "battery_charger_state          " <<
  // data->battery_charger_state
  //           << std::endl;
}

//************************************** 接收到电机驱动状态后发布话题
void pub_driverstatus(recv_from_driver_status_type *data) {
  if (!data) {
    return;
  }
}

//************************************** 接收到超声波数据后发布话题
void pub_ultrasonicdata(recv_from_ultrasonic_data *data) {
  if (!data) {
    return;
  }
  const float min_range = 0.12;
  const float max_range = 2.5;
  rangeData.header.stamp = rangeDatas.header.stamp = rclcpp::Clock().now();
  rangeData.radiation_type = rangeDatas.radiation_type = 0;  // ult
  rangeData.min_range = rangeDatas.min_range = min_range;
  rangeData.max_range = rangeDatas.max_range = max_range;
  rangeData.field_of_view = 24 * M_PI / 180.0f;
  rangeDatas.range.clear();
  for (int i = 0; i < max_type_ult; i++) {
    float ult_data = (float)data->ult_data[i] / 1000.f;
    rangeDatas.range.push_back(ult_data);
    rangeData.range = ult_data;
    rangeData.header.frame_id = ult_name[i];
    range_pub[i]->publish(rangeData);
  }
  ranges_pub->publish(rangeDatas);
}

//************************************** 接收到超声波数据后发布话题--lin通信
void pub_ultrasonicdata_lin(recv_from_ultrasonic_data_lin *data) {
  if (!data) {
    return;
  }

  //超声波处理方式变更,将源数据发布出去,由另一个节点来专门处理
  std_msgs::msg::UInt16MultiArray lin_ult_data_msgs;
  int data_size = sizeof(data->ult_data) / sizeof(data->ult_data[0]);

  if (cti_run_ver == "v7.0") {
    //如果是环卫车3.0的车这样处理
    for (int i = 0; i < data_size; i++) {
      lin_ult_data_msgs.data.push_back(data->ult_data[i]);
    }
    lin_ult_data_v3_0_pub->publish(lin_ult_data_msgs);
  } else {
    //根据指令将数据过滤一遍，指定关闭的探头数据发2500，探头顺序对应
    // int8_t linult_order_to_control[] = {2, -1, 1, 11, -1, 7, 6, -1, 10, 9, 4,
    // 3, 8, 5};
    for (int i = 0; i < lin_max_type_ult; i++) {
      if ((vehicle_ult_cmd & 0x00000001 << i) == 0) {
        int8_t ult_order = linult_order_to_control[i];  //取得探头编号
        if (ult_order > 0 && ult_order <= data_size) {
          data->ult_data[ult_order - 1] = 2500;  //对应的探头数据设置为2500
        }
      }
    }

    //数据放入消息
    for (int i = 0; i < data_size; i++) {
      lin_ult_data_msgs.data.push_back(data->ult_data[i]);
    }
    lin_ult_data_pub->publish(lin_ult_data_msgs);
  }
}
//**************************************对比两个输入的vector,返回对比结果
bool less_equa_compare(std::vector<int> vec1, std::vector<int> vec2) {
  if (vec1[0] < vec2[0]) {
    return true;
  }
  if ((vec1[0] == vec2[0]) && (vec1[1] < vec2[1])) {
    return true;
  }
  if ((vec1[0] == vec2[0]) && (vec1[1] == vec2[1]) && (vec1[2] <= vec2[2])) {
    return true;
  }
  return false;
}
//************************************** 提取版本号中的数字
//并和版本限制进行对比，如果超出版本限制，导航的运动控制命令不会下发
void getNumInString(std::string str) {
  int str_len;
  int start_itr;
  std::vector<int> version;

  std::string num_str;

  str_len = str.length();
  for (int i = 0; i < str_len; i++) {
    if (str[i] == 'V') {
      start_itr = i + 1;
      break;
    }
  }
  for (int i = start_itr; i < str_len; i++) {
    std::string temp_str = "";
    temp_str = str[i];
    if (str[i] >= '0' && str[i] <= '9')
      num_str.push_back(str[i]);
    else if (str[i] == '.') {
      version.push_back(atoi(num_str.c_str()));
      num_str.clear();
      continue;
    } else
      break;
  }
  version.push_back(atoi(num_str.c_str()));
  num_str.clear();
  // printf("version0: %d\n",version[0]);
  // printf("version1: %d\n",version[1]);
  // printf("version2: %d\n",version[2]);
  // compare with the min_version

  std::vector<int> max_version;
  std::vector<int> min_version;
  max_version.push_back(max_control_board_version_head_);
  max_version.push_back(max_control_board_version_mid_);
  max_version.push_back(max_control_board_version_end_);
  min_version.push_back(min_control_board_version_head_);
  min_version.push_back(min_control_board_version_mid_);
  min_version.push_back(min_control_board_version_end_);

  bool less_equa_than_max_version = less_equa_compare(version, max_version);
  bool more_equa_than_min_version = less_equa_compare(min_version, version);

  if (less_equa_than_max_version && more_equa_than_min_version) {
    control_version_right = 0;
  }
  if (!less_equa_than_max_version && more_equa_than_min_version) {
    control_version_right = 1;
  }
  if (less_equa_than_max_version && !more_equa_than_min_version) {
    control_version_right = -1;
  }
  firmwareVersionCheck.data = control_version_right;
  firmware_version_status_pub->publish(firmwareVersionCheck);
}
//************************************** 接收到固件版本号后发布话题
void pub_firmwareversion(recv_from_firmware_version_type *data) {
  if (!data) {
    return;
  }
  if (data->app_ver != NULL && data->app_ver != " ") {
    get_version_flag = true;
  }
  cti_fpga_serial_msgs::msg::FirmWareInfo fm_info_msg;
  fm_info_msg.src = data->upd_info.src;
  fm_info_msg.dest = data->upd_info.dest;
  fm_info_msg.run_area = data->run_area;
  fm_info_msg.update_status = data->update_status;
  fm_info_msg.boot_ver = data->boot_ver;
  fm_info_msg.app_ver = data->app_ver;
  fm_info_msg.update_lib_ver = data->update_lib_ver;

  firmware_version_check_pub->publish(fm_info_msg);

  switch (data->upd_info.src) {
    case MODULE_DEBUG_BOARD:
      RobotVersionDisplay.debug_interface_version = data->app_ver;
      break;
    case MODULE_MOVE_CONTROL_BOARD: {
      RobotVersionDisplay.operation_control_version = data->app_ver;
      std::string version_info;
      version_info = data->app_ver;
      getNumInString(version_info);
      break;
    }
    case MODULE_POWER_INTEGRATE_BOARD:
      RobotVersionDisplay.battery_tandem_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_FRONT_LEFT:
      RobotVersionDisplay.left_front_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_FRONT_RIGHT:
      RobotVersionDisplay.right_front_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_BACK_LEFT:
      RobotVersionDisplay.left_back_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_DRIVER_BACK_RIGHT:
      RobotVersionDisplay.right_back_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_TURN_FRONT:
      RobotVersionDisplay.forwards_way_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_TURN_BACK:
      RobotVersionDisplay.backwards_way_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_BRAKE_FRONT:
      RobotVersionDisplay.front_brake_diver_version = data->app_ver;
      break;
    case MODULE_MOTOR_BREAK_BACK:
      RobotVersionDisplay.back_brake_diver_version = data->app_ver;
      break;
    case MODULE_LIGHT_START:
      RobotVersionDisplay.light_control_version = data->app_ver;
      break;
    case MODULE_ULTRASONIC_1:
      RobotVersionDisplay.ultrasonicvers[0] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_2:
      RobotVersionDisplay.ultrasonicvers[1] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_3:
      RobotVersionDisplay.ultrasonicvers[2] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_4:
      RobotVersionDisplay.ultrasonicvers[3] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_5:
      RobotVersionDisplay.ultrasonicvers[4] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_6:
      RobotVersionDisplay.ultrasonicvers[5] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_7:
      RobotVersionDisplay.ultrasonicvers[6] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_8:
      RobotVersionDisplay.ultrasonicvers[7] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_9:
      RobotVersionDisplay.ultrasonicvers[8] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_10:
      RobotVersionDisplay.ultrasonicvers[9] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_11:
      RobotVersionDisplay.ultrasonicvers[10] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_12:
      RobotVersionDisplay.ultrasonicvers[11] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_13:
      RobotVersionDisplay.ultrasonicvers[12] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_14:
      RobotVersionDisplay.ultrasonicvers[13] = data->app_ver;
      break;
    case MODULE_ULTRASONIC_15:
      RobotVersionDisplay.ultrasonicvers[14] = data->app_ver;
      break;
    default:
      break;
  }
  firmvion_pub->publish(RobotVersionDisplay);
}

//************************************** 接收到命令应答之后计数
void compar_id_recv_send(recv_from_cmd_answer_type *data) {
  if (!data) {
    return;
  }
  cmd_send_map.clear();
  cmd_answer_success_cnt++;
}

//************************************** rfid成组发布缓存容器push_back
void oldTabstate_Pushback(cti_msgs::msg::TabState *tabstate_ptr, uint8_t id) {
  switch (id) {
    case 1:
      oldtabstate_rfid1.push_back(*tabstate_ptr);
      break;
    case 2:
      oldtabstate_rfid2.push_back(*tabstate_ptr);
      break;
    case 3:
      tabstate_ptr->id = 2;
      oldtabstate_rfid3.push_back(*tabstate_ptr);
      break;
    case 4:
      tabstate_ptr->id = 3;
      oldtabstate_rfid4.push_back(*tabstate_ptr);
      break;
    default:
      break;
  }
}

//************************************** rfid单独发布缓存容器push_back
void rfid_single_Pushback(cti_msgs::msg::TabState *tabstate_ptr, uint8_t id) {
  switch (id) {
    case 1:
      rfid_single.states.push_back(*tabstate_ptr);
      break;
    case 2:
      rfid_single.states.push_back(*tabstate_ptr);
      break;
    case 3:
      // tabstate_ptr->id = 2;
      rfid_single.states.push_back(*tabstate_ptr);
      break;
    case 4:
      // tabstate_ptr->id = 3;
      rfid_single.states.push_back(*tabstate_ptr);
      break;
    default:
      break;
  }
}

//************************************** rfid成组发布缓存容器clear
void oldTabstate_clear(uint8_t id) {
  switch (id) {
    case 1:
      oldtabstate_rfid1.clear();
      break;
    case 2:
      oldtabstate_rfid2.clear();
      break;
    case 3:
      oldtabstate_rfid3.clear();
      break;
    case 4:
      oldtabstate_rfid4.clear();
      break;
    default:
      break;
  }
}

//************************************** rfid接收超时检测
void recv_rfid_timeout(uint8_t id) {
  switch (id) {
    case 1:
      oldtime_rfid1 = rclcpp::Clock().now().seconds();
      break;
    case 2:
      oldtime_rfid2 = rclcpp::Clock().now().seconds();
      break;
    case 3:
      oldtime_rfid3 = rclcpp::Clock().now().seconds();
      break;
    case 4:
      oldtime_rfid4 = rclcpp::Clock().now().seconds();
      break;
    default:
      break;
  }
  double nowtime_rfid = rclcpp::Clock().now().seconds();
  if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid1)) {
    recv_rfid1_timeout = true;
    oldtabstate_rfid1.clear();
    cti_msgs::msg::TabState tabstate;
    tabstate.id = 1;
    tabstate.status = 140;  // 140 means recv timeout!
    tabstate.message = "";
    oldTabstate_Pushback(&tabstate, tabstate.id);
    oldtime_rfid1 = rclcpp::Clock().now().seconds();
    ////Info("RECV RFID: module_id_1 recv timeout!");
  } else {
    recv_rfid1_timeout = false;
  }
  if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid2)) {
    recv_rfid2_timeout = true;
    oldtabstate_rfid2.clear();
    cti_msgs::msg::TabState tabstate;
    tabstate.id = 2;
    tabstate.status = 140;  // 140 means recv timeout!
    tabstate.message = "";
    oldTabstate_Pushback(&tabstate, tabstate.id);
    oldtime_rfid2 = rclcpp::Clock().now().seconds();
    ////Info("RECV RFID: module_id_2 recv timeout!");
  } else {
    recv_rfid2_timeout = false;
  }
  if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid3)) {
    recv_rfid3_timeout = true;
    oldtabstate_rfid3.clear();
    cti_msgs::msg::TabState tabstate;
    tabstate.id = 3;
    tabstate.status = 140;  // 140 means recv timeout!
    tabstate.message = "";
    oldTabstate_Pushback(&tabstate, tabstate.id);
    oldtime_rfid3 = rclcpp::Clock().now().seconds();
    ////Info("RECV RFID: module_id_3 recv timeout!");
  } else {
    recv_rfid3_timeout = false;
  }
  if (RFIDTIMEOUT_DUR < (nowtime_rfid - oldtime_rfid4)) {
    recv_rfid4_timeout = true;
    oldtabstate_rfid4.clear();
    cti_msgs::msg::TabState tabstate;
    tabstate.id = 4;
    tabstate.status = 140;  // 140 means recv timeout!
    tabstate.message = "";
    oldTabstate_Pushback(&tabstate, tabstate.id);
    oldtime_rfid4 = rclcpp::Clock().now().seconds();
    ////Info("RECV RFID: module_id_4 recv timeout!");
  } else {
    recv_rfid4_timeout = false;
  }
}

//************************************** rfid接收到信息后发布
void pub_rfidinfo(recv_from_rfid_info_type *data) {
  if (!data) {
    return;
    // printf("error!reveive empty data!\n");
  }

  rfid_single.header.frame_id = "rfid_single";
  rfid_all.header.frame_id = "rfid_all";
  cti_msgs::msg::TabState tabstate;
  tabstate.id = data->module_id;
  tabstate.status = data->status;
  tabstate.name = "rfid";
  oldTabstate_clear(data->module_id);
  recv_rfid_timeout(data->module_id);
  if (data->status == 1) {
    tabstate.status = data->read_num;
    rfid_single_Pushback(&tabstate, data->module_id);
    boxrfid_pub_single->publish(rfid_single);
    rfid_single.states.clear();
    oldTabstate_Pushback(&tabstate, data->module_id);
  }
  if (data->status == 0) {
    switch (data->read_num) {
      case 1: {
        tabstate.message = hex2string(&(data->data[2]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        break;
      }
      case 2: {
        tabstate.message = hex2string(&(data->data[2]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        tabstate.message = hex2string(&(data->data[12]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        break;
      }
      case 3: {
        tabstate.message = hex2string(&(data->data[2]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        tabstate.message = hex2string(&(data->data[12]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        tabstate.message = hex2string(&(data->data[22]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        break;
      }
      case 4: {
        tabstate.message = hex2string(&(data->data[2]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        tabstate.message = hex2string(&(data->data[12]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        tabstate.message = hex2string(&(data->data[22]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        tabstate.message = hex2string(&(data->data[32]), 8);
        rfid_single_Pushback(&tabstate, data->module_id);
        oldTabstate_Pushback(&tabstate, data->module_id);
        break;
      }
      default:
        break;
    }
    rfid_single.header.stamp = rclcpp::Clock().now();
    rfid_single.header.frame_id = "/fpga_serial/rfid_single";
    boxrfid_pub_single->publish(rfid_single);
    rfid_single.states.clear();
  }
  if (4 == data->module_id || recv_rfid4_timeout == true) {
    rfid_all.states.clear();
    rfid_all.header.stamp = rclcpp::Clock().now();
    rfid_all.header.frame_id = "/fpga_serial/rfid_all";
    rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid1.begin(),
                           oldtabstate_rfid1.end());
    rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid2.begin(),
                           oldtabstate_rfid2.end());
    rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid3.begin(),
                           oldtabstate_rfid3.end());
    rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid4.begin(),
                           oldtabstate_rfid4.end());
    boxrfid_pub_all->publish(rfid_all);
    rfid_all.states.clear();
  }
  std::string log_data = hex2string(&(data->data[2]), 8);
  /*
  //Info("RECV_RFID:" << " module_id:" << (int)data->module_id << "
  data_length:" << (int)data->data_length << " status:" << (int)data->status
  << " read_num:" << int(data->read_num)<< " data:" << log_data );
  */
}
//************************************** 接收到底盘错误码发布
void pub_chassiserror(recv_chassis_error_report_type *data) {
  if (!data) {
    return;
  }
  std_msgs::msg::UInt32MultiArray chassis_error;
  chassis_error.data.push_back((uint32_t)data->module_type);
  chassis_error.data.push_back(data->module_error_code);
  chassis_error.data.push_back((uint32_t)data->module_error_level);
  chassis_error_pub->publish(chassis_error);

  module_type_global = data->module_type;
  module_error_code_global = data->module_error_code;
  module_error_level_global = data->module_error_level;
}
//*************************************Timer5定时器，发布地盘错误码
void timer5Callback() {
  std_msgs::msg::UInt32MultiArray chassis_error;

  if (0 == module_type_global) {
    chassis_error.data.push_back(0);
  } else {
    chassis_error.data.push_back((uint32_t)module_type_global);
    module_type_global = 0;
  }

  if (0 == module_error_code_global) {
    chassis_error.data.push_back(0);
  } else {
    chassis_error.data.push_back(module_error_code_global);
    module_error_code_global = 0;
  }

  if (0 == module_error_level_global) {
    chassis_error.data.push_back(0);
  } else {
    chassis_error.data.push_back((uint32_t)module_error_level_global);
    module_error_level_global = 0;
  }
  chassis_error_pub->publish(chassis_error);
}
//************************************** 接收到底盘导航重要信息
void pub_navigationlog(recv_navigation_log_status_type *data) {
  if (!data) {
    return;
  }
  navigation_log.header.stamp = rclcpp::Clock().now();
  navigation_log.header.frame_id = "/fpga_serial/navigationlog";
  navigation_log.liner_speed = data->liner_speed;
  navigation_log.turn_angle = data->turn_angle;
  navigation_log.break_torque =
      data->break_torque;  //刹车力度，直接向驱动下发力矩值
  navigation_log.enable_flag = data->enable_flag;
  navigation_log.actual_body_linear_vel_x = data->actual_body_linear_vel_x;
  navigation_log.actual_speed_base_on_left_front_wheel =
      data->actual_speed_base_on_left_front_wheel;  //左前轮折算的车体中心速度
  navigation_log.actual_speed_base_on_right_front_wheel =
      data->actual_speed_base_on_right_front_wheel;
  navigation_log.actual_speed_base_on_left_rear_wheel =
      data->actual_speed_base_on_left_rear_wheel;
  navigation_log.actual_speed_base_on_right_rear_wheel =
      data->actual_speed_base_on_right_rear_wheel;
  navigation_log.actual_turn_front_angle =
      data->actual_turn_front_angle;  // unit:degree 前后转向电机的角度
  navigation_log.actual_turn_rear_angle =
      data->actual_turn_rear_angle;  // unit:degree
  for (int i = 0; i < (sizeof(data->set_torque) / sizeof(int16_t)); i++) {
    navigation_log.set_torque.push_back(data->set_torque[i]);
  }
  for (int i = 0; i < (sizeof(data->now_encoder) / sizeof(uint16_t)); i++) {
    navigation_log.now_encoder.push_back(data->set_torque[i]);
  }
  navigation_log_pub->publish(navigation_log);
  navigation_log.set_torque.clear();
  navigation_log.now_encoder.clear();
}
//************************************** 接收sd卡格式化结果后发布话题
void pub_formatsdcard(send_format_sd_card_cmd_type *data) {
  if (!data) {
    return;
  }
  formatsdcard_result.data = data->portIndex;
  formatsdcard_pub->publish(formatsdcard_result);
}
//************************************** 经纬度数据转化,度.分->度.度
double gps_data_trans(double data) {
  double data_temp = data;
  ;
  if (data < 0) {
    data_temp = -data_temp;
  }
  int int_part = (int)data_temp;
  double dec_part = data_temp - int_part;
  dec_part = dec_part * 100 / 60.0;
  double ret = dec_part + int_part;
  if (data < 0) {
    ret = -ret;
    return ret;
  } else {
    return ret;
  }
}
//************************************** 接收sd卡格式化结果后发布话题
void pub_gps(recv_gps_data_type *data) {
  if (!data) {
    return;
  }
  gps_data.header.stamp = rclcpp::Clock().now();
  gps_data.header.frame_id = "/chassis_gps";
  // gps utc time
  gps_data.utc_seconds =
      data->utc_seconds;  // utc时间,00:00:00至今的秒数,当前未使用

  // position
  switch (data->position_stat)  //位置解状态,NONE=无定位 SINGLE=单点定位
                                // PSFDIFF=伪距差分 NARROW_FLOAT=浮点解
                                // NARROW_INT=固定解
  {
    case 0:
    case 48:
      gps_data.position_stat = "NONE";
      break;
    case 49:
      gps_data.position_stat = "SINGLE";
      break;
    case 50:
      gps_data.position_stat = "PSFDIFF";
      break;
    case 51:
      gps_data.position_stat = "NONE";
      break;
    case 52:
      gps_data.position_stat = "NONE";
      break;
    case 53:
      gps_data.position_stat = "NARROW_FLOAT";
      break;
    case 54:
      gps_data.position_stat = "NARROW_INT";
      break;
    default:
      gps_data.position_stat = "UNDEFINED";
      break;
  }
  gps_data.lat = (data->lat) / 100;  //负数表示南半球,单位(度)
  gps_data.lat = gps_data_trans(gps_data.lat);
  gps_data.lon = (data->lon) / 100;  //负数表示西半球,单位(度)
  gps_data.lon = gps_data_trans(gps_data.lon);
  gps_data.alt =
      data->alt;  // WGS84 椭球高,若使用海拔高,则需根据undulation计算转换.
  gps_data.lat_err = data->lat_err;            //纬度标准差,单位(m)
  gps_data.lon_err = data->lon_err;            // 经度标准差,单位(m)
  gps_data.alt_err = data->alt_err;            //高程标准差,单位(m)
  gps_data.diff_age = data->diff_age;          // 差分龄,单位(s)
  gps_data.undulation = data->undulation;      // 海拔高 = alt - undulation
  gps_data.sats_tracked = data->sats_tracked;  // 跟踪到的卫星数
  gps_data.sats_used = data->sats_used;        // 解算中使用的卫星数

  // angle 主天线(moving)到副天线(heading)所形成的向量,真北即经线北向
  // position
  switch (data->heading_stat)  // 定向解状态,仅 SOL_COMPUTED 时表示定位成功
  {
    case 0:
      gps_data.heading_stat = "NONE";
      break;
    case 1:
      gps_data.heading_stat = "SOL_COMPUTED";
      break;
    default:
      gps_data.heading_stat = "UNDEFINED";
      break;
  }
  gps_data.heading =
      data->heading;  // 航向角(度),以为真北为起点,顺时针 0.0 - 359.99
  gps_data.pitch = data->pitch;  // 俯仰角(度),水平为0°,±90°
  gps_data.heading_err = data->heading_err;  //航向角标准差,单位(度)
  gps_data.pitch_err = data->pitch_err;      //俯仰角标准差,单位(度)
  gps_data.baselinelen = data->baselineLen;  // 两天线基线长

  // velocity
  switch (data->velocity_stat)  // 速度解状态,仅 SOL_COMPUTED 时表示定位成功
  {
    case 0:
      gps_data.velocity_stat = "NONE";
      break;
    case 1:
      gps_data.velocity_stat = "SOL_COMPUTED";
      break;
    default:
      gps_data.velocity_stat = "UNDEFINED";
      break;
  }
  gps_data.speed_north =
      (data->speed_north) / 100;  // 东北天系下各速度分量,单位(m/s)
  gps_data.speed_east = (data->speed_east) / 100;
  gps_data.speed_up = (data->speed_up) / 100;
  gps_data.latency = data->latency;  // 速度延迟,单位(s)

  gps_pub->publish(gps_data);
}
//************************************** 定位状态命令话题回调函数
void localizerState_Callback(
    const cti_msgs::msg::RobotLocalizerState::SharedPtr msg) {
  if (localization_limit) {
    robotlocalizerstate_global = msg->id;
  } else {
    robotlocalizerstate_global = 2;
  }
}
//************************************** 发送细时间同步命令
void send_fine_sync_cmd() {
  //获取到微秒级别
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);

  time_sync_fine_msg_type timenow_cmd;

  // printf("tv_usec:%d\n",tv.tv_usec);
  timenow_cmd.sec = tv.tv_sec % 60;
  // printf("send_fine_sec:%d\n",timenow_cmd.sec);
  timenow_cmd.millisencond = (tv.tv_usec) / 1000;
  // printf("send_fine_millisec:%d\n",timenow_cmd.millisencond);
  timenow_cmd.microsecondsec = (tv.tv_usec) % 1000;
  // printf("send_fine_microsec:%d\n",timenow_cmd.microsecondsec);
  construct_serial_frame_ex(&user_frame, TIME_SYNC_FINE_CMD,
                            sizeof(timenow_cmd), &timenow_cmd);
  serial_frame_include_id_type user_frame_include_id_0;
  user_frame_include_id_0.id = 0;
  user_frame_include_id_0.cmd = TIME_SYNC_FINE_CMD;
  user_frame_include_id_0.need_id = 0;
  user_frame_include_id_0.frame = user_frame;
  pushData(&user_frame_include_id_0);
  // printf("________________________________________\n");
  // printf("Send_fine_sync_req::33>>>>>>>>>>>>>>>>>>>>>>\n");
}
//************************************** 发送延时时间同步命令
void send_delay_sync_cmd() {
  //获取到微秒级别
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);

  time_sync_fine_msg_type timenow_cmd;

  timenow_cmd.sec = tv.tv_sec % 60;
  timenow_cmd.millisencond = (tv.tv_usec) / 1000;
  timenow_cmd.microsecondsec = (tv.tv_usec) % 1000;
  construct_serial_frame_ex(&user_frame, TIME_SYNC_DELAY_CMD,
                            sizeof(timenow_cmd), &timenow_cmd);
  serial_frame_include_id_type user_frame_include_id_0;
  user_frame_include_id_0.id = 0;
  user_frame_include_id_0.cmd = TIME_SYNC_DELAY_CMD;
  user_frame_include_id_0.need_id = 0;
  user_frame_include_id_0.frame = user_frame;
  pushData(&user_frame_include_id_0);
  // printf("Send_delay_sync_req::34>>>>>>>>>>>>>>>>>>>>>>\n");
}
//************************************** 粗时间同步回应处理函数
void check_rough_res(time_sync_rough_msg_type *data) {
  if (data->year == sync_rough_msg_saved.year &&
      data->mon == sync_rough_msg_saved.mon &&
      data->day == sync_rough_msg_saved.day &&
      data->hour == sync_rough_msg_saved.hour &&
      data->min == sync_rough_msg_saved.min &&
      data->sec == sync_rough_msg_saved.sec) {
    //发送细时间同步
    need_send_fine_sync = true;
    //取消粗同步回应超时检测
    check_rough_sync_res_timeoout = false;
    check_rough_sync_res_cnt = 0;
    need_send_rough = false;
    // printf("rough_sync_successed@@@@@@@@@@@@@@@\n");
  } else {
    need_resend_rough = true;
  }
}

//************************************** 清扫箱状态发布
void pub_dustbin_state(dustbin_to_motion_t *data) {
  cti_msgs::msg::DustbinState dustbin_state_msg;
  dustbin_state_msg.header.stamp = rclcpp::Clock().now();
  dustbin_state_msg.header.frame_id = "/chassis_dustbin";

  dustbin_state_msg.water_tank_top = data->water_tank_top;
  dustbin_state_msg.water_tank_bottom = data->water_tank_bottom;
  dustbin_state_msg.dustbin_tail_cover = data->dustbin_tail_cover;
  dustbin_state_msg.dustbin_distance = data->dustbin_distance;
  dustbin_state_msg.motor_status = data->motor_status.data;

  for (int i = 0; i < sizeof(data->ultrasonic_distance) / sizeof(uint16_t);
       i++) {
    dustbin_state_msg.ultrasonic_distance.push_back(
        data->ultrasonic_distance[i]);
  }
  dustbin_state_msg.voltage1 = data->voltage1;
  dustbin_state_msg.voltage2 = data->voltage2;
  dustbin_state_msg.voltage3 = data->voltage3;

  dustbin_state_pub->publish(dustbin_state_msg);

  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustVehicleInfos;
  dustVehicleInfos.header.stamp = rclcpp::Clock().now();
  dustVehicleInfos.header.frame_id = "sanitation_vehicle";
  //水箱高水位
  cti_msgs::msg::Data dustVehicleInfo;
  dustVehicleInfo.name = "water_tank_top";
  dustVehicleInfo.data = std::to_string(data->water_tank_top);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //水箱低水位
  dustVehicleInfo.name = "water_tank_bottom";
  dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //电机状态
  dustVehicleInfo.name = "motor_status";
  dustVehicleInfo.data = std::to_string(data->motor_status.data);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  dust_vehicle_state_info_pub->publish(dustVehicleInfos);  //发布新的消息类型

  //发布超声波数据
  if (data->ultrasonic_distance[0] == 0xffff) {  //新版超声波
    sensor_msgs::msg::Range new_sweeperRangeData;
    int index = (data->ultrasonic_distance[1]) - 1;
    const float sweeper_min_range = 0.12;
    const float sweeper_max_range = 2.5;
    new_sweeperRangeData.header.stamp = rclcpp::Clock().now();
    new_sweeperRangeData.radiation_type = 0;  // ult
    new_sweeperRangeData.min_range = sweeper_min_range;
    new_sweeperRangeData.max_range = msg_max_range;
    new_sweeperRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < new_sweeper_max_type_ult; i++) {
      float ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
      new_sweeperRangeData.range = ult_data;
      new_sweeperRangeData.header.frame_id = new_sweeper_ult_name[index][i];
      new_sweeper_range_pub[index][i]->publish(new_sweeperRangeData);
    }
  } else {  //旧版超声波
    const float sweeper_min_range = 0.12;
    const float sweeper_max_range = 2.5;
    sweeperRangeData.header.stamp = rclcpp::Clock().now();
    sweeperRangeData.radiation_type = 0;  // ult
    sweeperRangeData.min_range = sweeper_min_range;
    sweeperRangeData.max_range = msg_max_range;
    sweeperRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < sweeper_max_type_ult; i++) {
      float ult_data = (float)data->ultrasonic_distance[i] / 1000.f;
      sweeperRangeData.range = ult_data;
      sweeperRangeData.header.frame_id = sweeper_ult_name[i];
      sweeper_range_pub[i]->publish(sweeperRangeData);
    }
  }
  Info("R_DBI_ES: " << (int)data->motor_status.data
                    << " R_DBI_WT: " << (int)data->water_tank_top
                    << " R_DBI_WB: " << (int)data->water_tank_bottom
                    << " R_DBI_TC: " << (int)data->dustbin_tail_cover
                    << " R_DBI_DS: " << (int)data->dustbin_distance
                    << " R_DBI_DOC: " << (int)data->recv_dustbin_on_the_car);

  //判断清扫车在车上的状态判断
  if (set_dustbin_on_car) {
    if ((data->recv_dustbin_on_the_car == 1 && box_type == 4) ||
        (data->recv_dustbin_on_the_car == 0 && box_type != 4)) {
      set_dustbin_on_car = false;
    }
  }
  //-------------------通讯检测------------------
  box_chat_state.module_id = 1;
  box_chat_state.recv_state = true;
  box_chat_state.time_recv = rclcpp::Clock().now().seconds();
}

//************************************** 环卫车清扫状态发布_旧的,没有加边刷伸缩
void pub_clean_state(clean_to_motion_t *data) {
  cti_msgs::msg::DustbinState clean_state_msg;
  clean_state_msg.header.stamp = rclcpp::Clock().now();
  clean_state_msg.header.frame_id = "/chassis_dustbin";

  clean_state_msg.water_tank_top = data->water_tank_top;
  clean_state_msg.water_tank_bottom = data->water_tank_bottom;
  clean_state_msg.dustbin_tail_cover = data->dustbin_tail_cover;
  clean_state_msg.dustbin_distance = data->dustbin_distance;
  clean_state_msg.motor_status = data->motor_status.data;

  for (int i = 0; i < sizeof(data->ultrasonic_distance) / sizeof(uint16_t);
       i++) {
    clean_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
  }
  clean_state_msg.voltage1 = data->voltage1;
  clean_state_msg.voltage2 = data->voltage2;
  clean_state_msg.voltage3 = data->voltage3;

  dustbin_state_pub->publish(clean_state_msg);

  //用cti_msgs::msg::dataArray 类型发布消息
  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustVehicleInfos;
  dustVehicleInfos.header.stamp = rclcpp::Clock().now();
  dustVehicleInfos.header.frame_id = "sanitation_vehicle";
  //水箱高水位
  cti_msgs::msg::Data dustVehicleInfo;
  dustVehicleInfo.name = "water_tank_top";
  dustVehicleInfo.data = std::to_string(data->water_tank_top);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //水箱低水位
  dustVehicleInfo.name = "water_tank_bottom";
  dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //电机状态
  dustVehicleInfo.name = "motor_status";
  dustVehicleInfo.data = std::to_string(data->motor_status.data);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //挡板状态
  dustVehicleInfo.name = "dam_board_status";
  dustVehicleInfo.data = std::to_string(data->dam_board_status);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //存储上水位传感器数据
  vehicle_water_tank_top.push_back(data->water_tank_top);
  vehicle_water_tank_bottom.push_back(data->water_tank_bottom);
  if (vehicle_water_tank_top.size() > max_vehicle_water_tank_restore) {
    vehicle_water_tank_top.pop_front();
  }
  if (vehicle_water_tank_bottom.size() > max_vehicle_water_tank_restore) {
    vehicle_water_tank_bottom.pop_front();
  }
  bool water_full = true;   //默认水满
  bool water_empty = true;  //默认水空
  for (int i = 0; i < max_vehicle_water_tank_restore - 1; i++) {
    if (1 == vehicle_water_tank_top[i]) {
      water_full = false;  //队列中有一个 == 1,就认为水没有满
      break;
    }
  }
  for (int i = 0; i < max_vehicle_water_tank_restore - 1; i++) {
    if (1 == vehicle_water_tank_bottom[i]) {
      water_empty = false;  //队列中有一个 == 1,就认为水没有空
      break;
    }
  }
  //计算水量百分比
  if (31 == data->motor_status.data) {
    if (false == vehicle_water_status.is_out) {
      vehicle_water_status.is_out = true;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  } else if (27 == data->motor_status.data || 0 == data->motor_status.data) {
    if (true == vehicle_water_status.is_out) {
      vehicle_water_status.is_out = false;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  }
  if (water_full) {
    vehicle_water_status.water_percnt_now = 100.0;
  } else if (water_empty) {
    vehicle_water_status.water_percnt_now = 0.0;
  } else {
    if (true == vehicle_water_status.is_out) {
      double time_now = rclcpp::Clock().now().seconds();
      double duration = time_now - vehicle_water_status.out_pre_time;
      vehicle_water_status.water_percnt_now =
          vehicle_water_status.water_percnt_now -
          duration * vehicle_water_status.out_per_sec;
      if (vehicle_water_status.water_percnt_now > 90.0) {
        vehicle_water_status.water_percnt_now = 90.0;
      }
      if (vehicle_water_status.water_percnt_now < 10.0) {
        vehicle_water_status.water_percnt_now = 10.0;
      }
      vehicle_water_status.out_pre_time = time_now;
    }
  }
  //水量百分比
  dustVehicleInfo.name = "water_percent";
  dustVehicleInfo.data =
      std::to_string((int)vehicle_water_status.water_percnt_now);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  std::string modify_file =
      "echo " + std::to_string(vehicle_water_status.water_percnt_now) + " > " +
      config_file_path;
  FILE *fpr = NULL;
  fpr = popen(modify_file.c_str(), "w");
  pclose(fpr);

  //消息发布
  dust_vehicle_state_info_pub->publish(dustVehicleInfos);

  //发布超声波数据
  if (data->ultrasonic_distance[0] == 0xffff) {  //新版超声波
    sensor_msgs::msg::Range new_sweeperRangeData;
    int index = (data->ultrasonic_distance[1]) - 1;
    const float sweeper_min_range = 0.12;
    const float sweeper_max_range = 2.5;
    new_sweeperRangeData.header.stamp = rclcpp::Clock().now();
    new_sweeperRangeData.radiation_type = 0;  // ult
    new_sweeperRangeData.min_range = sweeper_min_range;
    new_sweeperRangeData.max_range = msg_max_range;
    new_sweeperRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < new_sweeper_max_type_ult; i++) {
      float ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
      new_sweeperRangeData.range = ult_data;
      new_sweeperRangeData.header.frame_id = new_sweeper_ult_name[index][i];
      new_sweeper_range_pub[index][i]->publish(new_sweeperRangeData);
    }
  } else {  //旧版超声波
    const float sweeper_min_range = 0.12;
    const float sweeper_max_range = 2.5;
    sweeperRangeData.header.stamp = rclcpp::Clock().now();
    sweeperRangeData.radiation_type = 0;  // ult
    sweeperRangeData.min_range = sweeper_min_range;
    sweeperRangeData.max_range = msg_max_range;
    sweeperRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < sweeper_max_type_ult; i++) {
      float ult_data = (float)data->ultrasonic_distance[i] / 1000.f;
      sweeperRangeData.range = ult_data;
      sweeperRangeData.header.frame_id = sweeper_ult_name[i];
      sweeper_range_pub[i]->publish(sweeperRangeData);
    }
  }
  Info("R_CC_ES: " << (int)data->motor_status.data
                   << " R_CC_WT: " << (int)data->water_tank_top
                   << " R_CC_WB: " << (int)data->water_tank_bottom
                   << " R_CC_TC: " << (int)data->dustbin_tail_cover
                   << " R_CC_DS: " << (int)data->dustbin_distance
                   << " R_CC_DOC: " << (int)data->recv_dustbin_on_the_car
                   << " R_CC_DB: " << (int)data->dam_board_status);
  recv_clean_mechine_motor_status = data->motor_status.data;

  //判断清扫车在车上的状态判断
  if (set_dustbin_on_car) {
    if ((data->recv_dustbin_on_the_car == 1 && box_type == 4) ||
        (data->recv_dustbin_on_the_car == 0 && box_type != 4)) {
      set_dustbin_on_car = false;
    }
  }
  //发布挡板状态
  std_msgs::msg::UInt8 dustbin_damboard_msg;
  dustbin_damboard_msg.data = data->dam_board_status;
  dustbin_damboard_pub->publish(dustbin_damboard_msg);
}

//************************************** 环卫车清扫状态发布_新的,加边刷伸缩
void pub_clean_state_new(clean_to_motion_t_new *data) {
  //发布旧的消息类型
  cti_msgs::msg::DustbinState clean_state_msg;
  clean_state_msg.header.stamp = rclcpp::Clock().now();
  clean_state_msg.header.frame_id = "/chassis_dustbin";

  clean_state_msg.water_tank_top = data->water_tank_top;
  clean_state_msg.water_tank_bottom = data->water_tank_bottom;
  clean_state_msg.dustbin_tail_cover = data->dustbin_tail_cover;
  clean_state_msg.dustbin_distance = data->dustbin_distance;
  clean_state_msg.motor_status = data->motor_status.data;

  for (int i = 0; i < sizeof(data->ultrasonic_distance) / sizeof(uint16_t);
       i++) {
    clean_state_msg.ultrasonic_distance.push_back(data->ultrasonic_distance[i]);
  }
  clean_state_msg.voltage1 = data->voltage1;
  clean_state_msg.voltage2 = data->voltage2;
  clean_state_msg.voltage3 = data->voltage3;

  dustbin_state_pub->publish(clean_state_msg);

  //发布新的消息类型
  cti_msgs::msg::DustbinStateNew clean_state_msg_new;
  clean_state_msg_new.header.stamp = rclcpp::Clock().now();
  clean_state_msg_new.header.frame_id = "/chassis_dustbin";

  clean_state_msg_new.water_tank_top = data->water_tank_top;
  clean_state_msg_new.water_tank_bottom = data->water_tank_bottom;
  clean_state_msg_new.dustbin_tail_cover = data->dustbin_tail_cover;
  clean_state_msg_new.dustbin_distance = data->dustbin_distance;
  clean_state_msg_new.motor_status = data->motor_status.data;

  for (int i = 0; i < sizeof(data->ultrasonic_distance) / sizeof(uint16_t);
       i++) {
    clean_state_msg_new.ultrasonic_distance.push_back(
        data->ultrasonic_distance[i]);
  }
  clean_state_msg_new.voltage1 = data->voltage1;
  clean_state_msg_new.voltage2 = data->voltage2;
  clean_state_msg_new.voltage3 = data->voltage3;
  clean_state_msg_new.recv_dustbin_on_the_car = data->recv_dustbin_on_the_car;
  clean_state_msg_new.dam_board_status = data->dam_board_status;
  clean_state_msg_new.side_brush_transform_state =
      data->side_brush_transform_state;
  clean_state_msg_new.side_brush_speed = data->side_brush_speed;
  clean_state_msg_new.error_code = data->error_code;
  clean_state_msg_new.unused0 = data->side_brush_transform_error;
  clean_state_msg_new.unused1 = data->left_side_brush_error;
  clean_state_msg_new.unused2 = data->right_side_brush_error;
  clean_state_msg_new.unused3 = data->multi_status.data;

  dustbin_state_pub_new->publish(clean_state_msg_new);

  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustVehicleInfos;
  dustVehicleInfos.header.stamp = rclcpp::Clock().now();
  dustVehicleInfos.header.frame_id = "sanitation_vehicle";
  //水箱高水位
  cti_msgs::msg::Data dustVehicleInfo;
  dustVehicleInfo.name = "water_tank_top";
  dustVehicleInfo.data = std::to_string(data->water_tank_top);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //水箱低水位
  dustVehicleInfo.name = "water_tank_bottom";
  dustVehicleInfo.data = std::to_string(data->water_tank_bottom);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //电机状态
  dustVehicleInfo.name = "motor_status";
  dustVehicleInfo.data = std::to_string(data->motor_status.data);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //挡板状态
  dustVehicleInfo.name = "dam_board_status";
  dustVehicleInfo.data = std::to_string(data->dam_board_status);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //边刷伸展
  dustVehicleInfo.name = "sidebrush_state";
  dustVehicleInfo.data = std::to_string(data->side_brush_transform_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //边刷旋转速度
  dustVehicleInfo.name = "sidebrush_speed";
  dustVehicleInfo.data = std::to_string(data->side_brush_speed);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //清扫功能错误码
  dustVehicleInfo.name = "error_code";
  dustVehicleInfo.data = std::to_string(data->error_code);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT32;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //边刷伸展错误码
  dustVehicleInfo.name = "sidebrush_transform_error";
  dustVehicleInfo.data = std::to_string(data->side_brush_transform_error);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //左边刷错误码
  dustVehicleInfo.name = "left_sidebrush_error";
  dustVehicleInfo.data = std::to_string(data->left_side_brush_error);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //右边刷错误码
  dustVehicleInfo.name = "right_sidebrush_error";
  dustVehicleInfo.data = std::to_string(data->right_side_brush_error);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  //加水水泵状态
  dustVehicleInfo.name = "increase_water_pump";
  uint8_t increase_water_pump_state =
      data->multi_status.bits.increase_water_pump;
  dustVehicleInfo.data = std::to_string(increase_water_pump_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //震尘速度
  dustVehicleInfo.name = "shake_dust_motor_speed_feedback";
  dustVehicleInfo.data = std::to_string(data->shake_dust_motor_speed_feedback);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //滚刷速度
  dustVehicleInfo.name = "roll_brush_speed";
  dustVehicleInfo.data = std::to_string(data->roll_brush_speed);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //滚刷错误码
  dustVehicleInfo.name = "roll_brush_error_code";
  dustVehicleInfo.data = std::to_string(data->roll_brush_error_code);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //边刷上升状态
  dustVehicleInfo.name = "lift_motorr_state";
  dustVehicleInfo.data = std::to_string(data->lift_motorr_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //控制模式反馈状态
  dustVehicleInfo.name = "control_mode_state";
  dustVehicleInfo.data = std::to_string(data->control_mode_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //
  dustVehicleInfo.name = "push_board_state";
  std::string data_te;
  for (int i = 9; i > 0; i--) {
    data_te = data_te + std::to_string(data->push_board_state[i]) + ",";
  }
  dustVehicleInfo.data = data_te;
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //
  dustVehicleInfo.name = "push_board_error_code";
  std::string data_te2;
  for (int ii = 9; ii > 0; ii--) {
    data_te2 = data_te2 + std::to_string(data->push_board_error_code[ii]) + ",";
  }
  dustVehicleInfo.data = data_te2;
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //存储上水位传感器数据
  vehicle_water_tank_top.push_back(data->water_tank_top);
  vehicle_water_tank_bottom.push_back(data->water_tank_bottom);
  if (vehicle_water_tank_top.size() > max_vehicle_water_tank_restore) {
    vehicle_water_tank_top.pop_front();
  }
  if (vehicle_water_tank_bottom.size() > max_vehicle_water_tank_restore) {
    vehicle_water_tank_bottom.pop_front();
  }
  bool water_full = true;   //默认水满
  bool water_empty = true;  //默认水空
  for (int i = 0; i < max_vehicle_water_tank_restore - 1; i++) {
    if (1 == vehicle_water_tank_top[i]) {
      water_full = false;  //队列中有一个 == 1,就认为水没有满
      break;
    }
  }
  for (int i = 0; i < max_vehicle_water_tank_restore - 1; i++) {
    if (1 == vehicle_water_tank_bottom[i]) {
      water_empty = false;  //队列中有一个 == 1,就认为水没有空
      break;
    }
  }
  //计算水量百分比
  //排水状态
  if (31 == data->motor_status.data) {
    if (false == vehicle_water_status.is_out) {
      vehicle_water_status.is_out = true;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  } else if (27 == data->motor_status.data || 0 == data->motor_status.data) {
    if (true == vehicle_water_status.is_out) {
      vehicle_water_status.is_out = false;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  }
  //进水状态
  if (1 == increase_water_pump_state) {
    if (false == vehicle_water_status.is_in) {
      vehicle_water_status.is_in = true;
      vehicle_water_status.in_pre_time = rclcpp::Clock().now().seconds();
    }
  } else if (0 == increase_water_pump_state) {
    if (true == vehicle_water_status.is_in) {
      vehicle_water_status.is_out = false;
      vehicle_water_status.out_pre_time = rclcpp::Clock().now().seconds();
    }
  }

  if (water_full) {
    vehicle_water_status.water_percnt_now = 100.0;
  } else if (water_empty) {
    vehicle_water_status.water_percnt_now = 0.0;
  } else {
    //排水计算
    if (true == vehicle_water_status.is_out) {
      double time_now = rclcpp::Clock().now().seconds();
      double duration = time_now - vehicle_water_status.out_pre_time;
      vehicle_water_status.water_percnt_now =
          vehicle_water_status.water_percnt_now -
          duration * vehicle_water_status.out_per_sec;
      if (vehicle_water_status.water_percnt_now > 90.0) {
        vehicle_water_status.water_percnt_now = 90.0;
      }
      if (vehicle_water_status.water_percnt_now < 10.0) {
        vehicle_water_status.water_percnt_now = 10.0;
      }
      vehicle_water_status.out_pre_time = time_now;
    }
    //加水计算
    if (true == vehicle_water_status.is_in) {
      double time_now = rclcpp::Clock().now().seconds();
      double duration = time_now - vehicle_water_status.in_pre_time;
      vehicle_water_status.water_percnt_now =
          vehicle_water_status.water_percnt_now +
          duration * vehicle_water_status.in_per_sec;
      if (vehicle_water_status.water_percnt_now > 90.0) {
        vehicle_water_status.water_percnt_now = 90.0;
      }
      if (vehicle_water_status.water_percnt_now < 10.0) {
        vehicle_water_status.water_percnt_now = 10.0;
      }
      vehicle_water_status.out_pre_time = time_now;
    }
  }

  //水量百分比
  dustVehicleInfo.name = "water_percent";
  dustVehicleInfo.data =
      std::to_string(int(vehicle_water_status.water_percnt_now));
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);
  std::string modify_file =
      "echo " + std::to_string(vehicle_water_status.water_percnt_now) + " > " +
      config_file_path;
  FILE *fpr = NULL;
  fpr = popen(modify_file.c_str(), "w");
  pclose(fpr);
  //装饰灯状态发布
  dustVehicleInfo.name = "decorate_light";
  uint8_t decorate_light_state = data->multi_status.bits.decorate_light;
  dustVehicleInfo.data = std::to_string(decorate_light_state);
  dustVehicleInfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustVehicleInfos.datas.push_back(dustVehicleInfo);

  //消息发布
  dust_vehicle_state_info_pub->publish(dustVehicleInfos);

  //发布超声波数据
  if (data->ultrasonic_distance[0] == 0xffff) {  //新版超声波
    sensor_msgs::msg::Range new_sweeperRangeData;
    int index = (data->ultrasonic_distance[1]) - 1;
    const float sweeper_min_range = 0.12;
    const float sweeper_max_range = 2.5;
    new_sweeperRangeData.header.stamp = rclcpp::Clock().now();
    new_sweeperRangeData.radiation_type = 0;  // ult
    new_sweeperRangeData.min_range = sweeper_min_range;
    new_sweeperRangeData.max_range = msg_max_range;
    new_sweeperRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < new_sweeper_max_type_ult; i++) {
      float ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
      new_sweeperRangeData.range = ult_data;
      new_sweeperRangeData.header.frame_id = new_sweeper_ult_name[index][i];
      new_sweeper_range_pub[index][i]->publish(new_sweeperRangeData);
    }
  } else {  //旧版超声波
    const float sweeper_min_range = 0.12;
    const float sweeper_max_range = 2.5;
    sweeperRangeData.header.stamp = rclcpp::Clock().now();
    sweeperRangeData.radiation_type = 0;  // ult
    sweeperRangeData.min_range = sweeper_min_range;
    sweeperRangeData.max_range = msg_max_range;
    sweeperRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < sweeper_max_type_ult; i++) {
      float ult_data = (float)data->ultrasonic_distance[i] / 1000.f;
      sweeperRangeData.range = ult_data;
      sweeperRangeData.header.frame_id = sweeper_ult_name[i];
      sweeper_range_pub[i]->publish(sweeperRangeData);
    }
  }
  Info("R_CC_ES: " << (int)data->motor_status.data
                   << " R_CC_WT: " << (int)data->water_tank_top
                   << " R_CC_WB: " << (int)data->water_tank_bottom
                   << " R_CC_TC: " << (int)data->dustbin_tail_cover
                   << " R_CC_DS: " << (int)data->dustbin_distance
                   << " R_CC_DOC: " << (int)data->recv_dustbin_on_the_car
                   << " R_CC_DB: " << (int)data->dam_board_status
                   << " R_CC_SBT: " << (int)data->side_brush_transform_state
                   << " R_CC_SBS: " << (int)data->side_brush_speed
                   << " R_CC_ERR: " << (int)data->error_code
                   << " R_CC_SBTE: " << (int)data->side_brush_transform_error
                   << " R_CC_LSBE: " << (int)data->left_side_brush_error
                   << " R_CC_RSBE: " << (int)data->right_side_brush_error
                   << " R_CC_WP: " << (int)vehicle_water_status.water_percnt_now
                   << " R_CC_DE_LG: " << (int)decorate_light_state
                   << " R_CC_IN_PU: " << (int)increase_water_pump_state);
  recv_clean_mechine_motor_status = data->motor_status.data;

  //判断清扫车在车上的状态判断
  if (set_dustbin_on_car) {
    if ((data->recv_dustbin_on_the_car == 1 && box_type == 4) ||
        (data->recv_dustbin_on_the_car == 0 && box_type != 4)) {
      set_dustbin_on_car = false;
    }
  }
  //发布挡板状态
  std_msgs::msg::UInt8 dustbin_damboard_msg;
  dustbin_damboard_msg.data = data->dam_board_status;
  dustbin_damboard_pub->publish(dustbin_damboard_msg);
}
//**************************************
//集尘箱状态发布(旧的,没有电池,风机力矩等)
void pub_dust_box_state_old(dust_box_to_motion_t_old *data) {
  cti_msgs::msg::DustbinState dust_box_state_msg;
  dust_box_state_msg.header.stamp = rclcpp::Clock().now();
  dust_box_state_msg.header.frame_id = "/chassis_dust_box";

  dust_box_state_msg.water_tank_top = data->water_tank_top;
  dust_box_state_msg.water_tank_bottom = data->water_tank_bottom;
  dust_box_state_msg.dustbin_distance = data->dustbin_distance;
  int ultrasonic_distance_length =
      sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
  for (int i = 0; i < ultrasonic_distance_length; i++) {
    dust_box_state_msg.ultrasonic_distance.push_back(
        data->ultrasonic_distance[i]);
  }
  dust_box_state_msg.voltage1 = data->voltage1;
  dust_box_state_msg.voltage2 = data->voltage2;
  dust_box_state_msg.voltage3 = data->voltage3;
  dust_box_state_msg.motor_status = data->motor_status.data;
  dust_box_state_pub->publish(dust_box_state_msg);

  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustBoxInfos;
  dustBoxInfos.header.stamp = rclcpp::Clock().now();
  dustBoxInfos.header.frame_id = "sanitation_dust_box";

  cti_msgs::msg::Data dustboxinfo;
  //垃圾到顶盖的距离
  dustboxinfo.name = "dust_distance";
  dustboxinfo.data = std::to_string(data->dustbin_distance);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池1的电压
  dustboxinfo.name = "bat1_vol";
  dustboxinfo.data = std::to_string(data->voltage1);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池2的电压
  dustboxinfo.name = "bat2_vol";
  dustboxinfo.data = std::to_string(data->voltage2);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池3的电压
  dustboxinfo.name = "bat3_vol";
  dustboxinfo.data = std::to_string(data->voltage3);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //吸尘箱启动状态
  dustboxinfo.name = "motor_status";
  dustboxinfo.data = std::to_string(data->motor_status.data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //自动倾倒垃圾的状态
  dustboxinfo.name = "dust_autopush";
  dustboxinfo.data = std::to_string(data->auto_push);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机速度,控制反馈
  dustboxinfo.name = "fan_Speed";
  dustboxinfo.data = std::to_string(data->fan_speed);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //垃圾量百分比
  dustboxinfo.name = "trash_percent";
  dustboxinfo.data = std::to_string(data->trash_per);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //消息发布
  dust_box_state_info_pub->publish(dustBoxInfos);

  //箱尾超声波发布  //v6.0 v6.1为四个超声波都在箱子后面 v6.2
  // 1-3号超声波在箱子后面 4号在垃圾仓里 位于后方的原始数据发布
  std_msgs::msg::UInt16MultiArray dustbox_rear_ult;
  int size =
      sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
  for (int i = 0; i < size; i++) {
    dustbox_rear_ult.data.push_back(data->ultrasonic_distance[i]);
  }
  dustbox_rear_range_pub->publish(dustbox_rear_ult);
  if (cti_run_ver != "v6.0" && cti_run_ver != "v6.1") {
    sensor_msgs::msg::Range dustboxrRangeData;
    const float dustbox_min_range = 0.12;
    const float dustbox_max_range = 2.5;
    dustboxrRangeData.header.stamp = rclcpp::Clock().now();
    dustboxrRangeData.radiation_type = 0;  // ult
    dustboxrRangeData.min_range = dustbox_min_range;
    dustboxrRangeData.max_range = msg_max_range;
    dustboxrRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < dustbox_max_type_ult; i++) {
      float ult_data;
      if (0 == data->auto_push) {
        ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
      } else {
        ult_data =
            8.88;  // 8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
      }
      dustboxrRangeData.range = ult_data;
      dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
      dustbox_range_pub[i]->publish(dustboxrRangeData);
    }
    float ult_data;
    ult_data = (float)data->ultrasonic_distance[5] / 1000.f;
    dustboxrRangeData.range = ult_data;
    dustboxrRangeData.header.frame_id = "bottom";
    dustbox_bottom_range_pub->publish(dustboxrRangeData);
  } else {
    sensor_msgs::msg::Range dustboxrRangeData;
    const float dustbox_min_range = 0.12;
    const float dustbox_max_range = 2.5;
    dustboxrRangeData.header.stamp = rclcpp::Clock().now();
    dustboxrRangeData.radiation_type = 0;  // ult
    dustboxrRangeData.min_range = dustbox_min_range;
    dustboxrRangeData.max_range = msg_max_range;
    dustboxrRangeData.field_of_view = 24 * M_PI / 180.0f;
    for (int i = 0; i < dustbox_max_type_ult; i++) {
      float ult_data;
      if (0 == data->auto_push) {
        ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
      } else {
        ult_data =
            8.88;  // 8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
      }
      dustboxrRangeData.range = ult_data;
      dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
      dustbox_range_pub[i]->publish(dustboxrRangeData);
    }
  }
  //自动倒垃圾状态
  std_msgs::msg::UInt8 dust_box_autopush_msg;
  dust_box_autopush_msg.data = data->auto_push;
  dust_box_autopush_pub->publish(dust_box_autopush_msg);

  //风机速度
  std_msgs::msg::UInt8 dust_box_fanspeed_msg;
  dust_box_fanspeed_msg.data = data->fan_speed;
  dust_box_fanspeed_pub->publish(dust_box_fanspeed_msg);

  //无线充电
  cti_msgs::msg::BatteryState wireless_charge_state_msg;
  wireless_charge_state_msg.header.stamp = rclcpp::Clock().now();
  wireless_charge_state_msg.wireless_install_state = 1;
  wireless_charge_state_msg.wireless_voltage = data->wireless_voltage;
  wireless_charge_state_msg.wireless_current = data->wireless_current;
  wireless_charge_state_msg.wireless_reserve = data->wireless_reserve;
  wireless_charge_state_msg.wireless_state = data->wireless_state;
  wireless_charge_state_msg.wireless_temp = data->wireless_temp;
  wireless_charge_state_msg.wireless_changer = data->wireless_changer;
  dustbox_wireless_charge_state_pub->publish(wireless_charge_state_msg);

  //使用json发布状态
  Json::Value dustboxStateJson;
  Json::Value wirelessChargeJson;
  Json::FastWriter writer;
  dustboxStateJson["stamp"] = std::to_string(rclcpp::Clock().now().seconds());
  dustboxStateJson["portIndex"] = std::to_string(data->portIndex);
  dustboxStateJson["water_tank_top"] = std::to_string(data->water_tank_top);
  dustboxStateJson["dustbin_distance"] = std::to_string(data->dustbin_distance);
  dustboxStateJson["voltage1"] = std::to_string(data->voltage1);
  dustboxStateJson["voltage2"] = std::to_string(data->voltage2);
  dustboxStateJson["voltage3"] = std::to_string(data->voltage3);
  dustboxStateJson["motor_status"] = std::to_string(data->motor_status.data);
  dustboxStateJson["recv_dustbin_on_the_car"] =
      std::to_string(data->recv_dustbin_on_the_car);
  dustboxStateJson["auto_push"] = std::to_string(data->auto_push);
  dustboxStateJson["fan_speed"] = std::to_string(data->fan_speed);
  //无线充电状态start
  wirelessChargeJson["wireless_voltage"] =
      std::to_string(data->wireless_voltage);
  wirelessChargeJson["wireless_current"] =
      std::to_string(data->wireless_current);
  wirelessChargeJson["wireless_reserve"] =
      std::to_string(data->wireless_reserve);
  wirelessChargeJson["wireless_state"] = std::to_string(data->wireless_state);
  wirelessChargeJson["wireless_temp"] = std::to_string(data->wireless_temp);
  wirelessChargeJson["wireless_charger"] =
      std::to_string(data->wireless_changer);
  dustboxStateJson["wireless_charge"] = wirelessChargeJson;
  //无线充电状态 end
  std_msgs::msg::String dustboxStateStr;
  dustboxStateStr.data = Json::FastWriter().write(dustboxStateJson);
  dust_box_state_pub_json->publish(dustboxStateStr);

  Info("R_DBO_ES: " << (int)data->motor_status.data
                    << " R_DBO_WT: " << (int)data->water_tank_top
                    << " R_DBO_WB: " << (int)data->water_tank_bottom
                    << " R_DBO_DS: " << (int)data->dustbin_distance
                    << " R_DBO_DOC: " << (int)data->recv_dustbin_on_the_car
                    << " R_DBO_AP: " << (int)data->auto_push
                    << " R_DBO_FS: " << (int)data->fan_speed << " R_DBO_WC_IS: "
                    << (int)wireless_charge_state_msg.wireless_install_state
                    << " R_DBO_WC_VT: " << data->wireless_voltage
                    << " R_DBO_WC_CU: " << data->wireless_current
                    << " R_DBO_WC_RE: " << (int)data->wireless_reserve
                    << " R_DBO_WC_ST: " << (int)data->wireless_state
                    << " R_DBO_WC_TP: " << (int)data->wireless_temp
                    << " R_DBO_WC_CH: " << (int)data->wireless_changer
                    << " R_DBO_TRA_PER: " << (int)data->trash_per);

  //-------------------通讯检测------------------
  box_chat_state.module_id = 2;
  box_chat_state.recv_state = true;
  box_chat_state.time_recv = rclcpp::Clock().now().seconds();
}

//**************************************
//集尘箱状态发布(新的,加了电池,风机力矩等)
void pub_dust_box_state_new(dust_box_to_motion_t_new *data) {
  //发布旧的消息类型,用于兼容以前的版本
  cti_msgs::msg::DustbinState dust_box_state_msg;
  dust_box_state_msg.header.stamp = rclcpp::Clock().now();
  dust_box_state_msg.header.frame_id = "/chassis_dust_box";

  dust_box_state_msg.water_tank_top =
      data->auto_work;  // 2021-09-09
                        // 这个数据修改为新的含义，但是为了兼容旧的，依旧发出
                        // walter_tank_top
  dust_box_state_msg.water_tank_bottom =
      data->mannual_work;  // 2021-09-09
                           // 这个数据修改为新的含义，但是为了兼容旧的，依旧发出
                           // walter_tank_bottom
  dust_box_state_msg.dustbin_distance = data->dustbin_distance;
  int ultrasonic_distance_length =
      sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
  for (int i = 0; i < ultrasonic_distance_length; i++) {
    dust_box_state_msg.ultrasonic_distance.push_back(
        data->ultrasonic_distance[i]);
  }
  dust_box_state_msg.voltage1 = data->voltage1;
  dust_box_state_msg.voltage2 = data->voltage2;
  dust_box_state_msg.voltage3 = data->voltage3;
  dust_box_state_msg.motor_status = data->motor_status.data;
  dust_box_state_pub->publish(dust_box_state_msg);

  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray dustBoxInfos;
  dustBoxInfos.header.stamp = rclcpp::Clock().now();
  dustBoxInfos.header.frame_id = "sanitation_dust_box";
  cti_msgs::msg::Data dustboxinfo;
  //红外数据
  dustboxinfo.name = "infrared_data";
  dustboxinfo.data = std::to_string(data->infrared_data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  // std::cout<<"======================================="<<std::endl;
  // std::cout<<"======================================="<<std::endl;
  // std::cout<<"infrared_data:"<<data->infrared_data<<std::endl;
  // std::cout<<"infrared_data:"<<(double)data->infrared_data<<std::endl;
  //防夹数据
  dustboxinfo.name = "anti_pinch_data";
  dustboxinfo.data = std::to_string(data->anti_pinch_data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  // std::cout<<"anti_pinch_data:"<<data->anti_pinch_data<<std::endl;
  // std::cout<<"anti_pinch_data:"<<(double)data->anti_pinch_data<<std::endl;
  //拉绳距离
  dustboxinfo.name = "stay_cord_distance";
  dustboxinfo.data = std::to_string(data->stay_cord_distance);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  // std::cout<<"stay_cord_distance:"<<data->stay_cord_distance<<std::endl;
  // std::cout<<"stay_cord_distance:"<<(double)data->stay_cord_distance<<std::endl;
  // std::cout<<"======================================="<<std::endl;
  // std::cout<<"======================================="<<std::endl;
  //垃圾到顶盖的距离
  dustboxinfo.name = "dust_distance";
  dustboxinfo.data = std::to_string(data->dustbin_distance);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池1的电压
  dustboxinfo.name = "bat1_vol";
  dustboxinfo.data = std::to_string(data->voltage1);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池1的电量
  dustboxinfo.name = "bat1_soc";
  dustboxinfo.data = std::to_string(data->Bat_1_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池2的电压
  dustboxinfo.name = "bat2_vol";
  dustboxinfo.data = std::to_string(data->voltage2);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池2的电量
  dustboxinfo.name = "bat2_soc";
  dustboxinfo.data = std::to_string(data->Bat_2_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池3的电压
  dustboxinfo.name = "bat3_vol";
  dustboxinfo.data = std::to_string(data->voltage3);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池3的电量
  dustboxinfo.name = "bat3_soc";
  dustboxinfo.data = std::to_string(data->Bat_3_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池4的电压
  dustboxinfo.name = "bat4_vol";
  dustboxinfo.data = std::to_string(data->voltage4);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_FLOAT;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //电池4的电量
  dustboxinfo.name = "bat4_soc";
  dustboxinfo.data = std::to_string(data->Bat_4_Soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //箱子整体电量
  dustboxinfo.name = "dustbox_soc";
  uint8_t dustbox_soc = 0;
  if (data->voltage4 < 0) {
    //箱子最多装3个电池
    dustbox_soc = (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc) / 3;
  } else {
    //箱子最多装4个电池
    dustbox_soc = (data->Bat_1_Soc + data->Bat_2_Soc + data->Bat_3_Soc +
                   data->Bat_4_Soc) /
                  4;
  }
  if (dustbox_soc > 100) dustbox_soc = 100;
  dustboxinfo.data = std::to_string(dustbox_soc);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //吸尘箱启动状态
  dustboxinfo.name = "motor_status";
  dustboxinfo.data = std::to_string(data->motor_status.data);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //自动倾倒垃圾的状态
  dustboxinfo.name = "dust_autopush";
  dustboxinfo.data = std::to_string(data->auto_push);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机速度,控制反馈
  dustboxinfo.name = "fan_Speed";
  dustboxinfo.data = std::to_string(data->fan_speed);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机转矩
  dustboxinfo.name = "fan_torque";
  dustboxinfo.data = std::to_string(data->fan_torque);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机转速,硬件反馈
  dustboxinfo.name = "fan_speed_hd";
  dustboxinfo.data = std::to_string(data->fan_speed_closedloop);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //风机电流
  dustboxinfo.name = "fan_current";
  dustboxinfo.data = std::to_string(data->fan_current);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //板子错误码
  dustboxinfo.name = "error_code";
  dustboxinfo.data = std::to_string(data->errorInfo);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT64;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //箱子4g状态
  dustboxinfo.name = "4g_state";
  dustboxinfo.data = std::to_string(data->net_state);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //箱子4g信号
  dustboxinfo.name = "4g_signal";
  dustboxinfo.data = std::to_string(data->net_signal);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //垃圾量百分比
  dustboxinfo.name = "trash_percent";
  dustboxinfo.data = std::to_string(data->trash_per);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //自动工作模式 0: 无效,无此功能 1：关 2：开
  dustboxinfo.name = "auto_work";
  dustboxinfo.data = std::to_string(data->auto_work);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);
  //手动工作模式 0: 无效,无此功能 1：关 2：开
  dustboxinfo.name = "mannual_work";
  dustboxinfo.data = std::to_string(data->mannual_work);
  dustboxinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustBoxInfos.datas.push_back(dustboxinfo);

  //消息发布
  dust_box_state_info_pub->publish(dustBoxInfos);

  //电池电量发布
  cti_msgs::msg::BatteryCellsState BatCellsState;
  BatCellsState.header.stamp = rclcpp::Clock().now();
  BatCellsState.soc_all = dustbox_soc;
  BatCellsState.volt_all =
      cmax(cmax(cmax(data->voltage1, data->voltage2), data->voltage3),
           data->voltage4);
  cti_msgs::msg::BatteryCell BatCell;
  if (data->voltage1 > 5) {
    BatCell.bat_soc = data->Bat_1_Soc;
    BatCell.bat_volt = data->voltage1;
    BatCell.bat_cell_num = 1;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->voltage2 > 5) {
    BatCell.bat_soc = data->Bat_2_Soc;
    BatCell.bat_volt = data->voltage2;
    BatCell.bat_cell_num = 2;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->voltage3 > 5) {
    BatCell.bat_soc = data->Bat_3_Soc;
    BatCell.bat_volt = data->voltage3;
    BatCell.bat_cell_num = 3;
    BatCellsState.batcells.push_back(BatCell);
  }
  if (data->voltage4 > 5) {
    BatCell.bat_soc = data->Bat_4_Soc;
    BatCell.bat_volt = data->voltage4;
    BatCell.bat_cell_num = 4;
    BatCellsState.batcells.push_back(BatCell);
  }
  BatCellsState.charge_type = 1;  // 0:undefined 1:wireless_charge
                                  // 2:wired_charge
  BatCellsState.charge_voltage = data->wireless_voltage;
  BatCellsState.charge_current = data->wireless_current;
  BatCellsState.charge_reserve = data->wireless_reserve;
  BatCellsState.charge_state = data->wireless_state;
  BatCellsState.charge_temp = data->wireless_temp;
  //对充电标志位进行窗口滤波
  static std::deque<uint8_t> dustbox_charger_queue_;
  dustbox_charger_queue_.push_back(data->wireless_changer);
  if (dustbox_charger_queue_.size() > 3) dustbox_charger_queue_.pop_front();
  uint8_t wireless_changer = 1;
  if (dustbox_charger_queue_[0] == 0 && dustbox_charger_queue_[1] == 0 &&
      dustbox_charger_queue_[2] == 0)
    wireless_changer = 0;
  BatCellsState.charge_charger = wireless_changer;
  dustbox_batterycell_pub->publish(BatCellsState);
  //箱尾超声波发布  //v6.0 v6.1为四个超声波都在箱子后面 v6.2
  // 1-3号超声波在箱子后面 4号在垃圾仓里

  //所有的原始数据发布
  std_msgs::msg::UInt16MultiArray dustbox_rear_ult;
  int size =
      sizeof(data->ultrasonic_distance) / sizeof(data->ultrasonic_distance[0]);
  for (int i = 0; i < size; i++) {
    dustbox_rear_ult.data.push_back(data->ultrasonic_distance[i]);
  }
  //根据超声波控制命令dustbox_ult_cmd，来进行数据更改
  if ((dustbox_ult_cmd & 0x0000000000004000) == 0) {
    //箱子后右超声波关闭
    dustbox_rear_ult.data[2] = 2500;
  }
  if ((dustbox_ult_cmd & 0x0000000000008000) == 0) {
    //箱子后左超声波关闭
    dustbox_rear_ult.data[3] = 2500;
  }
  if ((dustbox_ult_cmd & 0x0000000000010000) == 0) {
    //箱子底部超声波关闭
    dustbox_rear_ult.data[4] = 2500;
  }

  dustbox_rear_range_pub->publish(dustbox_rear_ult);

  if (!LIN_ult_installed) {
    if (cti_run_ver != "v6.0" && cti_run_ver != "v6.1") {
      sensor_msgs::msg::Range dustboxrRangeData;
      const float dustbox_min_range = 0.12;
      const float dustbox_max_range = 2.5;
      dustboxrRangeData.header.stamp = rclcpp::Clock().now();
      dustboxrRangeData.radiation_type = 0;  // ult
      dustboxrRangeData.min_range = dustbox_min_range;
      dustboxrRangeData.max_range = msg_max_range;
      dustboxrRangeData.field_of_view = 24 * M_PI / 180.0f;
      for (int i = 0; i < dustbox_max_type_ult; i++) {
        float ult_data;
        if (0 == data->auto_push) {
          ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
        } else {
          ult_data =
              8.88;  // 8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
        }
        dustboxrRangeData.range = ult_data;
        dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
        dustbox_range_pub[i]->publish(dustboxrRangeData);
      }
      float ult_data;
      ult_data = (float)data->ultrasonic_distance[5] / 1000.f;
      dustboxrRangeData.range = ult_data;
      dustboxrRangeData.header.frame_id = "bottom";
      dustbox_bottom_range_pub->publish(dustboxrRangeData);
    } else {
      sensor_msgs::msg::Range dustboxrRangeData;
      const float dustbox_min_range = 0.12;
      const float dustbox_max_range = 2.5;
      dustboxrRangeData.header.stamp = rclcpp::Clock().now();
      dustboxrRangeData.radiation_type = 0;
      dustboxrRangeData.min_range = dustbox_min_range;
      dustboxrRangeData.max_range = msg_max_range;
      dustboxrRangeData.field_of_view = 24 * M_PI / 180.0f;
      for (int i = 0; i < dustbox_max_type_ult; i++) {
        float ult_data;
        if (0 == data->auto_push) {
          ult_data = (float)data->ultrasonic_distance[i + 2] / 1000.f;
        } else {
          ult_data =
              8.88;  // 8.88表示此时在倾倒垃圾,超声波数据给一个超出正常范围的值
        }
        dustboxrRangeData.range = ult_data;
        dustboxrRangeData.header.frame_id = dustbox_ult_name[i];
        dustbox_range_pub[i]->publish(dustboxrRangeData);
      }
    }
  }
  //自动倒垃圾状态
  std_msgs::msg::UInt8 dust_box_autopush_msg;
  dust_box_autopush_msg.data = data->auto_push;
  dust_box_autopush_pub->publish(dust_box_autopush_msg);

  //风机速度
  std_msgs::msg::UInt8 dust_box_fanspeed_msg;
  dust_box_fanspeed_msg.data = data->fan_speed;
  dust_box_fanspeed_pub->publish(dust_box_fanspeed_msg);

  //无线充电
  cti_msgs::msg::BatteryState wireless_charge_state_msg;
  wireless_charge_state_msg.header.stamp = rclcpp::Clock().now();
  wireless_charge_state_msg.wireless_install_state = 1;
  wireless_charge_state_msg.wireless_voltage = data->wireless_voltage;
  wireless_charge_state_msg.wireless_current = data->wireless_current;
  wireless_charge_state_msg.wireless_reserve = data->wireless_reserve;
  wireless_charge_state_msg.wireless_state = data->wireless_state;
  wireless_charge_state_msg.wireless_temp = data->wireless_temp;
  wireless_charge_state_msg.wireless_changer = data->wireless_changer;
  dustbox_wireless_charge_state_pub->publish(wireless_charge_state_msg);

  Info("R_DBO_ES: " << (int)data->motor_status.data
                    << " R_DBO_AW: " << (int)data->auto_work
                    << " R_DBO_MW: " << (int)data->mannual_work
                    << " R_DBO_DS: " << (int)data->dustbin_distance
                    << " R_DBO_DOC: " << (int)data->recv_dustbin_on_the_car
                    << " R_DBO_AP: " << (int)data->auto_push
                    << " R_DBO_FS: " << (int)data->fan_speed << " R_DBO_WC_IS: "
                    << (int)wireless_charge_state_msg.wireless_install_state
                    << " R_DBO_WC_VT: " << data->wireless_voltage
                    << " R_DBO_WC_CU: " << data->wireless_current
                    << " R_DBO_WC_RE: " << (int)data->wireless_reserve
                    << " R_DBO_WC_ST: " << (int)data->wireless_state
                    << " R_DBO_WC_TP: " << (int)data->wireless_temp
                    << " R_DBO_WC_CH: " << (int)data->wireless_changer
                    << " R_DBO_WC_CH_RE: " << (int)wireless_changer
                    << " R_DBO_BAT1: " << data->voltage1 << " R_DBO_SOC1: "
                    << (int)data->Bat_1_Soc << " R_DBO_BAT2: " << data->voltage2
                    << " R_DBO_SOC2: " << (int)data->Bat_2_Soc
                    << " R_DBO_BAT3: " << data->voltage3 << " R_DBO_SOC3: "
                    << (int)data->Bat_3_Soc << " R_DBO_BAT4: " << data->voltage4
                    << " R_DBO_SOC4: " << (int)data->Bat_4_Soc
                    << " R_DBO_FTQ: " << (int)data->fan_torque
                    << " R_DBO_FSCL: " << (int)data->fan_speed_closedloop
                    << " R_DBO_FCU: " << (int)data->fan_current
                    << " R_DBO_ERR: " << (int)data->errorInfo
                    << " R_DBO_TRA_PER: " << (int)data->trash_per
                    << " R_DBO_4G_ST: " << (int)data->net_state
                    << " R_DBO_4G_SI: " << (int)data->net_signal
                    << " R_DBO_PW_SOC: " << (int)dustbox_soc);
  //-------------------通讯检测------------------
  box_chat_state.module_id = 2;
  box_chat_state.recv_state = true;
  box_chat_state.time_recv = rclcpp::Clock().now().seconds();
}

//清扫箱id返回处理
void process_dustbin_id(dustbin_rf_set_cmd_t *data) {
  if (read_dustbin_id) {
    read_dustbin_id = false;
  } else {
    dustbin_set_state.if_recv = true;
    dustbin_set_state.recv_id = data->id;
  }
  pre_recv_dustbin_id = data->id;
  dustbinidstate.rw = data->rw;
  dustbinidstate.reset = data->reset;
  dustbinidstate.mode = data->mode;
  dustbinidstate.link = data->link;
  dustbinidstate.baud = data->baud;
  dustbinidstate.id = data->id;
  recv_dustbin_id_state_pub->publish(dustbinidstate);
  Info("R_DBI_ID: " << (int)data->id << " R_DBI_LN: " << (int)data->link);
}

//************************************** 雨水传感器数据发布
void pub_rain_sensor(rain_sensor_t *data) {
  cti_msgs::msg::BoxState rain_sensor_msgs;
  rain_sensor_msgs.header.stamp = rclcpp::Clock().now();
  cti_msgs::msg::TabState rain_sensor_msg;
  rain_sensor_msg.status = data->right;
  rain_sensor_msg.name = "right";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->left;
  rain_sensor_msg.name = "left";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_pub->publish(rain_sensor_msgs);
  Info("R_RS_R: " << (int)data->right << "R_RS_L: " << (int)data->left);
}

//**************************************
//雨水传感器数据发布--新的结构体<嵌入式有更新的有没有更新的，所以新结构体和旧结构体都要保留>
void pub_rain_sensor_new(rain_sensor_t_new *data) {
  cti_msgs::msg::BoxState rain_sensor_msgs;
  rain_sensor_msgs.header.stamp = rclcpp::Clock().now();
  cti_msgs::msg::TabState rain_sensor_msg;
  rain_sensor_msg.status = data->flag.bits.switch_door;
  rain_sensor_msg.name = "switch_door";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->flag.bits.tool_door;
  rain_sensor_msg.name = "tool_door";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->right;
  rain_sensor_msg.name = "right";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_msg.status = data->left;
  rain_sensor_msg.name = "left";
  rain_sensor_msgs.states.push_back(rain_sensor_msg);
  rain_sensor_pub->publish(rain_sensor_msgs);
  Info("R_RS_R: " << (int)data->right << "R_RS_L: " << (int)data->left
                  << "R_RS_F: " << (int)data->flag.data);
}

//*********************************** 智能垃圾箱状态发布
void pub_smart_trash(smart_trash_report *data) {
  //使用cti_msgs/DataArray消息类型发布消息
  cti_msgs::msg::DataArray smartTrashInfos;
  smartTrashInfos.header.stamp = rclcpp::Clock().now();
  smartTrashInfos.header.frame_id = "smart_trash";

  cti_msgs::msg::Data smarttrashinfo;
  //当前位置 uint8_t 1:底部：2:顶部
  smarttrashinfo.name = "current_position";
  smarttrashinfo.data = std::to_string(data->current_position);
  smarttrashinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  smartTrashInfos.datas.push_back(smarttrashinfo);
  //运动状态 uint8_t 0：静止 1:向上，2向下
  smarttrashinfo.name = "kinestate";
  smarttrashinfo.data = std::to_string(data->kinestate);
  smarttrashinfo.type = cti_msgs::msg::Data::TYPE_UINT8;
  smartTrashInfos.datas.push_back(smarttrashinfo);
  //具体位置 uint16_t
  smarttrashinfo.name = "location";
  smarttrashinfo.data = std::to_string(data->location);
  smarttrashinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  smartTrashInfos.datas.push_back(smarttrashinfo);
  //升降报错 uint16_t
  smarttrashinfo.name = "errorinfo";
  smarttrashinfo.data = std::to_string(data->errorinfo);
  smarttrashinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  smartTrashInfos.datas.push_back(smarttrashinfo);
  //当前连接lora的id号
  smarttrashinfo.name = "lora_id";
  smarttrashinfo.data = std::to_string(pre_recv_dustbin_id);
  smarttrashinfo.type = cti_msgs::msg::Data::TYPE_INT64;
  smartTrashInfos.datas.push_back(smarttrashinfo);
  //感应开关是否开关 1:关闭感应开盖功能 0: 打开感应开盖功能
  smarttrashinfo.name = "reaction_state";
  smarttrashinfo.data = std::to_string(data->reaction_state);
  smarttrashinfo.type = cti_msgs::msg::Data::TYPE_UINT16;
  smartTrashInfos.datas.push_back(smarttrashinfo);

  smart_trash_state_pub->publish(smartTrashInfos);

  Info("R_ST_PO: " << (int)data->current_position << "R_ST_ST: "
                   << (int)data->kinestate << "R_ST_LC: " << (int)data->location
                   << "R_ST_ERR: " << (int)data->errorinfo);
  //-------------------通讯检测------------------
  box_chat_state.module_id = 3;
  box_chat_state.recv_state = true;
  box_chat_state.time_recv = rclcpp::Clock().now().seconds();
}

//*********************************** 吸尘箱5g状态发布
void pub_dustbox_5g_state(dustbox_5g_recv_cmd_t *data) {
  cti_msgs::msg::DataArray dustbox5GStates;
  dustbox5GStates.header.stamp = rclcpp::Clock().now();
  dustbox5GStates.header.frame_id = "smart_trash";

  cti_msgs::msg::Data dustbox5GState;
  dustbox5GState.name = "port_index";
  dustbox5GState.data = std::to_string(data->portIndex);
  dustbox5GState.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustbox5GStates.datas.push_back(dustbox5GState);

  dustbox5GState.name = "msg1";
  int array_size = sizeof(data->msg1) / sizeof(data->msg1[0]);
  dustbox5GState.data.clear();
  for (int i = 0; i < array_size; i++) {
    dustbox5GState.data.push_back((char)data->msg1[i]);
  }
  std::string msg1 = dustbox5GState.data;
  dustbox5GState.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustbox5GStates.datas.push_back(dustbox5GState);

  dustbox5GState.name = "unused1";
  array_size = sizeof(data->unused1) / sizeof(data->unused1[0]);
  dustbox5GState.data.clear();
  for (int i = 0; i < array_size; i++) {
    dustbox5GState.data.push_back((char)data->unused1[i]);
  }
  std::string unused1 = dustbox5GState.data;
  dustbox5GState.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustbox5GStates.datas.push_back(dustbox5GState);

  dustbox5GState.name = "unused2";
  array_size = sizeof(data->unused2) / sizeof(data->unused2[0]);
  dustbox5GState.data.clear();
  for (int i = 0; i < array_size; i++) {
    dustbox5GState.data.push_back((char)data->unused2[i]);
  }
  std::string unused2 = dustbox5GState.data;
  dustbox5GState.type = cti_msgs::msg::Data::TYPE_UINT8;
  dustbox5GStates.datas.push_back(dustbox5GState);
  dustbox_5g_state_pub->publish(dustbox5GStates);
  Info("R_DOB_5G_PI: " << (int)data->portIndex << "R_DOB_5G_MSG1: "
                       << msg1.c_str() << "R_DOB_5G_UN1: " << unused1.c_str()
                       << "R_DOB_5G_UN2: " << unused2.c_str());
}
//--超声波模式接收处理
void process_ult_mode_report(report_lin_ult_mode_type *data) {
  //车身超声波处理
  // printf("*****************recv_ult_mode_report******************\n");
  // printf("data->src: %d\n",data->src);
  // printf("data->dest: %d\n",data->dest);
  // printf("data->mode: 0x%016lx\n",data->mode.data);
  // printf("*****************recv_ult_mode_report******************\n");

  if (data->src == MODULE_ULTRASONIC_MASTER) {
    vehicle_ult_set_state.recv_mode = data->mode.data;
    if (vehicle_ult_set_state.state == 2) {
      printf("vehicle_ult_set_state.set_mode: 0x%016lx\n",
             vehicle_ult_set_state.set_mode);
      printf("vehicle_ult_set_state.recv_mode: 0x%016lx\n",
             vehicle_ult_set_state.recv_mode);

      if (vehicle_ult_set_state.set_mode == vehicle_ult_set_state.recv_mode) {
        vehicle_ult_set_state.state = 0;
        vehicle_ult_check_resend_time = 0;
      } else {
        vehicle_ult_set_state.state = 3;
        vehicle_ult_check_resend_time = 0;
      }
    } else if (vehicle_ult_set_state.state == 4) {
      if (vehicle_ult_set_state.set_mode == vehicle_ult_set_state.recv_mode) {
        vehicle_ult_set_state.state = 0;
        vehicle_ult_set_resend_time = 0;
      } else {
        vehicle_ult_set_state.state = -3;
        vehicle_ult_set_resend_time = 0;
      }
    } else {
      vehicle_ult_set_state.state = 0;
    }
    Info("R_V_ULT_MD: " << data->mode.data);
  }

  //吸尘箱超声波处理
  if (data->src == MODULE_DUST_BOX_BOARD) {
    dustbox_ult_set_state.recv_mode = data->mode.data;
    if (dustbox_ult_set_state.state == 2) {
      if (dustbox_ult_set_state.set_mode == dustbox_ult_set_state.recv_mode) {
        dustbox_ult_set_state.state = 0;
        dustbox_ult_check_resend_time = 0;
      } else {
        dustbox_ult_set_state.state = 3;
        dustbox_ult_check_resend_time = 0;
      }
    } else if (dustbox_ult_set_state.state == 4) {
      if (dustbox_ult_set_state.set_mode == dustbox_ult_set_state.recv_mode) {
        dustbox_ult_set_state.state = 0;
        dustbox_ult_set_resend_time = 0;
      } else {
        dustbox_ult_set_state.state = -3;
        dustbox_ult_set_resend_time = 0;
      }
    } else {
      dustbox_ult_set_state.state = 0;
    }
    Info("R_D_ULT_MD: " << data->mode.data);
  }

#if DEBUG_PRINT
  RCLCPP_INFO(this->get_logger(), "recv ult mode repor: %08x", data->mode.data);
#endif
}
//************************************** 非升级命令接收处理
int process_nomal_cmd_ex(unsigned char cmd, unsigned char *data,
                         unsigned int length) {
  // printf("nomal cmd=%d\n",cmd);
  switch (cmd) {
    //////////////////////////////////////////////////////////////////
    case ULT_POS_SEND_CMD:
      msg_upa_pos_data_t *msg_upa_pos_data;
      if (sizeof(msg_upa_pos_data_t) != length) {
        std::cout << "recv msg_upa_pos_data_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(msg_upa_pos_data_t) << std::endl;
        return -1;
      }
      msg_upa_pos_data = (msg_upa_pos_data_t *)data;
      pub_alt_3_0_(msg_upa_pos_data);
      // pub_odom(msg_upa_pos_data);
      break;
    case RECV_FROM_CONTROL_STATUS:
      recv_from_control_status_type *recv_statusdata;
      if (sizeof(recv_from_control_status_type) != length) {
        std::cout << "recv control length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_control_status_type) << std::endl;
        // printf("recv control length:%d error! sizeof struct
        // length:%d",length,(int)sizeof(recv_from_control_status_type));
        return -1;
      }
      recv_ctrl_cnt++;
      recv_ctrl_cnt %= 65535;
      recv_ctrl_rate_cnt++;
      recv_ctrl_rate_cnt %= 65535;
      recv_statusdata = (recv_from_control_status_type *)data;
      pub_odom(recv_statusdata);
      break;
    case RECV_FROM_CONTROL_BATTERY_4_TO_1_STATUS:
      //////////////////////////////////////////////////////////////////
      recv_battery_4_to_1_active_report_status_type *recv_batterydata;
      recv_battery_4_to_1_active_report_status_with_wireless_type
          *recv_batterydata_with_wireless;
      recv_battery_4_to_1_active_report_status_type_3_0_
          *recv_battery_4_to_1_active_report_status_type_3_0;

      if (sizeof(recv_battery_4_to_1_active_report_status_type) != length &&
          sizeof(recv_battery_4_to_1_active_report_status_with_wireless_type) !=
              length &&
          sizeof(recv_battery_4_to_1_active_report_status_type_3_0_) !=
              length) {
        std::cout
            << "recv battery length: " << length
            << " error! sizeof struct length: "
            << sizeof(recv_battery_4_to_1_active_report_status_type)
            << " sizeof struct_with_wireless: "
            << sizeof(
                   recv_battery_4_to_1_active_report_status_with_wireless_type)
            << " sizeof struct_3.0: "
            << sizeof(recv_battery_4_to_1_active_report_status_type_3_0_)
            << std::endl;
        // printf("recv battery length:%d error! sizeof struct length:%d, sizeof
        // struct_with_wireless:%d\n",length,(int)sizeof(recv_battery_4_to_1_active_report_status_type),(int)sizeof(recv_battery_4_to_1_active_report_status_with_wireless_type));
        return -1;
      }
      if (length == sizeof(recv_battery_4_to_1_active_report_status_type)) {
        recv_batterydata =
            (recv_battery_4_to_1_active_report_status_type *)data;
        pub_battery(recv_batterydata);
      }
      if (length ==
          sizeof(recv_battery_4_to_1_active_report_status_with_wireless_type)) {
        recv_batterydata_with_wireless =
            (recv_battery_4_to_1_active_report_status_with_wireless_type *)data;
        pub_battery_with_wireless(recv_batterydata_with_wireless);
      }
      if (length ==
          sizeof(recv_battery_4_to_1_active_report_status_type_3_0_)) {
        recv_battery_4_to_1_active_report_status_type_3_0 =
            (recv_battery_4_to_1_active_report_status_type_3_0_ *)data;
        pub_battery_3_0(recv_battery_4_to_1_active_report_status_type_3_0);
      }
      break;
    case RECV_FROM_DRIVER_STATUS:
      recv_from_driver_status_type *recv_driverdata;
      if (sizeof(recv_from_driver_status_type) != length) {
        std::cout << "recv driver status length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_driver_status_type) << std::endl;
        // printf("recv driver status length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_from_driver_status_type));
        return -1;
      }
      recv_driverdata = (recv_from_driver_status_type *)data;
      pub_driverstatus(recv_driverdata);
      break;
    case RECV_FROM_ULTRASONIC_DATA:
      recv_ult_cnt++;
      recv_ult_cnt %= 65535;
      if (0 == LIN_ult_installed) {
        recv_from_ultrasonic_data *recv_ultdata;
        if (sizeof(recv_from_ultrasonic_data) != length) {
          std::cout << "recv ult data length: " << length
                    << " error! sizeof struct length: "
                    << sizeof(recv_from_ultrasonic_data) << std::endl;
          // printf("recv ult data length:%d error! sizeof struct
          // length:%d\n",length,(int)sizeof(recv_from_ultrasonic_data));
          return -1;
        }
        recv_ultdata = (recv_from_ultrasonic_data *)data;
        pub_ultrasonicdata(recv_ultdata);
      } else if (1 == LIN_ult_installed) {
        recv_from_ultrasonic_data_lin *recv_ultdata;
        if (sizeof(recv_from_ultrasonic_data_lin) != length) {
          std::cout << "recv ult data length: " << length
                    << " error! sizeof struct length: "
                    << sizeof(recv_from_ultrasonic_data_lin) << std::endl;
          // printf("recv ult data length:%d error! sizeof struct
          // length:%d\n",length,(int)sizeof(recv_from_ultrasonic_data_lin));
          return -1;
        }
        recv_ultdata = (recv_from_ultrasonic_data_lin *)data;
        pub_ultrasonicdata_lin(recv_ultdata);
      } else {
        std::cout << "error: LIN_ult_installed: " << LIN_ult_installed
                  << " is wrong value, the right value is 0 or 1!" << std::endl;
        // printf("error:  LIN_ult_installed: %d  is wrong value, the right
        // value is 0 or 1! ",LIN_ult_installed);
        return -1;
      }
      break;
    case UPD_CMD_210:
    case RECV_FROM_FIRMWARE_VERSION:
      Info("R_CMD_210: " << 1);
      recv_from_firmware_version_type *recv_version;
      if (sizeof(recv_from_firmware_version_type) != length) {
        std::cout << "recv version length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_firmware_version_type) << std::endl;
        // printf("recv version length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_from_firmware_version_type));
        return -1;
      }
      recv_version = (recv_from_firmware_version_type *)data;
      pub_firmwareversion(recv_version);
      break;
    case RECV_FROM_CMD_ANSWER:
      recv_from_cmd_answer_type *recv_cmdanswer;
      if (sizeof(recv_from_cmd_answer_type) != length) {
        std::cout << "recv command answer length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_cmd_answer_type) << std::endl;
        // printf("recv command answer length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_from_cmd_answer_type));
        return -1;
      }
      cmd_withid_recv_cnt++;
      recv_cmd_cnt++;
      recv_cmd_cnt %= 65535;
      recv_cmdanswer = (recv_from_cmd_answer_type *)data;
      compar_id_recv_send(recv_cmdanswer);
      break;
    case RECV_FROM_RFID_INFO:
      recv_from_rfid_info_type *recv_rfidinfo;
      if (sizeof(recv_from_rfid_info_type) != length) {
        std::cout << "recv rfid info length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_from_rfid_info_type) << std::endl;
        // printf("recv from rfid info length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_from_rfid_info_type));
        return -1;
      }
      recv_rfid_cnt++;
      recv_rfid_cnt %= 65535;
      recv_rfidinfo = (recv_from_rfid_info_type *)data;
      pub_rfidinfo(recv_rfidinfo);
      break;
    case RECV_CHASSIS_ERROE_REPORT:
      recv_chassis_error_report_type *recv_chassiserror;
      if (sizeof(recv_chassis_error_report_type) != length) {
        std::cout << "recv chassis error length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_chassis_error_report_type) << std::endl;
        // printf("recv from chassis error length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_chassis_error_report_type));
        return -1;
      }
      recv_chassiserror = (recv_chassis_error_report_type *)data;
      pub_chassiserror(recv_chassiserror);
      break;
    case RECV_NAVIGATION_LOG_STATUS:
      recv_navigation_log_status_type *recv_navigationlog;
      if (sizeof(recv_navigation_log_status_type) != length) {
        std::cout << "recv navigation log length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_navigation_log_status_type) << std::endl;
        // printf("recv from navigation log  length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_navigation_log_status_type));
        return -1;
      }
      recv_navigationlog = (recv_navigation_log_status_type *)data;
      pub_navigationlog(recv_navigationlog);
      break;
    // SDK格式化是否成功的回复。0：成功
    case SEND_FORMAT_SD_CARD_CMD:
      send_format_sd_card_cmd_type *recv_formatsdcard;
      if (sizeof(send_format_sd_card_cmd_type) != length) {
        std::cout << "recv format sd card length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(send_format_sd_card_cmd_type) << std::endl;
        // printf("recv from navigation log length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(send_format_sd_card_cmd_type));
        return -1;
      }
      recv_formatsdcard = (send_format_sd_card_cmd_type *)data;
      pub_formatsdcard(recv_formatsdcard);
      break;
    case RECV_FROM_GPS:
      recv_gps_data_type *recv_gps;
      if (sizeof(recv_gps_data_type) != length) {
        std::cout << "recv gps_data length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(recv_gps_data_type) << std::endl;
        // printf("recv from gps_data length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(recv_gps_data_type));
        return -1;
      }
      recv_gps = (recv_gps_data_type *)data;
      pub_gps(recv_gps);
      break;
    case UPD_CMD_206:
      get_version_flag = false;
      need_send_rough = true;  //运运控重启之后，重新进行时间粗同步
      Info("R_CMD_206: " << 1);
      break;
    case TIME_SYNC_ROUGH_CMD:
      time_sync_rough_msg_type *recv_rough_res;
      if (sizeof(time_sync_rough_msg_type) != length) {
        std::cout << "recv rough_time_response length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(time_sync_rough_msg_type) << std::endl;
        // printf("recv from rough_time_response length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(time_sync_rough_msg_type));
        return -1;
      }
      recv_rough_res = (time_sync_rough_msg_type *)data;
      check_rough_res(recv_rough_res);
      break;
    case TIME_SYNC_FINE_CMD:
      // Info(" time_sync_successd: "<<1);
      if (sizeof(time_sync_fine_msg_type) != length) {
        std::cout << "recv fine_time_response length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(time_sync_fine_msg_type) << std::endl;
        // printf("recv from fine_time_response length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(time_sync_fine_msg_type));
        return -1;
      }
      break;
    case TIME_SYNC_DELAY_CMD:
      if (sizeof(time_sync_fine_msg_type) != length) {
        std::cout << "recv delay_time_response length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(time_sync_fine_msg_type) << std::endl;
        // printf("recv from delay_time_response length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(time_sync_fine_msg_type));
        return -1;
      }
      send_delay_sync_cmd();
      break;
    case RECV_FROM_DUSTBIN_INFO:
      if (robot_type == 0) {
        dustbin_to_motion_t *recv_dustbin_state;
        if (sizeof(dustbin_to_motion_t) != length) {
          std::cout << "recv dustbin_to_motion_t length: " << length
                    << " error! sizeof struct length: "
                    << sizeof(dustbin_to_motion_t) << std::endl;
          // printf("recv from dustbin_to_motion_t length:%d error! sizeof
          // struct length:%d\n",length,(int)sizeof(dustbin_to_motion_t));
          return -1;
        }
        recv_dustbin_state = (dustbin_to_motion_t *)data;
        pub_dustbin_state(recv_dustbin_state);
      } else if (robot_type == 1) {
        clean_to_motion_t *recv_clean_state;
        clean_to_motion_t_new *recv_clean_state_new;
        serial_status.dustbin_state_recv_size = length;
        if (sizeof(clean_to_motion_t) == length) {
          serial_status.recv_from_dustbin_state_old_cnt++;
          serial_status.recv_from_dustbin_state_old_cnt %= 32000;
          recv_clean_state = (clean_to_motion_t *)data;
          pub_clean_state(recv_clean_state);
        } else if (sizeof(clean_to_motion_t_new) == length) {
          serial_status.recv_from_dustbin_state_new_cnt++;
          serial_status.recv_from_dustbin_state_new_cnt %= 32000;
          recv_clean_state_new = (clean_to_motion_t_new *)data;
          pub_clean_state_new(recv_clean_state_new);
        } else {
          std::cout << "recv clean_to_motion_t length: " << length
                    << " error! sizeof struct length: "
                    << sizeof(clean_to_motion_t)
                    << "or new: " << sizeof(clean_to_motion_t_new) << std::endl;
          // printf("recv from clean_to_motion_t length:%d error! sizeof struct
          // length:old: %d or new:
          // %d\n",length,(int)sizeof(clean_to_motion_t),(int)sizeof(clean_to_motion_t_new));
          return -1;
        }
      } else {
        dustbin_to_motion_t *recv_dustbin_state;
        if (sizeof(dustbin_to_motion_t) != length) {
          std::cout << "recv dustbin_to_motion_t length: " << length
                    << " error! sizeof struct length: "
                    << sizeof(dustbin_to_motion_t) << std::endl;
          // printf("recv from dustbin_to_motion_t length:%d error! sizeof
          // struct length:%d\n",length,(int)sizeof(dustbin_to_motion_t));
          return -1;
        }
        recv_dustbin_state = (dustbin_to_motion_t *)data;
        pub_dustbin_state(recv_dustbin_state);
      }
      break;
    case DUSBIN_RF_SET_READ_CMD:
      dustbin_rf_set_cmd_t *recv_dustbin_id;
      if (sizeof(dustbin_rf_set_cmd_t) != length) {
        std::cout << "recv dustbin_rf_set_cmd_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(dustbin_rf_set_cmd_t) << std::endl;
        // printf("recv from dustbin_rf_set_cmd_t length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(dustbin_rf_set_cmd_t));
        return -1;
      }
      recv_dustbin_id = (dustbin_rf_set_cmd_t *)data;
      process_dustbin_id(recv_dustbin_id);
      break;
    case RECV_FROM_DUST_BOX_INFO:
      dust_box_to_motion_t_old *recv_dust_box_state_old;
      dust_box_to_motion_t_new *recv_dust_box_state_new;
      serial_status.dustbox_state_recv_size = length;
      if (sizeof(dust_box_to_motion_t_old) == length) {
        serial_status.recv_from_dustbox_state_old_cnt++;
        serial_status.recv_from_dustbox_state_old_cnt %= 32000;
        recv_dust_box_state_old = (dust_box_to_motion_t_old *)data;
        pub_dust_box_state_old(recv_dust_box_state_old);
      } else if (sizeof(dust_box_to_motion_t_new) == length) {
        serial_status.recv_from_dustbox_state_new_cnt++;
        serial_status.recv_from_dustbox_state_new_cnt %= 32000;
        recv_dust_box_state_new = (dust_box_to_motion_t_new *)data;
        pub_dust_box_state_new(recv_dust_box_state_new);
      } else {
        recv_dust_box_state_new = (dust_box_to_motion_t_new *)data;
        pub_dust_box_state_new(recv_dust_box_state_new);
        std::cout << "recv dust_box_to_motion_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(dustbin_rf_set_cmd_t)
                  << " or new: " << sizeof(dust_box_to_motion_t_new)
                  << std::endl;
        // printf("recv from dust_box_to_motion_t length:%d error! sizeof struct
        // length: old: %d or new:
        // %d\n",length,(int)sizeof(dust_box_to_motion_t_old),(int)sizeof(dust_box_to_motion_t_new));
        return -1;
      }
      break;
    case RAIN_SENSOR:
      rain_sensor_t *recv_rain_sensor;
      rain_sensor_t_new *recv_rain_sensor_new;
      if (sizeof(rain_sensor_t) != length &&
          sizeof(rain_sensor_t_new) != length) {
        std::cout << "recv rain_sensor_t length: " << length
                  << " error! sizeof struct length old: "
                  << sizeof(rain_sensor_t)
                  << "size of struct length new: " << sizeof(rain_sensor_t_new)
                  << std::endl;
        // printf("recv from rain_sensor_t length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(rain_sensor_t));
        return -1;
      }
      if (sizeof(rain_sensor_t) == length) {
        recv_rain_sensor = (rain_sensor_t *)data;
        pub_rain_sensor(recv_rain_sensor);
      }
      if (sizeof(rain_sensor_t_new) == length) {
        recv_rain_sensor_new = (rain_sensor_t_new *)data;
        pub_rain_sensor_new(recv_rain_sensor_new);
      }
      break;
    case SMART_TRASH_RECV:
      smart_trash_report *recv_smart_trash;
      if (sizeof(smart_trash_report) != length) {
        std::cout << "recv smart_trash_report length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(smart_trash_report) << std::endl;
        // printf("recv from smart_trash_report length:%d error! sizeof struct
        // length:%d\n",length,(int)sizeof(smart_trash_report));
        return -1;
      }
      recv_smart_trash = (smart_trash_report *)data;
      pub_smart_trash(recv_smart_trash);
      break;
    case DUST_BOX_5G_RECV_CMD:
      dustbox_5g_recv_cmd_t *dustbox_5g_recv;
      if (sizeof(dustbox_5g_recv_cmd_t) != length) {
        std::cout << "recv dustbox_5g_recv_cmd_t length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(dustbox_5g_recv_cmd_t) << std::endl;
        // printf("recv from dustbox_5g_recv_cmd_t length:%d error! sizeof
        // struct length:%d\n",length,(int)sizeof(dustbox_5g_recv_cmd_t));
        return -1;
      }
      dustbox_5g_recv = (dustbox_5g_recv_cmd_t *)data;
      pub_dustbox_5g_state(dustbox_5g_recv);
      break;
    case RECV_LIN_ULT_MODE:
      report_lin_ult_mode_type *recv_ult_mode;
      if (sizeof(report_lin_ult_mode_type) != length) {
        std::cout << "recv report_lin_ult_mode_type length: " << length
                  << " error! sizeof struct length: "
                  << sizeof(report_lin_ult_mode_type) << std::endl;
        // printf("recv from report_lin_ult_mode_type length:%d error! sizeof
        // struct length:%d\n",length,(int)sizeof(report_lin_ult_mode_type));
        return -1;
      }
      recv_ult_mode = (report_lin_ult_mode_type *)data;
      process_ult_mode_report(recv_ult_mode);
      break;
    default:
      break;
  }
  return 0;
}

//************************************** 升级命令接收处理
int process_update_cmd_ex(unsigned char cmd, unsigned char *data,
                          unsigned int length) {
  if ((NULL == data) || (length > 510)) {
    return -1;
  }
  cti_fpga_serial_msgs::msg::UpdateInfo upd_data;
  seq_num++;
  upd_data.seq_num = seq_num;
  for (int i = 0; i < length + 2; i++) {
    upd_data.data.push_back(*(data + i));
  }
  stm32_pub->publish(upd_data);
}

//************************************** 定时器 代码更新超时处理
void timerCallback() {
  //--代码更新超时处理--
  if (stm32_update_flag && (++TIMEOUT) >= 20) {
    TIMEOUT = 0;
    stm32_update_flag = false;
  }
}

//************************************** 定时器2 查询版本号和发送时间同步
void timer2Callback() {
  //--查询版本号--
  if (get_version_flag == false) {
    send_to_check_version_type check_version_cmd;
    update_info_type update_info_struct;
    update_info_struct.src = MODULE_CHECK_UPD;
    update_info_struct.dest = MODULE_MOVE_CONTROL_BOARD;
    check_version_cmd.upd_info = update_info_struct;
    check_version_cmd.check = 01;
    construct_serial_frame_ex(&user_frame, SEND_TO_CHECK_PROGRAM_VERSION,
                              sizeof(check_version_cmd), &check_version_cmd);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = SEND_TO_CHECK_PROGRAM_VERSION;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
  }

  if ((sendtime_loop_num >= 60 || need_resend_rough) && need_send_rough) {
    //获取年月日时分秒
    struct tm *local;
    time_t t;
    t = time(NULL);
    local = localtime(&t);
    //获取到微秒级别
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);

    time_sync_rough_msg_type timenow_cmd;

    timenow_cmd.year = local->tm_year;
    timenow_cmd.mon = local->tm_mon;
    timenow_cmd.day = local->tm_mday;
    timenow_cmd.hour = local->tm_hour;
    timenow_cmd.min = local->tm_min;
    timenow_cmd.sec = local->tm_sec;
    construct_serial_frame_ex(&user_frame, TIME_SYNC_ROUGH_CMD,
                              sizeof(timenow_cmd), &timenow_cmd);
    serial_frame_include_id_type user_frame_include_id_0;
    user_frame_include_id_0.id = 0;
    user_frame_include_id_0.cmd = TIME_SYNC_ROUGH_CMD;
    user_frame_include_id_0.need_id = 0;
    user_frame_include_id_0.frame = user_frame;
    pushData(&user_frame_include_id_0);
    sendtime_loop_num = 0;
    need_resend_rough = false;
    sync_rough_msg_saved = timenow_cmd;
    check_rough_sync_res_timeoout = true;
    check_rough_sync_res_cnt = 0;
  }
  sendtime_loop_num++;

  if (check_rough_sync_res_timeoout) {
    if (check_rough_sync_res_cnt >= 3) {
      ////Info(" recv_rough_sync_res_timeout: "<<1);
      need_resend_rough = true;
      check_rough_sync_res_timeoout = false;
      check_rough_sync_res_cnt = 0;
    }
    check_rough_sync_res_cnt++;
  }

  if (need_send_fine_sync) {
    if (send_fine_sync_cnt >= 60) {
      send_fine_sync_cnt = 0;
      send_fine_sync_cmd();
    }
    send_fine_sync_cnt++;
  }

  cmd_answer_cnt.cmd_answer_success_num = cmd_answer_success_cnt;
  cmd_answer_cnt.cmd_answer_fail_num =
      cmd_withid_send_cnt - cmd_answer_success_cnt;
  cmd_answer_cnt.cmd_withid_send_num = cmd_withid_send_cnt;
  cmd_answer_pub->publish(cmd_answer_cnt);
}

//************************************** 清扫箱命令重发函数
void resend_dustbin_ctrl_cmd() {
  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(resend_to_dustbin_cmd),
                            &resend_to_dustbin_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
  Info("RS_DBI_ES: " << (int)resend_to_dustbin_cmd.engin_start)
}

//************************************** 环卫车清扫命令重发函数
void resend_clean_ctrl_cmd() {
  construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                            sizeof(resend_to_clean_cmd), &resend_to_clean_cmd);
  serial_frame_include_id_type user_frame_include_id_1;
  user_frame_include_id_1.id = 0;
  user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
  user_frame_include_id_1.need_id = 0;
  user_frame_include_id_1.frame = user_frame;
  pushData(&user_frame_include_id_1);
  Info("RS_CC_ES: " << (int)resend_to_clean_cmd.engin_start)
}

//************************************** 定时器3 命令应答超时处理
void timer3Callback() {
  //--命令应答超时处理
  if (!cmd_send_map.empty()) {
    double endtime = rclcpp::Clock().now().seconds();
    double cmd_time_duration = 0;
    std::map<uint16_t, double>::iterator it;
    it = cmd_send_map.begin();
    double starttime = it->second;
    cmd_time_duration = endtime - starttime;
    if (cmd_time_duration >= cmd_answer_timeout) {
      cmd_answer_fail_cnt++;
      ////Info("RECV_ANSWER: "<<" id:"<<int(it->first)<<"
      /// cnt:"<<cmd_answer_fail_cnt);
      // cmd_send_map.erase(it);
      cmd_send_map.clear();
    }
  }
  //--清扫车命令应答处理
  //，发送清扫车命令后，检查状态来判断命令是否执行。(0.02*50)
  //秒检查一次。如果状态不对，重发，最多重发三次。超过三次则超时
  static uint8_t check_clean_machine_status_cnt = 0;
  static uint8_t resend_clean_machine_cmd_cnt = 0;
  if (check_clean_mechine_state && check_clean_machine_status_cnt >= 20) {
    if ((resend_to_dustbin_cmd.engin_start == 1 &&
         recv_clean_mechine_motor_status == 31) ||
        (resend_to_dustbin_cmd.engin_start == 0 &&
         recv_clean_mechine_motor_status == 0)) {
      check_clean_mechine_state = false;
      serial_status.control_clean_mechine = 0;
    }
    if ((resend_to_dustbin_cmd.engin_start == 1 &&
         recv_clean_mechine_motor_status != 31) ||
        (resend_to_dustbin_cmd.engin_start == 0 &&
         recv_clean_mechine_motor_status != 0)) {
      if (robot_type == 0) {
        resend_dustbin_ctrl_cmd();
      } else if (robot_type == 1) {
        resend_clean_ctrl_cmd();
      } else {
        resend_dustbin_ctrl_cmd();
      }
      resend_clean_machine_cmd_cnt++;
      if (resend_clean_machine_cmd_cnt >= 5) {
        check_clean_mechine_state = false;
        serial_status.control_clean_mechine = -1;
        resend_clean_machine_cmd_cnt = 0;
      }
    }
    check_clean_machine_status_cnt = 0;
  }
  check_clean_machine_status_cnt++;
}

//************************************** 定时器4
// rfid接收超时，监控串口接收状态并发布话题
void timer4Callback() {
  //--rfid全部接收超时处理
  if (200 <= recvrfid_loop_num) {
    if (recv_rfid_cnt == recv_rfid_cnt_old) {
      rfid_all.states.clear();
      recv_rfid1_timeout = true;
      recv_rfid2_timeout = true;
      recv_rfid3_timeout = true;
      recv_rfid4_timeout = true;
      oldtabstate_rfid1.clear();
      cti_msgs::msg::TabState tabstate;
      tabstate.id = 1;
      tabstate.status = 140;  // 140 means recv timeout!
      tabstate.message = "";
      oldTabstate_Pushback(&tabstate, tabstate.id);

      oldtabstate_rfid2.clear();
      tabstate.id = 2;
      oldTabstate_Pushback(&tabstate, tabstate.id);

      oldtabstate_rfid3.clear();
      tabstate.id = 3;
      oldTabstate_Pushback(&tabstate, tabstate.id);

      oldtabstate_rfid4.clear();
      tabstate.id = 4;
      oldTabstate_Pushback(&tabstate, tabstate.id);

      rfid_all.header.frame_id = "rfid_all";
      rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid1.begin(),
                             oldtabstate_rfid1.end());
      rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid2.begin(),
                             oldtabstate_rfid2.end());
      rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid3.begin(),
                             oldtabstate_rfid3.end());
      rfid_all.states.insert(rfid_all.states.end(), oldtabstate_rfid4.begin(),
                             oldtabstate_rfid4.end());
      boxrfid_pub_all->publish(rfid_all);
      rfid_all.states.clear();
      Info("R_RF_TMO: " << 1);
      serial_status.data =
          serial_status.data |
          0x0002;  //如果0.02*200 = 4秒內不能接收到rfid,状态位倒数第二位置1
      serial_status.recv_rfid = -1;
    } else {
      serial_status.data =
          serial_status.data &
          0xFFFD;  //如果rfid接收正常 ，状态位倒数第二位重置 为0
      serial_status.recv_rfid = 0;
    }
    recv_rfid_cnt_old = recv_rfid_cnt;
    recvrfid_loop_num = 0;
  }
  recvrfid_loop_num++;
  //--控制信息接收超时处理 0.1s
  if (5 <= recvctrl_loop_num) {
    if (recv_ctrl_cnt == recv_ctrl_cnt_old) {
      serial_status.data =
          serial_status.data |
          0x0001;  //如果控制信息上传 0.1s內没上传，状态位倒数第一位置1
      serial_status.recv_ctrl = -1;
    } else {
      serial_status.data =
          serial_status.data & 0xFFFE;  //如果控制信息正常，状态位倒数第一位置0
      serial_status.recv_ctrl = 0;
    }
    recv_ctrl_cnt_old = recv_ctrl_cnt;
    recvctrl_loop_num = 0;
  }
  recvctrl_loop_num++;
  //--控制信息频率降低处理 1s
  if (50 <= recvctrl_rate_loop_num) {
    float recv_ctrl_rate = abs(recv_ctrl_rate_cnt - recv_ctrl_rate_cnt_old) / 1;
    if (recv_ctrl_rate > 10000) {
      recv_ctrl_rate = 25;
    }
    // node_status_publisher_ptr_->CHECK_MIN_VALUE("/value/cti_fpga_serial/recv_ctrl_rate",recv_ctrl_rate,20,15,"value
    // recv_ctrl_rate is too low");
    Info("R_CT_RA: " << recv_ctrl_rate);
    // printf("recv_ctrl_rate:%f\n",recv_ctrl_rate);
    if (recv_ctrl_rate <= RECV_CTRL_RATE_MIN) {
      serial_status.data =
          serial_status.data |
          0x0010;  //如果控制信息上传频率小于RECV_CTRL_RATE,状态位倒数第五位置1
      serial_status.recv_ctrl_rate = -1;
    } else {
      serial_status.data =
          serial_status.data &
          0xFFEF;  //如果控制信息上传频率大于RECV_CTRL_RATE，状态位倒数第五位置0
      serial_status.recv_ctrl_rate = 0;
    }
    recv_ctrl_rate_cnt_old = recv_ctrl_rate_cnt;
    recvctrl_rate_loop_num = 0;
  }
  recvctrl_rate_loop_num++;
  //--超声波接收超时处理 1s
  if (50 <= recvult_loop_num) {
    if (recv_ult_cnt == recv_ult_cnt_old) {
      serial_status.data =
          serial_status.data |
          0x0004;  //如果超声波信息上传 1秒内没有上传，状态位倒数第3位重置1
      serial_status.recv_ult = -1;
    } else {
      serial_status.data =
          serial_status.data &
          0xFFFB;  //如果超声波信息上传正常，状态位倒数第三位重置0
      serial_status.recv_ult = 0;
    }
    recv_ult_cnt_old = recv_ult_cnt;
    recvult_loop_num = 0;
  }
  recvult_loop_num++;
  //--命令应答接收超时处理 0.5s
  if (25 <= recvcmd_loop_num) {
    if (recv_cmd_cnt == recv_cmd_cnt_old) {
      serial_status.data =
          serial_status.data |
          0x0008;  //如果命令应答0.5s内没有上传，状态位倒数第四位重置1
      serial_status.recv_cmd_answer = -1;
    } else {
      serial_status.data = serial_status.data &
                           0xFFF7;  //如果命令应答正常，状态位倒数第四位重置0
      serial_status.recv_cmd_answer = 0;
    }
    recv_cmd_cnt_old = recv_cmd_cnt;
    recvcmd_loop_num = 0;
  }
  recvcmd_loop_num++;
  //--recv_pthread线程的状态信息处理 0.1s
  if (5 <= recv_pthread_loop_num) {
    if (recv_pthread_cnt == recv_pthread_cnt_old) {
      serial_status.data =
          serial_status.data |
          0x0040;  //如果数据接收线程1s内没有接到新数据，状态倒数第七位置1
      serial_status.recv_pthread_recv = -1;
    } else {
      serial_status.data = serial_status.data &
                           0xFFBF;  //如果数据接收线程正常，状态倒数第七位置0
      serial_status.recv_pthread_recv = 0;
    }
    recv_cmd_cnt_old = recv_cmd_cnt;
    recvcmd_loop_num = 0;

    if (recv_pthread_crc_status == 1) {
      serial_status.data =
          serial_status.data | 0x0080;  //如果数据接受CRC错误，状态倒数第八位置1
      serial_status.recv_pthread_crc = -1;
    } else {
      serial_status.data = serial_status.data & 0xFF7F;
      ;  //如果数据接收线程CRC正确，状态倒数第八位置0
      serial_status.recv_pthread_crc = 0;
    }
  }
  recv_pthread_loop_num++;

  if (control_version_right != 0) {
    serial_status.data =
        serial_status.data | 0x0100;  //如果运控版本错误，状态倒数第九位置1
    serial_status.control_board_version = control_version_right;
  } else {
    serial_status.data = serial_status.data & 0xFEFF;
    ;  //如果运控版本正确，状态倒数第九位置0
    serial_status.control_board_version = 0;
  }
  //--状态信息发布(状态改变时)
  if (old_serial_status != serial_status.data) {
    serial_status_pub->publish(serial_status);
    old_serial_status = serial_status.data;
    Info("SE_ST: " << (int)serial_status.data);
  }
  //-- 状态信息发布(定时发布)
  static uint8_t pub_status_num_loop = 0;
  if (20 <= pub_status_num_loop) {
    serial_status_pub->publish(serial_status);
    pub_status_num_loop = 0;
    Info("SE_ST: " << (int)serial_status.data);
  }
  pub_status_num_loop++;
  //--顶升测试命令超时 1s
  static uint16_t recv_lift_test_num_old = 0;
  static uint16_t recv_lift_test_num_loop = 0;
  if (50 <= recv_lift_test_num_loop) {
    if (recv_lift_test_num == recv_lift_test_num_old) {
      lift_test_flag = false;
    }
    recv_lift_test_num_old = recv_lift_test_num;
    recv_lift_test_num_loop = 0;
  }
  recv_lift_test_num_loop++;
}

void timer6Callback() {
  std_msgs::msg::UInt8 nodeStatus;
  nodeStatus.data = 1;
  node_status_pub->publish(nodeStatus);
  Info(" BEAT: " << 1);
}
void timer7Callback() {
  cti_msgs::msg::TabState set_dustbin_id_state;
  set_dustbin_id_state.id = dustbin_set_state.send_id;
  set_dustbin_id_state.name = "set dustbin id state";
  //一秒发送一次查询
  if (dustbin_set_state.if_send && !dustbin_set_state.if_recv) {
    dustbin_rf_set_cmd_t send_set_dustbin_id;
    send_set_dustbin_id.rw = 0;
    send_set_dustbin_id.baud =
        -1;  //一定要设置成-1 否则会修改运控和lora通信的波特率

    construct_serial_frame_ex(&user_frame, DUSBIN_RF_SET_READ_CMD,
                              sizeof(send_set_dustbin_id),
                              &send_set_dustbin_id);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = DUSBIN_RF_SET_READ_CMD;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
    Info("S_RID: " << 1);
  }

  //如果接收到查询结果,则进行判断
  if (dustbin_set_state.if_recv) {
    if (dustbin_set_state.send_id == dustbin_set_state.recv_id) {
      set_dustbin_id_state.status = 1;  // 1:成功
      set_dustbin_id_state_pub->publish(set_dustbin_id_state);
      dustbin_set_state.reset();
    } else {
      dustbin_set_state.if_recv = false;
      dustbin_set_state.recv_id = -2;
      if (dustbin_set_state.send_id_loop_cnt == 0) {
        set_dustbin_id_state.status = -1;  //-1: 失败
        set_dustbin_id_state_pub->publish(set_dustbin_id_state);
        Info("SID_FA: " << 1);
      }
    }
  }

  // 5秒钟发送一次id设置命令
  dustbin_set_state.send_id_loop_cnt %= 5;
  if (dustbin_set_state.if_send && dustbin_set_state.send_id_loop_cnt == 0) {
    dustbin_rf_set_cmd_t send_set_dustbin_id;
    send_set_dustbin_id.rw = 1;
    send_set_dustbin_id.baud =
        -1;  //一定要设置成-1 否则会修改运控和lora通信的波特率
    send_set_dustbin_id.id = dustbin_set_state.send_id;

    construct_serial_frame_ex(&user_frame, DUSBIN_RF_SET_READ_CMD,
                              sizeof(send_set_dustbin_id),
                              &send_set_dustbin_id);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = DUSBIN_RF_SET_READ_CMD;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);

    dustbin_set_state.send_id_times += 1;
    dustbin_set_state.send_id_times %= 65535;
    Info("S_SID_TM: " << (int)send_set_dustbin_id.id);
  }
  dustbin_set_state.send_id_loop_cnt += 1;

  //如果查询超过3次(15秒),视为超时
  if (dustbin_set_state.send_id_times > 3 && dustbin_set_state.if_send) {
    //已经超时,没有匹配上
    set_dustbin_id_state.status = -2;  //-2: 超时
    set_dustbin_id_state_pub->publish(set_dustbin_id_state);
    Info("SID_TMO: " << 1);
  }

  //箱子类型发生改变,则一秒发一次set_dustbin_on_car直到设置成功
  static uint8_t set_dustbin_on_car_times = 0;
  if (set_dustbin_on_car && robot_type == 0) {
    //切换箱子时发送指令
    if (robot_type == 0) {
      if (4 == box_type) {
        send_to_dustbin_cmd.dustbin_on_car = 1;  //清扫箱在车上
      } else {
        send_to_dustbin_cmd.dustbin_on_car = 0;
      }
      construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                                sizeof(send_to_dustbin_cmd),
                                &send_to_dustbin_cmd);
    }
    // else if(robot_type == 1){
    //     if(4 == box_type){
    //         send_to_clean_cmd.dustbin_on_car = 1;//清扫箱在车上
    //     }else{
    //         send_to_clean_cmd.dustbin_on_car = 0;
    //     }
    // }else{
    //     if(4 == box_type){
    //         send_to_dustbin_cmd.dustbin_on_car = 1;//清扫箱在车上
    //     }else{
    //         send_to_dustbin_cmd.dustbin_on_car = 0;
    //     }
    // }
    construct_serial_frame_ex(&user_frame, SEND_TO_DUSTBIN_CMD,
                              sizeof(send_to_dustbin_cmd),
                              &send_to_dustbin_cmd);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = SEND_TO_DUSTBIN_CMD;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);

    Info("S_BT: " << (int)box_type);
    // printf("setting_box_type: %d, set_times: %d\n",
    // box_type,set_dustbin_on_car_times);
    set_dustbin_on_car_times++;
    if (set_dustbin_on_car_times >= 5) {
      set_dustbin_on_car_times = 0;
      set_dustbin_on_car = false;
      Info("SBT_TMO: " << 1);
    }
  }
}

double get_duration(double in_time) {
  return rclcpp::Clock().now().seconds() - in_time;
}

void timer_chat_timeout_Callbak() {
  static uint8_t in_cnt = 0;
  //开机10s后才开始检测
  if (in_cnt < 10) {
    in_cnt++;
    return;
  }

  cti_msgs::msg::BoxState communicate_msg;
  communicate_msg.header.stamp = rclcpp::Clock().now();
  cti_msgs::msg::TabState chassis_state;
  chassis_state.id = 0;
  cti_msgs::msg::TabState dustbox_state;
  dustbox_state.id = 1;
  //含义定义:
  // id: 模块 0:底盘通讯 1:箱子通讯
  // status: 0:空闲 1:通讯正常 -1:通讯超时
  // message: free:空闲; ok:通讯正常; timeout:通讯超时;

  //底盘通讯检测
  if (get_duration(chassis_chat_state.time_recv) > chassis_chat_timeout_secs) {
    chassis_chat_state.recv_state = false;
  }
  if (!chassis_chat_state.recv_state) {
    //底盘通讯异常
    chassis_state.status = -1;
    chassis_state.message = "timeout";
  } else {
    //底盘通讯正常
    chassis_state.status = 1;
    chassis_state.message = "ok";
  }
  //箱子通讯检测
  if (get_duration(box_chat_state.time_recv) > box_chat_timeout_secs) {
    box_chat_state.recv_state = false;
  }
  if (dustbox_lora_id != -1) {
    if (!box_chat_state.recv_state) {
      //箱子通讯异常
      dustbox_state.status = -1;
      dustbox_state.message = "timeout";
    } else {
      //箱子通讯正常
      dustbox_state.status = 1;
      dustbox_state.message = "ok";
    }
  } else {
    // lora_id没有设置
    //发布通讯空闲状态
    box_chat_state.reset();
    dustbox_state.status = 0;
    dustbox_state.message = "free";
  }
  communicate_msg.states.push_back(chassis_state);
  Info("VEH_CHAT: " << (int)chassis_state.status);
  communicate_msg.states.push_back(dustbox_state);
  Info("DBX_ID: " << (int)box_chat_state.module_id);
  Info("DBX_CHAT: " << (int)dustbox_state.status);
  chat_statue_pub->publish(communicate_msg);
}

void timer_set_process_work_mode_Callback() {
  if (process_twork_mode_t.cur_cnt > 0) {
    process_twork_mode_t.cur_cnt--;
    process_twork_mode_t.process_work_mode_enum_ =
        process_work_mode::process_work_mode_test;
  } else {
    process_twork_mode_t.process_work_mode_enum_ =
        process_work_mode::process_work_mode_normal;
  }
}

void timer_set_ult_mode_Callback() {
  // printf("--------------------vehicle ult_state--------------------\n");
  // printf("vehicle_ult_set_state.state: %d\n",vehicle_ult_set_state.state);
  // printf("vehicle_ult_set_state.set_mode:
  // 0x%016lx\n",vehicle_ult_set_state.set_mode);
  // printf("vehicle_ult_set_state.set_start_time:
  // %lf\n",vehicle_ult_set_state.set_start_time);
  // printf("vehicle_ult_set_state.recv_mode:
  // 0x%016lx\n",vehicle_ult_set_state.recv_mode);
  // printf("vehicle_ult_set_state.check_start_time:
  // %f\n",vehicle_ult_set_state.check_start_time);
  // printf("--------------------dustbox ult_state--------------------\n");
  // printf("dustbox_ult_set_state.state: %d\n",dustbox_ult_set_state.state);
  // printf("dustbox_ult_set_state.set_mode:
  // 0x%016lx\n",dustbox_ult_set_state.set_mode);
  // printf("dustbox_ult_set_state.set_start_time:
  // %lf\n",dustbox_ult_set_state.set_start_time);
  // printf("dustbox_ult_set_state.recv_mode:
  // 0x%016lx\n",dustbox_ult_set_state.recv_mode);
  // printf("dustbox_ult_set_state.check_start_time:
  // %f\n",dustbox_ult_set_state.check_start_time);

  //车身超声波处理
  if (vehicle_ult_set_state.state == 0 || vehicle_ult_set_state.state == 127) {
    //空闲或初始化 do nothing
    vehicle_ult_set_state.check_start_time = 0.0;
    vehicle_ult_set_state.set_start_time = 0.0;
  } else if (vehicle_ult_set_state.state == 1) {
    //发送查询命令
    check_lin_ult_mode_type vehicle_send_check_ult_mode;
    vehicle_send_check_ult_mode.src = MODULE_CONTROL_PC;
    vehicle_send_check_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
    vehicle_send_check_ult_mode.check_ult_Serial =
        0xffffffffffffffff;  //全部查询
    construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE,
                              sizeof(vehicle_send_check_ult_mode),
                              &vehicle_send_check_ult_mode);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
    vehicle_ult_set_state.check_start_time = rclcpp::Clock().now().seconds();
    //更改状态
    vehicle_ult_set_state.state = 2;
  } else if (vehicle_ult_set_state.state == 2) {
    //等待查询结果
    double time_now = rclcpp::Clock().now().seconds();
    if (time_now - vehicle_ult_set_state.check_start_time > 3) {
      vehicle_ult_check_resend_time++;
      if (vehicle_ult_check_resend_time <= 5) {
        //发送查询命令
        check_lin_ult_mode_type vehicle_send_check_ult_mode;
        vehicle_send_check_ult_mode.src = MODULE_CONTROL_PC;
        vehicle_send_check_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
        vehicle_send_check_ult_mode.check_ult_Serial =
            0xffffffffffffffff;  //全部查询
        construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE,
                                  sizeof(vehicle_send_check_ult_mode),
                                  &vehicle_send_check_ult_mode);
        serial_frame_include_id_type user_frame_include_id_1;
        user_frame_include_id_1.id = 0;
        user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
        user_frame_include_id_1.need_id = 0;
        user_frame_include_id_1.frame = user_frame;
        pushData(&user_frame_include_id_1);
        vehicle_ult_set_state.check_start_time =
            rclcpp::Clock().now().seconds();
      } else {
        vehicle_ult_set_state.state = -1;  //查询超时
        vehicle_ult_check_resend_time = 0;
      }
    }
  } else if (vehicle_ult_set_state.state == 3) {
    vehicle_ult_check_resend_time = 0;
    //发送设置命令
    set_lin_ult_mode_type vehicle_send_set_ult_mode;
    vehicle_send_set_ult_mode.src = MODULE_CONTROL_PC;
    vehicle_send_set_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
    vehicle_send_set_ult_mode.mode.data =
        vehicle_ult_set_state.set_mode;  //设置模式
    construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE,
                              sizeof(vehicle_send_set_ult_mode),
                              &vehicle_send_set_ult_mode);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
    vehicle_ult_set_state.set_start_time = rclcpp::Clock().now().seconds();
    //更改状态
    vehicle_ult_set_state.state = 4;
  } else if (vehicle_ult_set_state.state == 4) {
    //等待设置结果
    double time_now = rclcpp::Clock().now().seconds();
    if (time_now - vehicle_ult_set_state.set_start_time > 3) {
      vehicle_ult_set_resend_time++;
      if (vehicle_ult_set_resend_time <= 5) {
        //发送设置命令
        set_lin_ult_mode_type vehicle_send_set_ult_mode;
        vehicle_send_set_ult_mode.src = MODULE_CONTROL_PC;
        vehicle_send_set_ult_mode.dest = MODULE_ULTRASONIC_MASTER;
        vehicle_send_set_ult_mode.mode.data =
            vehicle_ult_set_state.set_mode;  //设置模式
        construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE,
                                  sizeof(vehicle_send_set_ult_mode),
                                  &vehicle_send_set_ult_mode);
        serial_frame_include_id_type user_frame_include_id_1;
        user_frame_include_id_1.id = 0;
        user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
        user_frame_include_id_1.need_id = 0;
        user_frame_include_id_1.frame = user_frame;
        pushData(&user_frame_include_id_1);
        vehicle_ult_set_state.set_start_time = rclcpp::Clock().now().seconds();
      } else {
        vehicle_ult_set_state.state = -2;  //设置超时
        vehicle_ult_set_resend_time = 0;
      }
    }
  }

  //吸尘箱超声波处理
  if (dustbox_ult_set_state.state == 0 || dustbox_ult_set_state.state == 127) {
    //空闲或初始化 do nothing
    dustbox_ult_set_state.check_start_time = 0.0;
    dustbox_ult_set_state.set_start_time = 0.0;
  } else if (dustbox_ult_set_state.state == 1) {
    //发送查询命令
    check_lin_ult_mode_type dustbox_send_check_ult_mode;
    dustbox_send_check_ult_mode.src = MODULE_CONTROL_PC;
    dustbox_send_check_ult_mode.dest = MODULE_DUST_BOX_BOARD;
    dustbox_send_check_ult_mode.check_ult_Serial =
        0xffffffffffffffff;  //全部查询
    construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE,
                              sizeof(dustbox_send_check_ult_mode),
                              &dustbox_send_check_ult_mode);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
    dustbox_ult_set_state.check_start_time = rclcpp::Clock().now().seconds();
    //更改状态
    dustbox_ult_set_state.state = 2;
  } else if (dustbox_ult_set_state.state == 2) {
    //等待查询结果
    double time_now = rclcpp::Clock().now().seconds();
    if (time_now - dustbox_ult_set_state.check_start_time > 3) {
      dustbox_ult_check_resend_time++;
      if (dustbox_ult_check_resend_time <= 5) {
        //发送查询命令
        check_lin_ult_mode_type dustbox_send_check_ult_mode;
        dustbox_send_check_ult_mode.src = MODULE_CONTROL_PC;
        dustbox_send_check_ult_mode.dest = MODULE_DUST_BOX_BOARD;
        dustbox_send_check_ult_mode.check_ult_Serial =
            0xffffffffffffffff;  //全部查询
        construct_serial_frame_ex(&user_frame, CHECK_LIN_ULT_MODE,
                                  sizeof(dustbox_send_check_ult_mode),
                                  &dustbox_send_check_ult_mode);
        serial_frame_include_id_type user_frame_include_id_1;
        user_frame_include_id_1.id = 0;
        user_frame_include_id_1.cmd = CHECK_LIN_ULT_MODE;
        user_frame_include_id_1.need_id = 0;
        user_frame_include_id_1.frame = user_frame;
        pushData(&user_frame_include_id_1);
        dustbox_ult_set_state.check_start_time =
            rclcpp::Clock().now().seconds();
      } else {
        dustbox_ult_set_state.state = -1;  //查询超时
        dustbox_ult_check_resend_time = 0;
      }
    }
  } else if (dustbox_ult_set_state.state == 3) {
    dustbox_ult_check_resend_time = 0;
    //发送设置命令
    set_lin_ult_mode_type dustbox_send_set_ult_mode;
    dustbox_send_set_ult_mode.src = MODULE_CONTROL_PC;
    dustbox_send_set_ult_mode.dest = MODULE_DUST_BOX_BOARD;
    dustbox_send_set_ult_mode.mode.data =
        dustbox_ult_set_state.set_mode;  //设置模式
    construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE,
                              sizeof(dustbox_send_set_ult_mode),
                              &dustbox_send_set_ult_mode);
    serial_frame_include_id_type user_frame_include_id_1;
    user_frame_include_id_1.id = 0;
    user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
    user_frame_include_id_1.need_id = 0;
    user_frame_include_id_1.frame = user_frame;
    pushData(&user_frame_include_id_1);
    dustbox_ult_set_state.set_start_time = rclcpp::Clock().now().seconds();
    //更改状态
    dustbox_ult_set_state.state = 4;
  } else if (dustbox_ult_set_state.state == 4) {
    //等待设置结果
    double time_now = rclcpp::Clock().now().seconds();
    if (time_now - dustbox_ult_set_state.set_start_time > 3) {
      dustbox_ult_set_resend_time++;
      if (dustbox_ult_set_resend_time <= 5) {
        //发送设置命令
        set_lin_ult_mode_type dustbox_send_set_ult_mode;
        dustbox_send_set_ult_mode.src = MODULE_CONTROL_PC;
        dustbox_send_set_ult_mode.dest = MODULE_DUST_BOX_BOARD;
        dustbox_send_set_ult_mode.mode.data =
            dustbox_ult_set_state.set_mode;  //设置模式
        construct_serial_frame_ex(&user_frame, SET_LIN_ULT_MODE,
                                  sizeof(dustbox_send_set_ult_mode),
                                  &dustbox_send_set_ult_mode);
        serial_frame_include_id_type user_frame_include_id_1;
        user_frame_include_id_1.id = 0;
        user_frame_include_id_1.cmd = SET_LIN_ULT_MODE;
        user_frame_include_id_1.need_id = 0;
        user_frame_include_id_1.frame = user_frame;
        pushData(&user_frame_include_id_1);
        dustbox_ult_set_state.set_start_time = rclcpp::Clock().now().seconds();
      } else {
        dustbox_ult_set_state.state = -2;  //设置超时
        dustbox_ult_set_resend_time = 0;
      }
    }
  }
#if DEBUG_PRINT
  printf("--------------------vehicle ult_state--------------------\n");
  printf("vehicle_ult_set_state.state: %d\n", vehicle_ult_set_state.state);
  printf("vehicle_ult_set_state.set_mode: %08x\n",
         vehicle_ult_set_state.set_mode);
  printf("vehicle_ult_set_state.set_start_time: %lf\n",
         vehicle_ult_set_state.set_start_time);
  printf("vehicle_ult_set_state.recv_mode: %08x\n",
         vehicle_ult_set_state.recv_mode);
  printf("vehicle_ult_set_state.check_start_time: %f\n",
         vehicle_ult_set_state.check_start_time);
  printf("--------------------dustbox ult_state--------------------\n");
  printf("dustbox_ult_set_state.state: %d\n", dustbox_ult_set_state.state);
  printf("dustbox_ult_set_state.set_mode: %08x\n",
         dustbox_ult_set_state.set_mode);
  printf("dustbox_ult_set_state.set_start_time: %lf\n",
         dustbox_ult_set_state.set_start_time);
  printf("dustbox_ult_set_state.recv_mode: %08x\n",
         dustbox_ult_set_state.recv_mode);
  printf("dustbox_ult_set_state.check_start_time: %f\n",
         dustbox_ult_set_state.check_start_time);
#endif
  Info("ULT_ST: "
       << " V_ULT_ST_ST: " << (int)vehicle_ult_set_state.state
       << " V_ULT_ST_SM: " << vehicle_ult_set_state.set_mode
       << " V_ULT_ST_SMT: " << vehicle_ult_set_state.set_start_time
       << " V_ULT_ST_RM: " << vehicle_ult_set_state.recv_mode
       << " V_ULT_ST_RMT: " << vehicle_ult_set_state.check_start_time
       << " D_ULT_ST_ST: " << (int)dustbox_ult_set_state.state
       << " D_ULT_ST_SM: " << dustbox_ult_set_state.set_mode
       << " D_ULT_ST_SMT: " << dustbox_ult_set_state.set_start_time
       << " D_ULT_ST_RM: " << dustbox_ult_set_state.recv_mode
       << " D_ULT_ST_RMT: " << dustbox_ult_set_state.check_start_time)
}
int get_port_type(std::string port_type_name) {
  if (port_type_name == "serial") {
    port_type = PORT_SERIAL;
    return 0;
  } else if (port_type_name == "udp") {
    port_type = PORT_UDP;
    return 0;
  } else {
    return -1;
  }
}
//************************************** 串口发送数据
void sendSerialData(void) {
  if (!databuf.empty()) {
    send_single_serial_frame(&(databuf.front().frame), 0);
    databuf.pop_front();
  }
}
//************************************** 网口发送数据
void sendUdpData(void) {
  if (!databuf.empty()) {
    send_single_udp_frame(&(databuf.front().frame), 0);
    databuf.pop_front();
  }
}

void INIT_ROS_SET(rclcpp::Node::SharedPtr &node_) {
  std::string cmd_answer_topic;
  std::string version_topic;
  double timer2_duration;
  int pub_hz;
  bool broadcast_tf;
  int send_to_dustbox_fanspeed;
  int send_to_damboard_control;
  //----------------------获取参数
  //通讯端口类型及参数  "serial":串口通讯 "udp":网口udp通讯
  node_->declare_parameter("port_type", std::string("serial"));
  port_type_name = node_->get_parameter("port_type").as_string();
  //串口通讯参数
  node_->declare_parameter("port_name", std::string("/dev/cti_fpga"));
  serial_port = node_->get_parameter("port_name").as_string();
  //网口通讯参数
  node_->declare_parameter("udp_ip", std::string("192.168.1.102"));
  udp_ip = node_->get_parameter("udp_ip").as_string();
  node_->declare_parameter("udp_port", 8888);
  udp_port = node_->get_parameter("udp_port").as_int();
  node_->declare_parameter("udp_ip_dest", std::string("192.168.1.222"));
  udp_ip_dest = node_->get_parameter("udp_ip_dest").as_string();
  node_->declare_parameter("udp_port_dest", 8887);
  udp_port_dest = node_->get_parameter("udp_port_dest").as_int();

  node_->declare_parameter("pub_hz", 50);
  pub_hz = node_->get_parameter("pub_hz").as_int();

  node_->declare_parameter("broadcast_tf", false);
  broadcast_tf = node_->get_parameter("broadcast_tf").as_bool();

  node_->declare_parameter("timer2_duration", 1.0);
  timer2_duration = node_->get_parameter("timer2_duration").as_double();

  node_->declare_parameter(
      "version_topic", std::string("/cti/fpga_serial/operationControlVersion"));
  version_topic = node_->get_parameter("version_topic").as_string();

  node_->declare_parameter("cmd_answer_topic",
                           "/cti/fpga_serial/cmd_answer_cnt");
  cmd_answer_topic = node_->get_parameter("cmd_answer_topic").as_string();

  node_->declare_parameter("car_wheel_base", 0.8);
  car_wheel_base = node_->get_parameter("car_wheel_base").as_double();

  node_->declare_parameter("cmd_answer_timeout", 0.01);
  cmd_answer_timeout = node_->get_parameter("cmd_answer_timeout").as_double();

  node_->declare_parameter("max_control_board_version_head", 0);
  max_control_board_version_head_ =
      node_->get_parameter("max_control_board_version_head").as_int();
  node_->declare_parameter("min_control_board_version_head", 0);
  min_control_board_version_head_ =
      node_->get_parameter("min_control_board_version_head").as_int();
  node_->declare_parameter("max_control_board_version_mid", 0);
  max_control_board_version_mid_ =
      node_->get_parameter("max_control_board_version_mid").as_int();
  node_->declare_parameter("min_control_board_version_mid", 0);
  min_control_board_version_mid_ =
      node_->get_parameter("min_control_board_version_mid").as_int();
  node_->declare_parameter("max_control_board_version_end", 0);
  max_control_board_version_end_ =
      node_->get_parameter("max_control_board_version_end").as_int();
  node_->declare_parameter("min_control_board_version_end", 0);
  min_control_board_version_end_ =
      node_->get_parameter("min_control_board_version_end").as_int();
  node_->declare_parameter("lift_test_switch", false);
  lift_test_switch_ = node_->get_parameter("lift_test_switch").as_bool();
  node_->declare_parameter("default_sweeper_id", 0);
  default_sweeper_id = node_->get_parameter("default_sweeper_id").as_int();
  node_->declare_parameter("default_dustbox_fanspeed", 80);
  send_to_dustbox_fanspeed =
      node_->get_parameter("default_dustbox_fanspeed").as_int();
  send_to_dust_box_cmd.fan_speed = send_to_dustbox_fanspeed;
  node_->declare_parameter("default_damboard_control", 0);
  send_to_damboard_control =
      node_->get_parameter("default_damboard_control").as_int();
  send_to_clean_cmd.dam_board = send_to_damboard_control;
  node_->declare_parameter("robot_type", 0);
  robot_type = node_->get_parameter("robot_type").as_int();

  node_->declare_parameter("CTI_RUN_VER", "");
  cti_run_ver = node_->get_parameter("CTI_RUN_VER").as_string();

  node_->declare_parameter("default_side_brush_transform", 0);
  default_side_brush_transform =
      node_->get_parameter("default_side_brush_transform").as_int();
  node_->declare_parameter("default_side_brush_speed", 0);
  default_side_brush_speed =
      node_->get_parameter("default_side_brush_speed").as_int();
  node_->declare_parameter("LIN_ult_installed", 0);
  LIN_ult_installed = node_->get_parameter("LIN_ult_installed").as_int();
  // printf("LIN_ult_installed: %d\n",LIN_ult_installed);

  node_->declare_parameter("config_file_path", "/home/neousys");
  config_file_path = node_->get_parameter("config_file_path").as_string();
  config_file_path = config_file_path + "/config.yaml";
  // printf("config_file_path: %s\n",config_file_path.c_str());
  node_->declare_parameter("water_out_per_sec", 0.0139);
  vehicle_water_status.out_per_sec =
      node_->get_parameter("water_out_per_sec").as_double();
  node_->declare_parameter("water_in_per_sec", 0.1);
  vehicle_water_status.in_per_sec =
      node_->get_parameter("water_in_per_sec").as_double();
  node_->declare_parameter("localization_limit", true);
  localization_limit = node_->get_parameter("localization_limit").as_bool();
  // printf("localization_limit: %s\n",localization_limit?"true":"false");
  node_->declare_parameter("obstatle_disobs_limit", true);
  obstatle_disobs_limit =
      node_->get_parameter("obstatle_disobs_limit").as_bool();
  node_->declare_parameter("chassis_chat_timeout_secs", 10.0);
  chassis_chat_timeout_secs =
      node_->get_parameter("chassis_chat_timeout_secs").as_double();
  node_->declare_parameter("box_chat_timeout_secs", 10.0);
  box_chat_timeout_secs =
      node_->get_parameter("box_chat_timeout_secs").as_double();
  node_->declare_parameter("msg_max_range", 10.0);
  msg_max_range = node_->get_parameter("msg_max_range").as_double();
  // printf("obstatle_disobs_limit: %s\n",obstatle_disobs_limit?"true":"false");

  //----------------------ros话题订阅定义
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      sub_cmd_vel =
          node_->create_subscription<geometry_msgs::msg::TwistStamped>(
              "/cmd_vel", 10, cmd_vel_callback);

  rclcpp::Subscription<cti_msgs::msg::TargetPose>::SharedPtr
      sub_contraposition =
          node_->create_subscription<cti_msgs::msg::TargetPose>(
              "/box_pose", 10, boxContraposition_Callback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_sub =
      node_->create_subscription<std_msgs::msg::Int32>("cti/fpga_serial/boxcmd",
                                                       10, boxUpDown_Callback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr boxlock_sub =
      node_->create_subscription<std_msgs::msg::Int32>(
          "cti/fpga_serial/boxlockcmd", 10, boxLock_Callback);

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr poweroff_sub =
      node_->create_subscription<std_msgs::msg::Int16MultiArray>(
          "cti/fpga_serial/poweroffcmd", 10, powerOff_Callback);

  rclcpp::Subscription<cti_fpga_serial_msgs::msg::UpdateInfo>::SharedPtr
      stm32_sub =
          node_->create_subscription<cti_fpga_serial_msgs::msg::UpdateInfo>(
              "cti/fpga_serial/stmupdate", 10, stmUpdate_Callback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light_sub =
      node_->create_subscription<std_msgs::msg::Int32>(
          "cti/fpga_serial/light_type", 2, lightType_Callback);

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr light_v3_0_sub =
      node_->create_subscription<cti_msgs::msg::DataArray>(
          "cti/fpga_serial/light_type_v3_0", 20, lightType_v3_0_Callback);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/lidar_pose", 10, position_Callback);

  rclcpp::Subscription<cti_msgs::msg::AutoTransmission>::SharedPtr disObs_sub =
      node_->create_subscription<cti_msgs::msg::AutoTransmission>(
          "/cti/obstacle/disObs", 10, disObs_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr checkversion_sub =
      node_->create_subscription<std_msgs::msg::UInt8>(
          "/cti/fpga_serial/checkversion", 1, checkversion_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr formatsdcard_sub =
      node_->create_subscription<std_msgs::msg::UInt8>(
          "/cti/fpga_serial/formatsdcard", 1, formatsdcard_Callback);

  rclcpp::Subscription<cti_msgs::msg::RobotLocalizerState>::SharedPtr
      localizerState_sub =
          node_->create_subscription<cti_msgs::msg::RobotLocalizerState>(
              "/cti_localizer_state", 1, localizerState_Callback);

  rclcpp::Subscription<cti_msgs::msg::DustbinControl>::SharedPtr
      dustbin_control_sub;
  rclcpp::Subscription<cti_msgs::msg::DustbinControlNew>::SharedPtr
      dustbin_control_sub_new;
  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dustbin_control_info_sub;

  // poweroff_cmd_type(const std_msgs::msg::UInt32 msg)
  if (robot_type == 1) {
    dustbin_control_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control", 1, cleanControl_Callback);
    dustbin_control_sub_new =
        node_->create_subscription<cti_msgs::msg::DustbinControlNew>(
            "/cti/chassis_serial/dustbin_control_new", 1,
            cleanControlNew_Callback);
    dustbin_control_info_sub =
        node_->create_subscription<cti_msgs::msg::DataArray>(
            "/cti/chassis_serial/sanitation_vehicle_control", 1,
            cleanControlInfo_Callback);
  } else if (robot_type == 0) {
    dustbin_control_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control", 1, dustbinControl_Callback);
    dustbin_control_info_sub =
        node_->create_subscription<cti_msgs::msg::DataArray>(
            "/cti/chassis_serial/sanitation_vehicle_control", 1,
            cleanControlInfo_Callback);
  } else {
    dustbin_control_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control", 1, dustbinControl_Callback);
    dustbin_control_info_sub =
        node_->create_subscription<cti_msgs::msg::DataArray>(
            "/cti/chassis_serial/sanitation_vehicle_control", 1,
            cleanControlInfo_Callback);
  }
  rclcpp::Subscription<cti_msgs::msg::DustbinControl>::SharedPtr
      dustbin_control_resend_sub;
  if (robot_type == 1) {
    dustbin_control_resend_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control_include_resend", 1,
            cleanControlResend_Callback);
  } else if (robot_type == 0) {
    dustbin_control_resend_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control_include_resend", 1,
            dustbinControlResend_Callback);
  } else {
    dustbin_control_resend_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control_include_resend", 1,
            dustbinControlResend_Callback);
  }
  if (lift_test_switch_) {
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lift_test_sub =
        node_->create_subscription<std_msgs::msg::Int32>(
            "cti/fpga_serial/jacking_test", 1, lifttest_Callback);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_type_sub =
      node_->create_subscription<std_msgs::msg::Int32>("/cti/rblite/boxtype", 1,
                                                       boxtype_Callback);

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sweeper_box_id_sub =
      node_->create_subscription<std_msgs::msg::Int64>(
          "/cti/chassis_serial/set_sweeper_box_id", 1, sweeperBoxID_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      wireless_charge_control_sub =
          node_->create_subscription<std_msgs::msg::UInt8>(
              "/cti/chassis_serial/wireless_charge_control", 1,
              wirelessCharge_Callback);

  rclcpp::Subscription<cti_msgs::msg::DustbinControl>::SharedPtr
      dust_box_control_sub =
          node_->create_subscription<cti_msgs::msg::DustbinControl>(
              "/cti/chassis_serial/dust_box_control", 1,
              dustBoxControl_Callback);

  rclcpp::Subscription<cti_msgs::msg::DustboxControl>::SharedPtr
      dust_box_control_sub_new =
          node_->create_subscription<cti_msgs::msg::DustboxControl>(
              "/cti/chassis_serial/dust_box_control_new", 1,
              dustBoxControlNew_Callback);

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dust_box_control_info_sub =
          node_->create_subscription<cti_msgs::msg::DataArray>(
              "/cti/chassis_serial/sanitation_dustbox_control", 1,
              dustBoxControlInfo_Callback);  //环卫车集尘箱控制
                                             // cti_msgs::msg::DataArray消息类型
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      dust_box_autopush_control_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dust_box_fan_speed_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dustbin_damboard_sub;
  if (robot_type == 1) {
    dust_box_autopush_control_sub =
        node_->create_subscription<std_msgs::msg::UInt8>(
            "/cti/chassis_serial/dust_box_autopush_control", 1,
            dustBoxAutopushControl_Callback);

    dust_box_fan_speed_sub = node_->create_subscription<std_msgs::msg::UInt8>(
        "/cti/chassis_serial/dust_box_fanspeed_control", 1,
        dustBoxFanSpeed_Callback);

    dustbin_damboard_sub = node_->create_subscription<std_msgs::msg::UInt8>(
        "/cti/chassis_serial/dustbin_damboard_control", 1,
        dustbinDamboard_Callback);
  }
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      dustbin_sideBrushTrans_sub =
          node_->create_subscription<std_msgs::msg::UInt8>(
              "/cti/chassis_serial/dustbin_sideBrushTrans_control", 1,
              dustbinSideBrushTrans_Callback);  //测试用的,单独控制边刷伸展

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr smart_trash_sub =
      node_->create_subscription<cti_msgs::msg::DataArray>(
          "/cti/chassis_serial/smart_trash_control", 1, smartTrash_Callback);

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dustbox_5g_check_sub =
          node_->create_subscription<cti_msgs::msg::DataArray>(
              "/cti/chassis_serial/sanitation_dustbox_5g_check", 1,
              dustbox5GCheck_Callback);
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ledshow_sub =
      node_->create_subscription<std_msgs::msg::Int32>("/cti/ledshow/cmd", 1,
                                                       ledshow_Callback);
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ult_cmd_sub_ =
      node_->create_subscription<std_msgs::msg::Int64>(
          "/cti/ultrasonic/ult_cmd", 1, ultCmdCallback);

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr
      power_control_sub =
          node_->create_subscription<std_msgs::msg::UInt32MultiArray>(
              "/cti/fpga_serial/power_control", 1, poweroff_cmd_type_callback);

  //----------------------ros话题发布定义
  // odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/robot_odom",
  // 10);
  odom_pub_4wd =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot_odom_4wd", 10);
  odom_pub_calc =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot_odom_calc", 10);
  sins_pub =
      node_->create_publisher<cti_msgs::msg::Sins>("/cti/fpga_serial/sins", 10);

  imudata_pub = node_->create_publisher<sensor_msgs::msg::Imu>(
      "/cti/fpga_serial/imu", 10);
  // steer_pub =
  // node_->create_publisher<std_msgs::msg::Float64>("/robot_steer_angle",
  // 10);//2020年4月2日停用
  battery_pub = node_->create_publisher<cti_msgs::msg::BatteryState>(
      "/cti/cti_fpga/battery_state", 10);
  batcell_pub = node_->create_publisher<cti_msgs::msg::BatteryCellsState>(
      "/cti/cti_fpga/batcell_state", 10);
  // info_pub =
  // node_->create_publisher<std_msgs::msg::Float64MultiArray>("/cti/fpga_serial/run_info",
  // 10); //2020年4月2日停用
  ctlinfo_pub = node_->create_publisher<cti_msgs::msg::VehicleCtlRunInfo>(
      "/cti/fpga_serial/ctlrun_info", 10);

  state_pub = node_->create_publisher<std_msgs::msg::String>(
      "/cti/fpga_serial/error", 10);
  stm32_pub = node_->create_publisher<cti_fpga_serial_msgs::msg::UpdateInfo>(
      "/cti/fpga_serial/stminfo", 10);

  cmd_answer_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::CmdAnswer>(
          cmd_answer_topic, 10);

  boxrfid_pub_all = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/fpga_serial/rfid_all", 10);

  boxrfid_pub_single = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/fpga_serial/rfid_single", 10);

  ranges_pub = node_->create_publisher<cti_msgs::msg::Range>(
      "/cti/fpga_serial/multiple_ultrasonic", 1);

  for (int i = 0; i < max_type_ult; i++) {
    range_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/fpga_serial/" + ult_name[i], 1);
  }
  for (int i = 0; i < sweeper_max_type_ult; i++) {
    sweeper_range_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + sweeper_ult_name[i], 1);
  }
  for (int i = 0; i < SWEEPER_ULT_MODULE_NUM; i++) {
    for (int j = 0; j < new_sweeper_max_type_ult; j++) {
      new_sweeper_range_pub[i][j] =
          node_->create_publisher<sensor_msgs::msg::Range>(
              "/cti/chassis_serial/" + new_sweeper_ult_name[i][j], 1);
    }
  }
  for (int i = 0; i < ULT_3_0_TYPE_NUM; i++) {
    alt_3_0_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + ult_3_0_name[i], 1);
  }

  //集尘箱超声波发布
  for (int i = 0; i < dustbox_max_type_ult; i++) {
    dustbox_range_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + dustbox_ult_name[i], 1);
  }
  //车身lin通信超声波原生数据发布
  raw_lin_range_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::RangeRawData>(
          "/cti/chassis_serial/range_raw_data", 25);
  firmvion_pub = node_->create_publisher<cti_msgs::msg::RobotVersionDisplay>(
      "/cti/fpga_serial/operationControlVersion",
      rclcpp::QoS{1}.transient_local());
  serial_status_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::SerialStatus>(
          "/cti/fpga_serial/serial_status", 1);
  box_laser_pub = node_->create_publisher<cti_msgs::msg::Rtcm>(
      "/cti/fpga_serial/box_laser", 10);
  chassis_error_pub = node_->create_publisher<std_msgs::msg::UInt32MultiArray>(
      "/cti/chassis_serial/chassis_error", 10);
  navigation_log_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::NavigationLog>(
          "/cti/fpga_serial/navigation_log", 10);
  baro_status_pub = node_->create_publisher<std_msgs::msg::UInt16>(
      "/cti/chassis_serial/baro_status", 10);
  formatsdcard_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/fpga_serial/formatsdcard_result", 1);
  gps_pub = node_->create_publisher<cti_msgs::msg::GnssRTK>(
      "/cti/chassis_serial/gnss", 10);
  compass_pub = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/cti/chassis_serial/compass", 10);
  node_status_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/node_active", 25);
  dustbin_state_pub = node_->create_publisher<cti_msgs::msg::DustbinState>(
      "/cti/chassis_serial/dustbin_state",
      25);  //新的环卫车清扫状态发布,将以前拆开发布的数据放在这里发布
  //新的环卫车清扫状态发布,将以前拆开发布的数据放在这里发布
  dustbin_state_pub_new =
      node_->create_publisher<cti_msgs::msg::DustbinStateNew>(
          "/cti/chassis_serial/dustbin_state_new", 25);
  firmware_version_status_pub = node_->create_publisher<std_msgs::msg::Int8>(
      "/cti/chassis_serial/control_board_version_status", 1);
  firmware_version_check_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::FirmWareInfo>(
          "/cti/chassis_serial/FW_check_report", 1);
  recv_chassis_info_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::VehicleState>(
          "/cti/chassis_serial/chassis_info", 10);
  set_dustbin_id_state_pub = node_->create_publisher<cti_msgs::msg::TabState>(
      "/cti/chassis_serial/dustbin_set_id_state", 1);
  recv_dustbin_id_state_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::DustbinidState>(
          "/cti/chassis_serial/dustbin_id_state", 1);
  dust_box_state_pub = node_->create_publisher<cti_msgs::msg::DustbinState>(
      "/cti/chassis_serial/dust_box_state", 25);
  dust_box_state_pub_new = node_->create_publisher<cti_msgs::msg::DustboxState>(
      "/cti/chassis_serial/dust_box_state_new", 25);
  dust_box_state_info_pub = node_->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/sanitation_dustbox_state",
      25);  // cti_msgs/DataArray类型//环卫车,吸尘箱状态发布
  dust_box_autopush_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dust_box_autopush_state", 25);
  dust_box_fanspeed_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dust_box_fanspeed_state", 25);
  dustbin_damboard_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dustbin_damboard_state", 25);
  dustbox_wireless_charge_state_pub =
      node_->create_publisher<cti_msgs::msg::BatteryState>(
          "/cti/chassis_serial/dustbox_wireless_charge_state", 2);
  dustbox_bottom_range_pub = node_->create_publisher<sensor_msgs::msg::Range>(
      "/cti/chassis_serial/dustbox_bottom", 2);
  boxlock_state_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/boxlock_state", 25);
  dust_box_state_pub_json = node_->create_publisher<std_msgs::msg::String>(
      "/cti/chassis_serial/dust_box_state_json", 25);
  rain_sensor_pub = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/chassis_serial/rain_sensor", 25);
  smart_trash_state_pub = node_->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/smart_trash_state", 25);
  dust_vehicle_state_info_pub =
      node_->create_publisher<cti_msgs::msg::DataArray>(
          "/cti/chassis_serial/sanitation_vehicle_state",
          25);  // cti_msgs/DataArray类型//环卫车,车辆清扫状态发布
  dustbox_5g_state_pub = node_->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/sanitation_dustbox_5g_state",
      25);  //吸尘箱5g状态发布
  dustbox_batterycell_pub =
      node_->create_publisher<cti_msgs::msg::BatteryCellsState>(
          "/cti/chassis_serial/dustbox_batcell_state", 25);  //吸尘箱总电量发布
  chat_statue_pub = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/chassis_serial/communication_state", 1);  //通信状态发布
  lin_ult_data_pub = node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "/cti/chassis_serial/lin_ult_data", 1);  // lin通信超声波源数据发布
  dustbox_rear_range_pub =
      node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
          "/cti/chassis_serial/sanitation_dustbox_ult",
          2);  //吸尘箱超声波原始数据发送
  lin_ult_data_v3_0_pub = node_->create_publisher<
      std_msgs::msg::UInt16MultiArray>(
      "/cti/chassis_serial/lin_ult_data_v3_0",
      1);  //环卫车3.0lin通信超声波源数据发布 =
           // node_->create_publisher<std_msgs::msg::UInt16MultiArray>("/cti/chassis_serial/lin_ult_data",1);//lin通信超声波源数据发布

  new_lin_ult_data_v3_0_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::UltV30Datas>(
          "/cti/chassis_serial/new_lin_ult_data_v3_0",
          1);  // new环卫车3.0lin通信超声波源数据发布 =

  //----------------------ros定时器定义
  rclcpp::TimerBase::SharedPtr timer =
      node_->create_wall_timer(1s, &timerCallback);
  rclcpp::TimerBase::SharedPtr timer2 = node_->create_wall_timer(
      std::chrono::duration<double>(timer2_duration), &timer2Callback);
  rclcpp::TimerBase::SharedPtr timer3 = node_->create_wall_timer(
      std::chrono::duration<double>(cmd_answer_timeout), &timer3Callback);
  rclcpp::TimerBase::SharedPtr timer4 = node_->create_wall_timer(
      std::chrono::duration<double>(RECVTIMEOUT_DUR), &timer4Callback);
  //关闭timer5以后，接到底盘发布的错误消息会发布出去，打开timer5,平时会发底盘正常的消息
  // rclcpp::TimerBase::SharedPtr timer5 =
  // node_->create_wall_timer(0.25s,&timer5Callback);
  rclcpp::TimerBase::SharedPtr timer6 =
      node_->create_wall_timer(0.04s, &timer6Callback);
  rclcpp::TimerBase::SharedPtr timer7 =
      node_->create_wall_timer(1s, &timer7Callback);  //清扫箱箱号设置和获取
  rclcpp::TimerBase::SharedPtr timer_chat_timeout =
      node_->create_wall_timer(1s, &timer_chat_timeout_Callbak);  //通信超时检测
  rclcpp::TimerBase::SharedPtr timer_set_ult_mode = node_->create_wall_timer(
      0.5s, &timer_set_ult_mode_Callback);  //设置超声波模式
}

void INIT_DATAS() {
  //车身lin超声波默认模式赋值
  //车身前右，后右，左前下，右前下，其他设置为又收又发，车身有效位设置对应值,无效位全部为1
  vehicle_defalt_linult_mode.bits.lin_front_left = 2;
  vehicle_defalt_linult_mode.bits.lin_front_right = 1;
  vehicle_defalt_linult_mode.bits.lin_back_center = 2;
  vehicle_defalt_linult_mode.bits.lin_rear_left = 2;
  vehicle_defalt_linult_mode.bits.lin_rear_right = 1;
  vehicle_defalt_linult_mode.bits.unused1 = 0xffff;
  vehicle_defalt_linult_mode.bits.lin_left_front_up = 2;
  vehicle_defalt_linult_mode.bits.lin_left_front_down = 1;
  vehicle_defalt_linult_mode.bits.lin_left_center = 2;
  vehicle_defalt_linult_mode.bits.lin_right_center = 2;
  vehicle_defalt_linult_mode.bits.lin_right_front_down = 1;
  vehicle_defalt_linult_mode.bits.lin_right_front_up = 2;
  vehicle_defalt_linult_mode.bits.unused2 = 0xffff;
  vehicle_defalt_linult_mode.bits.lin_dustbox_rear_right = 0xff;
  vehicle_defalt_linult_mode.bits.lin_dustbox_rear_left = 0xff;
  vehicle_defalt_linult_mode.bits.lin_dustbox_bottom = 0xff;
  vehicle_defalt_linult_mode.bits.unused3 = 0xffff;
  vehicle_defalt_linult_mode.bits.unused4 = 0xffff;
  //车身lin超声波 全部关闭模式赋值，车身有效位全部为0,无效位全部为1
  vehicle_all_stop_work_mode.data = 0;
  vehicle_all_stop_work_mode.bits.unused1 = 0xffff;
  vehicle_all_stop_work_mode.bits.unused2 = 0xffff;
  vehicle_all_stop_work_mode.bits.unused3 = 0xffff;
  vehicle_all_stop_work_mode.bits.unused4 = 0xffff;
  vehicle_all_stop_work_mode.bits.lin_dustbox_rear_right = 0xff;
  vehicle_all_stop_work_mode.bits.lin_dustbox_rear_left = 0xff;
  vehicle_all_stop_work_mode.bits.lin_dustbox_bottom = 0xff;
  //吸尘箱lin超声波默认模式赋值 箱子后右 设置为只收不发，
  //其他设置为又收又发，箱子有效位设置为对应值，其余全为1
  dustbox_defalt_linult_mode.data = 0xffffffffffffffff;
  dustbox_defalt_linult_mode.bits.lin_dustbox_rear_right = 1;
  dustbox_defalt_linult_mode.bits.lin_dustbox_rear_left = 2;
  dustbox_defalt_linult_mode.bits.lin_dustbox_bottom = 2;
  //吸尘箱lin超声波 全部关闭模式赋值，吸尘箱有效位全部为0,无效位全部为1
  dustbox_all_stop_work_mode.data = 0xffffffffffffffff;
  dustbox_all_stop_work_mode.bits.lin_dustbox_rear_right = 0x00;
  dustbox_all_stop_work_mode.bits.lin_dustbox_rear_left = 0x00;
  dustbox_all_stop_work_mode.bits.lin_dustbox_bottom = 0x00;

  // printf("vehicle_defalt_linult_mode.data:
  // 0x%016lx\n",vehicle_defalt_linult_mode.data);
  // printf("vehicle_all_stop_work_mode.data:
  // 0x%016lx\n",vehicle_all_stop_work_mode.data);
  // printf("dustbox_defalt_linult_mode.data:
  // 0x%016lx\n",dustbox_defalt_linult_mode.data);
  // printf("dustbox_all_stop_work_mode.data:
  // 0x%016lx\n",dustbox_all_stop_work_mode.data);

  //----------------------设串口通信sdk
  // std::cout<<"open port started! "<<std::endl;
  //   strcpy((char *)SERIAL_DEVICE_PORT, serial_port.c_str());
  // strcpy((char *)SERIAL_DEVICE_PORT, "/dev/serialLock0");
  // strcpy((char *)SERIAL_DEVICE_PORT, "/dev/ttyUSB0");
  // process_nomal_cmd_cb = process_nomal_cmd_ex;
  // process_update_cmd_cb = process_update_cmd_ex;
  // module_serial_init();
  // auto period = std::chrono::milliseconds(100);
  // rclcpp::WallRate loop_rate(10ms);
  //----------------------------
  // initLog(filelog);
  //--
  // databuf.clear();
  // std::cout<<"open port finished! "<<std::endl;
  //---------------------创建接收数据的线程
  // int p_ret = 0;
  // p_ret = pthread_create(&th_serial_recv, NULL, serial_recv_thread, NULL);
  // if(p_ret != 0)
  //{
  // printf("serial thread creat error!\n");
  //}
  //----------------------给每一个RFID容器赋初值，防止boxload_callback判断错误
  cti_msgs::msg::TabState tabstate;
  oldtabstate_rfid1.push_back(tabstate);
  oldtabstate_rfid2.push_back(tabstate);
  oldtabstate_rfid3.push_back(tabstate);
  oldtabstate_rfid4.push_back(tabstate);
  //-----------------------serial_status赋初值
  serial_status.recv_from_dustbin_state_old_cnt = 0;
  serial_status.callback_dustbin_cnt_old_cnt = 0;
  serial_status.recv_from_dustbin_state_new_cnt = 0;
  serial_status.callback_dustbin_cnt_new_cnt = 0;
  serial_status.dustbin_state_old_size = (uint16_t)sizeof(clean_to_motion_t);
  serial_status.dustbin_state_new_size =
      (uint16_t)sizeof(clean_to_motion_t_new);
  serial_status.dustbin_state_recv_size = 0;
  serial_status.recv_from_dustbox_state_old_cnt = 0;
  serial_status.callback_dustbox_cnt_old_cnt = 0;
  serial_status.recv_from_dustbox_state_new_cnt = 0;
  serial_status.callback_dustbox_cnt_new_cnt = 0;
  serial_status.dustbox_state_old_size =
      (uint16_t)sizeof(dust_box_to_motion_t_old);
  serial_status.dustbox_state_new_size =
      (uint16_t)sizeof(dust_box_to_motion_t_new);
  serial_status.dustbox_state_recv_size = 0;
  //从文件中读取初始水箱水量,如果没有,则新建,并创建水量为90%
  // std::ifstream in;
  // in.open(config_file_path.c_str(),std::ios::in);
  // if(!in.is_open()){
  // printf("Could not open the config file: %s\n",config_file_path.c_str());
  //     Info("could not open file: "<< config_file_path);
  //    std::string touch_cmd = "touch " + config_file_path;
  //    std::string modify_file = "echo 90.0 > " + config_file_path;
  //     FILE *fpr = NULL;
  //     fpr = popen(touch_cmd.c_str(),"w");
  //     pclose(fpr);
  //    fpr = popen(modify_file.c_str(),"w");
  //    pclose(fpr);
  //    vehicle_water_status.water_percnt_now = 90.0;
  //  }else{
  //     std::string water_per_stored;
  //     getline(in,water_per_stored);
  //     vehicle_water_status.water_percnt_now = atof(water_per_stored.c_str());
  // }
}

void INIT_LOG(rclcpp::Node::SharedPtr &node_) {
  std::string filelog;
  //----------------------log文件名称
  node_->declare_parameter("CTI_RUN_LOG_PATH", "");
  if (node_->has_parameter("CTI_RUN_LOG_PATH")) {
    filelog = node_->get_parameter("CTI_RUN_LOG_PATH").as_string();
    filelog += "/control.log";
  } else {
    node_->declare_parameter("filenamelog", "/home/neousys/log/control.log");
    filelog = node_->get_parameter("filenamelog").as_string();
  }
  filelog = std::string("/home/neousys/log/control.log");
  std::cout << "filenamelog:" << filelog << std::endl;

  initLog(filelog);
}

int INIT_COMMUNICTION(rclcpp::Node::SharedPtr &node_) {
  //获取通信类型
  if (get_port_type(port_type_name) < 0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "port_type is Unavailable: %s， the available port is "
                 "[serial] or [net]",
                 port_type_name.c_str());
    return -1;
  }
  //串口通信sdk
  if (port_type == PORT_SERIAL) {
    RCLCPP_INFO(node_->get_logger(),
                "the port_type  PORT_SERIAL was selected!");
    strcpy((char *)SERIAL_DEVICE_PORT, serial_port.c_str());
    module_serial_init();
    if (pthread_create(&th_serial_recv, NULL, serial_recv_thread, NULL) != 0) {
      RCLCPP_INFO(node_->get_logger(), "serial recv thread create error!");
    }
  }
  // udp通信sdk
  if (port_type == PORT_UDP) {
    RCLCPP_INFO(node_->get_logger(), "the port_type  PORT_UDP was selected!");
    strcpy((char *)UDP_IP, udp_ip.c_str());
    UDP_PORT = udp_port;
    strcpy((char *)UDP_IP_DEST, udp_ip_dest.c_str());
    UDP_PORT_DEST = udp_port_dest;
    module_udp_init();
    if (pthread_create(&th_serial_recv, NULL, udp_recv_thread, NULL) != 0) {
      RCLCPP_INFO(node_->get_logger(), "serial recv thread create error!");
    }
  }
  //通信通用设置
  process_nomal_cmd_cb = process_nomal_cmd_ex;
  process_update_cmd_cb = process_update_cmd_ex;
}

//************************************** 主函数
int main(int argc, char **argv) {
  //----------------------初始化
  rclcpp::init(argc, argv);
  auto node_ = rclcpp::Node::make_shared("cti_fpga_serial_node");
  init_signal();

  // INIT_ROS_SET(node_);
  /////////////////////////////
  std::string cmd_answer_topic;
  std::string version_topic;
  double timer2_duration;
  int pub_hz;
  bool broadcast_tf;
  int send_to_dustbox_fanspeed;
  int send_to_damboard_control;
  //----------------------获取参数
  //通讯端口类型及参数  "serial":串口通讯 "udp":网口udp通讯
  node_->declare_parameter("port_type", std::string("serial"));
  port_type_name = node_->get_parameter("port_type").as_string();
  //串口通讯参数
  node_->declare_parameter("port_name", std::string("/dev/cti_fpga"));
  serial_port = node_->get_parameter("port_name").as_string();
  //网口通讯参数
  node_->declare_parameter("udp_ip", std::string("192.168.1.102"));
  udp_ip = node_->get_parameter("udp_ip").as_string();
  node_->declare_parameter("udp_port", 8888);
  udp_port = node_->get_parameter("udp_port").as_int();
  node_->declare_parameter("udp_ip_dest", std::string("192.168.1.222"));
  udp_ip_dest = node_->get_parameter("udp_ip_dest").as_string();
  node_->declare_parameter("udp_port_dest", 8887);
  udp_port_dest = node_->get_parameter("udp_port_dest").as_int();

  node_->declare_parameter("pub_hz", 50);
  pub_hz = node_->get_parameter("pub_hz").as_int();

  node_->declare_parameter("broadcast_tf", false);
  broadcast_tf = node_->get_parameter("broadcast_tf").as_bool();

  node_->declare_parameter("timer2_duration", 1.0);
  timer2_duration = node_->get_parameter("timer2_duration").as_double();

  node_->declare_parameter(
      "version_topic", std::string("/cti/fpga_serial/operationControlVersion"));
  version_topic = node_->get_parameter("version_topic").as_string();

  node_->declare_parameter("cmd_answer_topic",
                           "/cti/fpga_serial/cmd_answer_cnt");
  cmd_answer_topic = node_->get_parameter("cmd_answer_topic").as_string();

  node_->declare_parameter("car_wheel_base", 0.8);
  car_wheel_base = node_->get_parameter("car_wheel_base").as_double();

  node_->declare_parameter("cmd_answer_timeout", 0.01);
  cmd_answer_timeout = node_->get_parameter("cmd_answer_timeout").as_double();

  node_->declare_parameter("max_control_board_version_head", 0);
  max_control_board_version_head_ =
      node_->get_parameter("max_control_board_version_head").as_int();
  node_->declare_parameter("min_control_board_version_head", 0);
  min_control_board_version_head_ =
      node_->get_parameter("min_control_board_version_head").as_int();
  node_->declare_parameter("max_control_board_version_mid", 0);
  max_control_board_version_mid_ =
      node_->get_parameter("max_control_board_version_mid").as_int();
  node_->declare_parameter("min_control_board_version_mid", 0);
  min_control_board_version_mid_ =
      node_->get_parameter("min_control_board_version_mid").as_int();
  node_->declare_parameter("max_control_board_version_end", 0);
  max_control_board_version_end_ =
      node_->get_parameter("max_control_board_version_end").as_int();
  node_->declare_parameter("min_control_board_version_end", 0);
  min_control_board_version_end_ =
      node_->get_parameter("min_control_board_version_end").as_int();
  node_->declare_parameter("lift_test_switch", false);
  lift_test_switch_ = node_->get_parameter("lift_test_switch").as_bool();
  node_->declare_parameter("default_sweeper_id", 0);
  default_sweeper_id = node_->get_parameter("default_sweeper_id").as_int();
  node_->declare_parameter("default_dustbox_fanspeed", 80);
  send_to_dustbox_fanspeed =
      node_->get_parameter("default_dustbox_fanspeed").as_int();
  send_to_dust_box_cmd.fan_speed = send_to_dustbox_fanspeed;
  node_->declare_parameter("default_damboard_control", 0);
  send_to_damboard_control =
      node_->get_parameter("default_damboard_control").as_int();
  send_to_clean_cmd.dam_board = send_to_damboard_control;
  node_->declare_parameter("robot_type", 0);
  robot_type = node_->get_parameter("robot_type").as_int();

  node_->declare_parameter("CTI_RUN_VER", "");
  cti_run_ver = node_->get_parameter("CTI_RUN_VER").as_string();

  node_->declare_parameter("default_side_brush_transform", 0);
  default_side_brush_transform =
      node_->get_parameter("default_side_brush_transform").as_int();
  node_->declare_parameter("default_side_brush_speed", 0);
  default_side_brush_speed =
      node_->get_parameter("default_side_brush_speed").as_int();
  node_->declare_parameter("LIN_ult_installed", 0);
  LIN_ult_installed = node_->get_parameter("LIN_ult_installed").as_int();
  // printf("LIN_ult_installed: %d\n",LIN_ult_installed);

  node_->declare_parameter("config_file_path", "/home/neousys");
  config_file_path = node_->get_parameter("config_file_path").as_string();
  config_file_path = config_file_path + "/config.yaml";
  // printf("config_file_path: %s\n",config_file_path.c_str());
  node_->declare_parameter("water_out_per_sec", 0.0139);
  vehicle_water_status.out_per_sec =
      node_->get_parameter("water_out_per_sec").as_double();
  node_->declare_parameter("water_in_per_sec", 0.1);
  vehicle_water_status.in_per_sec =
      node_->get_parameter("water_in_per_sec").as_double();
  node_->declare_parameter("localization_limit", true);
  localization_limit = node_->get_parameter("localization_limit").as_bool();
  // printf("localization_limit: %s\n",localization_limit?"true":"false");
  node_->declare_parameter("obstatle_disobs_limit", true);
  obstatle_disobs_limit =
      node_->get_parameter("obstatle_disobs_limit").as_bool();
  node_->declare_parameter("chassis_chat_timeout_secs", 10.0);
  chassis_chat_timeout_secs =
      node_->get_parameter("chassis_chat_timeout_secs").as_double();
  node_->declare_parameter("box_chat_timeout_secs", 10.0);
  box_chat_timeout_secs =
      node_->get_parameter("box_chat_timeout_secs").as_double();
  node_->declare_parameter("msg_max_range", 10.0);
  msg_max_range = node_->get_parameter("msg_max_range").as_double();
  // printf("obstatle_disobs_limit: %s\n",obstatle_disobs_limit?"true":"false");

  //----------------------ros话题订阅定义
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_soft_stop =
      node_->create_subscription<std_msgs::msg::Int8>(
          "/cti/rblite/emergency_stop_recovery", 10, soft_stop_callback);
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      sub_cmd_vel =
          node_->create_subscription<geometry_msgs::msg::TwistStamped>(
              "/cmd_vel", 10, cmd_vel_callback);

  rclcpp::Subscription<cti_msgs::msg::TargetPose>::SharedPtr
      sub_contraposition =
          node_->create_subscription<cti_msgs::msg::TargetPose>(
              "/box_pose", 10, boxContraposition_Callback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_sub =
      node_->create_subscription<std_msgs::msg::Int32>("cti/fpga_serial/boxcmd",
                                                       10, boxUpDown_Callback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr boxlock_sub =
      node_->create_subscription<std_msgs::msg::Int32>(
          "cti/fpga_serial/boxlockcmd", 10, boxLock_Callback);

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr poweroff_sub =
      node_->create_subscription<std_msgs::msg::Int16MultiArray>(
          "cti/fpga_serial/poweroffcmd", 10, powerOff_Callback);

  rclcpp::Subscription<cti_fpga_serial_msgs::msg::UpdateInfo>::SharedPtr
      stm32_sub =
          node_->create_subscription<cti_fpga_serial_msgs::msg::UpdateInfo>(
              "cti/fpga_serial/stmupdate", 10, stmUpdate_Callback);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light_sub =
      node_->create_subscription<std_msgs::msg::Int32>(
          "cti/fpga_serial/light_type", 2, lightType_Callback);

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr light_v3_0_sub =
      node_->create_subscription<cti_msgs::msg::DataArray>(
          "cti/fpga_serial/light_type_v3_0", 20, lightType_v3_0_Callback);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/lidar_pose", 10, position_Callback);

  rclcpp::Subscription<cti_msgs::msg::AutoTransmission>::SharedPtr disObs_sub =
      node_->create_subscription<cti_msgs::msg::AutoTransmission>(
          "/cti/obstacle/disObs", 10, disObs_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr checkversion_sub =
      node_->create_subscription<std_msgs::msg::UInt8>(
          "/cti/fpga_serial/checkversion", 1, checkversion_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr formatsdcard_sub =
      node_->create_subscription<std_msgs::msg::UInt8>(
          "/cti/fpga_serial/formatsdcard", 1, formatsdcard_Callback);

  rclcpp::Subscription<cti_msgs::msg::RobotLocalizerState>::SharedPtr
      localizerState_sub =
          node_->create_subscription<cti_msgs::msg::RobotLocalizerState>(
              "/cti_localizer_state", 1, localizerState_Callback);

  rclcpp::Subscription<cti_msgs::msg::DustbinControl>::SharedPtr
      dustbin_control_sub;
  rclcpp::Subscription<cti_msgs::msg::DustbinControlNew>::SharedPtr
      dustbin_control_sub_new;
  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dustbin_control_info_sub;

  // poweroff_cmd_type(const std_msgs::msg::UInt32 msg)
  if (robot_type == 1) {
    dustbin_control_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control", 1, cleanControl_Callback);
    dustbin_control_sub_new =
        node_->create_subscription<cti_msgs::msg::DustbinControlNew>(
            "/cti/chassis_serial/dustbin_control_new", 1,
            cleanControlNew_Callback);
    dustbin_control_info_sub =
        node_->create_subscription<cti_msgs::msg::DataArray>(
            "/cti/chassis_serial/sanitation_vehicle_control", 1,
            cleanControlInfo_Callback);
  } else if (robot_type == 0) {
    dustbin_control_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control", 1, dustbinControl_Callback);
    dustbin_control_info_sub =
        node_->create_subscription<cti_msgs::msg::DataArray>(
            "/cti/chassis_serial/sanitation_vehicle_control", 1,
            cleanControlInfo_Callback);
  } else {
    dustbin_control_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control", 1, dustbinControl_Callback);
    dustbin_control_info_sub =
        node_->create_subscription<cti_msgs::msg::DataArray>(
            "/cti/chassis_serial/sanitation_vehicle_control", 1,
            cleanControlInfo_Callback);
  }
  rclcpp::Subscription<cti_msgs::msg::DustbinControl>::SharedPtr
      dustbin_control_resend_sub;
  if (robot_type == 1) {
    dustbin_control_resend_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control_include_resend", 1,
            cleanControlResend_Callback);
  } else if (robot_type == 0) {
    dustbin_control_resend_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control_include_resend", 1,
            dustbinControlResend_Callback);
  } else {
    dustbin_control_resend_sub =
        node_->create_subscription<cti_msgs::msg::DustbinControl>(
            "/cti/chassis_serial/dustbin_control_include_resend", 1,
            dustbinControlResend_Callback);
  }
  if (lift_test_switch_) {
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lift_test_sub =
        node_->create_subscription<std_msgs::msg::Int32>(
            "cti/fpga_serial/jacking_test", 1, lifttest_Callback);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_type_sub =
      node_->create_subscription<std_msgs::msg::Int32>("/cti/rblite/boxtype", 1,
                                                       boxtype_Callback);

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sweeper_box_id_sub =
      node_->create_subscription<std_msgs::msg::Int64>(
          "/cti/chassis_serial/set_sweeper_box_id", 1, sweeperBoxID_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      wireless_charge_control_sub =
          node_->create_subscription<std_msgs::msg::UInt8>(
              "/cti/chassis_serial/wireless_charge_control", 1,
              wirelessCharge_Callback);

  rclcpp::Subscription<cti_msgs::msg::DustbinControl>::SharedPtr
      dust_box_control_sub =
          node_->create_subscription<cti_msgs::msg::DustbinControl>(
              "/cti/chassis_serial/dust_box_control", 1,
              dustBoxControl_Callback);

  rclcpp::Subscription<cti_msgs::msg::DustboxControl>::SharedPtr
      dust_box_control_sub_new =
          node_->create_subscription<cti_msgs::msg::DustboxControl>(
              "/cti/chassis_serial/dust_box_control_new", 1,
              dustBoxControlNew_Callback);

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dust_box_control_info_sub =
          node_->create_subscription<cti_msgs::msg::DataArray>(
              "/cti/chassis_serial/sanitation_dustbox_control", 1,
              dustBoxControlInfo_Callback);  //环卫车集尘箱控制
                                             // cti_msgs::msg::DataArray消息类型
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      dust_box_autopush_control_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dust_box_fan_speed_sub;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dustbin_damboard_sub;
  if (robot_type == 1) {
    dust_box_autopush_control_sub =
        node_->create_subscription<std_msgs::msg::UInt8>(
            "/cti/chassis_serial/dust_box_autopush_control", 1,
            dustBoxAutopushControl_Callback);

    dust_box_fan_speed_sub = node_->create_subscription<std_msgs::msg::UInt8>(
        "/cti/chassis_serial/dust_box_fanspeed_control", 1,
        dustBoxFanSpeed_Callback);

    dustbin_damboard_sub = node_->create_subscription<std_msgs::msg::UInt8>(
        "/cti/chassis_serial/dustbin_damboard_control", 1,
        dustbinDamboard_Callback);
  }
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      dustbin_sideBrushTrans_sub =
          node_->create_subscription<std_msgs::msg::UInt8>(
              "/cti/chassis_serial/dustbin_sideBrushTrans_control", 1,
              dustbinSideBrushTrans_Callback);  //测试用的,单独控制边刷伸展

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr smart_trash_sub =
      node_->create_subscription<cti_msgs::msg::DataArray>(
          "/cti/chassis_serial/smart_trash_control", 1, smartTrash_Callback);

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dustbox_5g_check_sub =
          node_->create_subscription<cti_msgs::msg::DataArray>(
              "/cti/chassis_serial/sanitation_dustbox_5g_check", 1,
              dustbox5GCheck_Callback);
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ledshow_sub =
      node_->create_subscription<std_msgs::msg::Int32>("/cti/ledshow/cmd", 1,
                                                       ledshow_Callback);
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ult_cmd_sub_ =
      node_->create_subscription<std_msgs::msg::Int64>(
          "/cti/ultrasonic/ult_cmd", 1, ultCmdCallback);

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr
      power_control_sub =
          node_->create_subscription<std_msgs::msg::UInt32MultiArray>(
              "/cti/fpga_serial/power_control", 1, poweroff_cmd_type_callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr exit_charging_sub =
      node_->create_subscription<std_msgs::msg::UInt8>(
          "/cti/chassis_serial/exit_charging_cmd", 1, exit_charging_Callback);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sprayControl_sub =
      node_->create_subscription<std_msgs::msg::UInt8>(
          "/cti/chassis_serial/spray_control_cmd", 1, sprayControl_Callback);

  //----------------------ros话题发布定义
  // odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/robot_odom",
  // 10);
  odom_pub_4wd =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot_odom_4wd", 10);
  odom_pub_calc =
      node_->create_publisher<nav_msgs::msg::Odometry>("/robot_odom_calc", 10);
  sins_pub =
      node_->create_publisher<cti_msgs::msg::Sins>("/cti/fpga_serial/sins", 10);

  imudata_pub = node_->create_publisher<sensor_msgs::msg::Imu>(
      "/cti/fpga_serial/imu", 10);
  // steer_pub =
  // node_->create_publisher<std_msgs::msg::Float64>("/robot_steer_angle",
  // 10);//2020年4月2日停用
  battery_pub = node_->create_publisher<cti_msgs::msg::BatteryState>(
      "/cti/cti_fpga/battery_state", 10);
  batcell_pub = node_->create_publisher<cti_msgs::msg::BatteryCellsState>(
      "/cti/cti_fpga/batcell_state", 10);
  // info_pub =
  // node_->create_publisher<std_msgs::msg::Float64MultiArray>("/cti/fpga_serial/run_info",
  // 10); //2020年4月2日停用
  ctlinfo_pub = node_->create_publisher<cti_msgs::msg::VehicleCtlRunInfo>(
      "/cti/fpga_serial/ctlrun_info", 10);

  state_pub = node_->create_publisher<std_msgs::msg::String>(
      "/cti/fpga_serial/error", 10);
  stm32_pub = node_->create_publisher<cti_fpga_serial_msgs::msg::UpdateInfo>(
      "/cti/fpga_serial/stminfo", 10);

  cmd_answer_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::CmdAnswer>(
          cmd_answer_topic, 10);

  boxrfid_pub_all = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/fpga_serial/rfid_all", 10);

  boxrfid_pub_single = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/fpga_serial/rfid_single", 10);

  ranges_pub = node_->create_publisher<cti_msgs::msg::Range>(
      "/cti/fpga_serial/multiple_ultrasonic", 1);

  for (int i = 0; i < max_type_ult; i++) {
    range_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/fpga_serial/" + ult_name[i], 1);
  }
  for (int i = 0; i < sweeper_max_type_ult; i++) {
    sweeper_range_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + sweeper_ult_name[i], 1);
  }
  for (int i = 0; i < SWEEPER_ULT_MODULE_NUM; i++) {
    for (int j = 0; j < new_sweeper_max_type_ult; j++) {
      new_sweeper_range_pub[i][j] =
          node_->create_publisher<sensor_msgs::msg::Range>(
              "/cti/chassis_serial/" + new_sweeper_ult_name[i][j], 1);
    }
  }
  for (int i = 0; i < ULT_3_0_TYPE_NUM; i++) {
    alt_3_0_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + ult_3_0_name[i], 1);
  }

  //集尘箱超声波发布
  for (int i = 0; i < dustbox_max_type_ult; i++) {
    dustbox_range_pub[i] = node_->create_publisher<sensor_msgs::msg::Range>(
        "/cti/chassis_serial/" + dustbox_ult_name[i], 1);
  }
  //车身lin通信超声波原生数据发布
  raw_lin_range_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::RangeRawData>(
          "/cti/chassis_serial/range_raw_data", 25);
  firmvion_pub = node_->create_publisher<cti_msgs::msg::RobotVersionDisplay>(
      "/cti/fpga_serial/operationControlVersion",
      rclcpp::QoS{1}.transient_local());
  serial_status_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::SerialStatus>(
          "/cti/fpga_serial/serial_status", 1);
  box_laser_pub = node_->create_publisher<cti_msgs::msg::Rtcm>(
      "/cti/fpga_serial/box_laser", 10);
  chassis_error_pub = node_->create_publisher<std_msgs::msg::UInt32MultiArray>(
      "/cti/chassis_serial/chassis_error", 10);
  navigation_log_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::NavigationLog>(
          "/cti/fpga_serial/navigation_log", 10);
  baro_status_pub = node_->create_publisher<std_msgs::msg::UInt16>(
      "/cti/chassis_serial/baro_status", 10);
  formatsdcard_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/fpga_serial/formatsdcard_result", 1);
  gps_pub = node_->create_publisher<cti_msgs::msg::GnssRTK>(
      "/cti/chassis_serial/gnss", 10);
  compass_pub = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/cti/chassis_serial/compass", 10);
  node_status_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/node_active", 25);
  dustbin_state_pub = node_->create_publisher<cti_msgs::msg::DustbinState>(
      "/cti/chassis_serial/dustbin_state",
      25);  //新的环卫车清扫状态发布,将以前拆开发布的数据放在这里发布
  //新的环卫车清扫状态发布,将以前拆开发布的数据放在这里发布
  dustbin_state_pub_new =
      node_->create_publisher<cti_msgs::msg::DustbinStateNew>(
          "/cti/chassis_serial/dustbin_state_new", 25);
  firmware_version_status_pub = node_->create_publisher<std_msgs::msg::Int8>(
      "/cti/chassis_serial/control_board_version_status", 1);
  firmware_version_check_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::FirmWareInfo>(
          "/cti/chassis_serial/FW_check_report", 1);
  recv_chassis_info_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::VehicleState>(
          "/cti/chassis_serial/chassis_info", 10);
  set_dustbin_id_state_pub = node_->create_publisher<cti_msgs::msg::TabState>(
      "/cti/chassis_serial/dustbin_set_id_state", 1);
  recv_dustbin_id_state_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::DustbinidState>(
          "/cti/chassis_serial/dustbin_id_state", 1);
  dust_box_state_pub = node_->create_publisher<cti_msgs::msg::DustbinState>(
      "/cti/chassis_serial/dust_box_state", 25);
  dust_box_state_pub_new = node_->create_publisher<cti_msgs::msg::DustboxState>(
      "/cti/chassis_serial/dust_box_state_new", 25);
  dust_box_state_info_pub = node_->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/sanitation_dustbox_state",
      25);  // cti_msgs/DataArray类型//环卫车,吸尘箱状态发布
  dust_box_autopush_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dust_box_autopush_state", 25);
  dust_box_fanspeed_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dust_box_fanspeed_state", 25);
  dustbin_damboard_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/dustbin_damboard_state", 25);
  dustbox_wireless_charge_state_pub =
      node_->create_publisher<cti_msgs::msg::BatteryState>(
          "/cti/chassis_serial/dustbox_wireless_charge_state", 2);
  dustbox_bottom_range_pub = node_->create_publisher<sensor_msgs::msg::Range>(
      "/cti/chassis_serial/dustbox_bottom", 2);
  boxlock_state_pub = node_->create_publisher<std_msgs::msg::UInt8>(
      "/cti/chassis_serial/boxlock_state", 25);
  dust_box_state_pub_json = node_->create_publisher<std_msgs::msg::String>(
      "/cti/chassis_serial/dust_box_state_json", 25);
  rain_sensor_pub = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/chassis_serial/rain_sensor", 25);
  smart_trash_state_pub = node_->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/smart_trash_state", 25);
  dust_vehicle_state_info_pub =
      node_->create_publisher<cti_msgs::msg::DataArray>(
          "/cti/chassis_serial/sanitation_vehicle_state",
          25);  // cti_msgs/DataArray类型//环卫车,车辆清扫状态发布
  dustbox_5g_state_pub = node_->create_publisher<cti_msgs::msg::DataArray>(
      "/cti/chassis_serial/sanitation_dustbox_5g_state",
      25);  //吸尘箱5g状态发布
  dustbox_batterycell_pub =
      node_->create_publisher<cti_msgs::msg::BatteryCellsState>(
          "/cti/chassis_serial/dustbox_batcell_state", 25);  //吸尘箱总电量发布
  chat_statue_pub = node_->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/chassis_serial/communication_state", 1);  //通信状态发布
  lin_ult_data_pub = node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "/cti/chassis_serial/lin_ult_data", 1);  // lin通信超声波源数据发布
  dustbox_rear_range_pub =
      node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
          "/cti/chassis_serial/sanitation_dustbox_ult",
          2);  //吸尘箱超声波原始数据发送
  lin_ult_data_v3_0_pub = node_->create_publisher<
      std_msgs::msg::UInt16MultiArray>(
      "/cti/chassis_serial/lin_ult_data_v3_0",
      1);  //环卫车3.0lin通信超声波源数据发布 =
           // node_->create_publisher<std_msgs::msg::UInt16MultiArray>("/cti/chassis_serial/lin_ult_data",1);//lin通信超声波源数据发布

  new_lin_ult_data_v3_0_pub =
      node_->create_publisher<cti_fpga_serial_msgs::msg::UltV30Datas>(
          "/cti/chassis_serial/new_lin_ult_data_v3_0",
          1);  // new环卫车3.0lin通信超声波源数据发布 =

  //----------------------ros定时器定义
  rclcpp::TimerBase::SharedPtr timer =
      node_->create_wall_timer(1s, &timerCallback);
  rclcpp::TimerBase::SharedPtr timer2 = node_->create_wall_timer(
      std::chrono::duration<double>(timer2_duration), &timer2Callback);
  rclcpp::TimerBase::SharedPtr timer3 = node_->create_wall_timer(
      std::chrono::duration<double>(cmd_answer_timeout), &timer3Callback);
  rclcpp::TimerBase::SharedPtr timer4 = node_->create_wall_timer(
      std::chrono::duration<double>(RECVTIMEOUT_DUR), &timer4Callback);
  //关闭timer5以后，接到底盘发布的错误消息会发布出去，打开timer5,平时会发底盘正常的消息
  // rclcpp::TimerBase::SharedPtr timer5 =
  // node_->create_wall_timer(0.25s,&timer5Callback);
  rclcpp::TimerBase::SharedPtr timer6 =
      node_->create_wall_timer(0.04s, &timer6Callback);
  rclcpp::TimerBase::SharedPtr timer7 =
      node_->create_wall_timer(1s, &timer7Callback);  //清扫箱箱号设置和获取
  rclcpp::TimerBase::SharedPtr timer_chat_timeout =
      node_->create_wall_timer(1s, &timer_chat_timeout_Callbak);  //通信超时检测
  rclcpp::TimerBase::SharedPtr timer_set_ult_mode = node_->create_wall_timer(
      0.5s, &timer_set_ult_mode_Callback);  //设置超声波模式
  rclcpp::TimerBase::SharedPtr timer_process_work_mode =
      node_->create_wall_timer(1s, &timer_set_process_work_mode_Callback);  //
  ////////////////////////////
  // INIT_LOG(node_);
  ///////////////////////////
  std::string filelog;
  //----------------------log文件名称
  node_->declare_parameter("CTI_RUN_LOG_PATH", "");
  if (node_->has_parameter("CTI_RUN_LOG_PATH")) {
    filelog = node_->get_parameter("CTI_RUN_LOG_PATH").as_string();
    filelog += "/control.log";
  } else {
    node_->declare_parameter("filenamelog", "/home/neousys/log/control.log");
    filelog = node_->get_parameter("filenamelog").as_string();
  }
  filelog = std::string("/home/neousys/log/control.log");
  std::cout << "filenamelog:" << filelog << std::endl;

  initLog(filelog);
  ///////////////////////////
  // INIT_DATAS();
  ///////////////////////////
  //车身lin超声波默认模式赋值
  //车身前右，后右，左前下，右前下，其他设置为又收又发，车身有效位设置对应值,无效位全部为1
  vehicle_defalt_linult_mode.bits.lin_front_left = 2;
  vehicle_defalt_linult_mode.bits.lin_front_right = 1;
  vehicle_defalt_linult_mode.bits.lin_back_center = 2;
  vehicle_defalt_linult_mode.bits.lin_rear_left = 2;
  vehicle_defalt_linult_mode.bits.lin_rear_right = 1;
  vehicle_defalt_linult_mode.bits.unused1 = 0xffff;
  vehicle_defalt_linult_mode.bits.lin_left_front_up = 2;
  vehicle_defalt_linult_mode.bits.lin_left_front_down = 1;
  vehicle_defalt_linult_mode.bits.lin_left_center = 2;
  vehicle_defalt_linult_mode.bits.lin_right_center = 2;
  vehicle_defalt_linult_mode.bits.lin_right_front_down = 1;
  vehicle_defalt_linult_mode.bits.lin_right_front_up = 2;
  vehicle_defalt_linult_mode.bits.unused2 = 0xffff;
  vehicle_defalt_linult_mode.bits.lin_dustbox_rear_right = 0xff;
  vehicle_defalt_linult_mode.bits.lin_dustbox_rear_left = 0xff;
  vehicle_defalt_linult_mode.bits.lin_dustbox_bottom = 0xff;
  vehicle_defalt_linult_mode.bits.unused3 = 0xffff;
  vehicle_defalt_linult_mode.bits.unused4 = 0xffff;
  //车身lin超声波 全部关闭模式赋值，车身有效位全部为0,无效位全部为1
  vehicle_all_stop_work_mode.data = 0;
  vehicle_all_stop_work_mode.bits.unused1 = 0xffff;
  vehicle_all_stop_work_mode.bits.unused2 = 0xffff;
  vehicle_all_stop_work_mode.bits.unused3 = 0xffff;
  vehicle_all_stop_work_mode.bits.unused4 = 0xffff;
  vehicle_all_stop_work_mode.bits.lin_dustbox_rear_right = 0xff;
  vehicle_all_stop_work_mode.bits.lin_dustbox_rear_left = 0xff;
  vehicle_all_stop_work_mode.bits.lin_dustbox_bottom = 0xff;
  //吸尘箱lin超声波默认模式赋值 箱子后右 设置为只收不发，
  //其他设置为又收又发，箱子有效位设置为对应值，其余全为1
  dustbox_defalt_linult_mode.data = 0xffffffffffffffff;
  dustbox_defalt_linult_mode.bits.lin_dustbox_rear_right = 1;
  dustbox_defalt_linult_mode.bits.lin_dustbox_rear_left = 2;
  dustbox_defalt_linult_mode.bits.lin_dustbox_bottom = 2;
  //吸尘箱lin超声波 全部关闭模式赋值，吸尘箱有效位全部为0,无效位全部为1
  dustbox_all_stop_work_mode.data = 0xffffffffffffffff;
  dustbox_all_stop_work_mode.bits.lin_dustbox_rear_right = 0x00;
  dustbox_all_stop_work_mode.bits.lin_dustbox_rear_left = 0x00;
  dustbox_all_stop_work_mode.bits.lin_dustbox_bottom = 0x00;

  // printf("vehicle_defalt_linult_mode.data:
  // 0x%016lx\n",vehicle_defalt_linult_mode.data);
  // printf("vehicle_all_stop_work_mode.data:
  // 0x%016lx\n",vehicle_all_stop_work_mode.data);
  // printf("dustbox_defalt_linult_mode.data:
  // 0x%016lx\n",dustbox_defalt_linult_mode.data);
  // printf("dustbox_all_stop_work_mode.data:
  // 0x%016lx\n",dustbox_all_stop_work_mode.data);

  //----------------------设串口通信sdk
  // std::cout<<"open port started! "<<std::endl;
  //   strcpy((char *)SERIAL_DEVICE_PORT, serial_port.c_str());
  // strcpy((char *)SERIAL_DEVICE_PORT, "/dev/serialLock0");
  // strcpy((char *)SERIAL_DEVICE_PORT, "/dev/ttyUSB0");
  // process_nomal_cmd_cb = process_nomal_cmd_ex;
  // process_update_cmd_cb = process_update_cmd_ex;
  // module_serial_init();
  // auto period = std::chrono::milliseconds(100);
  // rclcpp::WallRate loop_rate(10ms);
  //----------------------------
  // initLog(filelog);
  //--
  // databuf.clear();
  // std::cout<<"open port finished! "<<std::endl;
  //---------------------创建接收数据的线程
  // int p_ret = 0;
  // p_ret = pthread_create(&th_serial_recv, NULL, serial_recv_thread, NULL);
  // if(p_ret != 0)
  //{
  // printf("serial thread creat error!\n");
  //}
  //----------------------给每一个RFID容器赋初值，防止boxload_callback判断错误
  cti_msgs::msg::TabState tabstate;
  oldtabstate_rfid1.push_back(tabstate);
  oldtabstate_rfid2.push_back(tabstate);
  oldtabstate_rfid3.push_back(tabstate);
  oldtabstate_rfid4.push_back(tabstate);
  //-----------------------serial_status赋初值
  serial_status.recv_from_dustbin_state_old_cnt = 0;
  serial_status.callback_dustbin_cnt_old_cnt = 0;
  serial_status.recv_from_dustbin_state_new_cnt = 0;
  serial_status.callback_dustbin_cnt_new_cnt = 0;
  serial_status.dustbin_state_old_size = (uint16_t)sizeof(clean_to_motion_t);
  serial_status.dustbin_state_new_size =
      (uint16_t)sizeof(clean_to_motion_t_new);
  serial_status.dustbin_state_recv_size = 0;
  serial_status.recv_from_dustbox_state_old_cnt = 0;
  serial_status.callback_dustbox_cnt_old_cnt = 0;
  serial_status.recv_from_dustbox_state_new_cnt = 0;
  serial_status.callback_dustbox_cnt_new_cnt = 0;
  serial_status.dustbox_state_old_size =
      (uint16_t)sizeof(dust_box_to_motion_t_old);
  serial_status.dustbox_state_new_size =
      (uint16_t)sizeof(dust_box_to_motion_t_new);
  serial_status.dustbox_state_recv_size = 0;
  //从文件中读取初始水箱水量,如果没有,则新建,并创建水量为90%
  // std::ifstream in;
  // in.open(config_file_path.c_str(),std::ios::in);
  // if(!in.is_open()){
  // printf("Could not open the config file: %s\n",config_file_path.c_str());
  //     Info("could not open file: "<< config_file_path);
  //    std::string touch_cmd = "touch " + config_file_path;
  //    std::string modify_file = "echo 90.0 > " + config_file_path;
  //     FILE *fpr = NULL;
  //     fpr = popen(touch_cmd.c_str(),"w");
  //     pclose(fpr);
  //    fpr = popen(modify_file.c_str(),"w");
  //    pclose(fpr);
  //    vehicle_water_status.water_percnt_now = 90.0;
  //  }else{
  //     std::string water_per_stored;
  //     getline(in,water_per_stored);
  //     vehicle_water_status.water_percnt_now = atof(water_per_stored.c_str());
  // }
  //////////////////////////
  // INIT_COMMUNICTION(node_);
  ////////////////////////////
  //获取通信类型
  if (get_port_type(port_type_name) < 0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "port_type is Unavailable: %s， the available port is "
                 "[serial] or [net]",
                 port_type_name.c_str());
    return -1;
  }
  //串口通信sdk
  if (port_type == PORT_SERIAL) {
    RCLCPP_INFO(node_->get_logger(),
                "the port_type  PORT_SERIAL was selected!");
    strcpy((char *)SERIAL_DEVICE_PORT, serial_port.c_str());
    module_serial_init();
    if (pthread_create(&th_serial_recv, NULL, serial_recv_thread, NULL) != 0) {
      RCLCPP_INFO(node_->get_logger(), "serial recv thread create error!");
    }
  }

  // udp通信sdk
  if (port_type == PORT_UDP) {
    RCLCPP_INFO(node_->get_logger(), "the port_type  PORT_UDP was selected!");
    strcpy((char *)UDP_IP, udp_ip.c_str());
    UDP_PORT = udp_port;
    strcpy((char *)UDP_IP_DEST, udp_ip_dest.c_str());
    UDP_PORT_DEST = udp_port_dest;
    module_udp_init();
    if (pthread_create(&th_serial_recv, NULL, udp_recv_thread, NULL) != 0) {
      RCLCPP_INFO(node_->get_logger(), "serial recv thread create error!");
    }
  }
  //通信通用设置
  process_nomal_cmd_cb = process_nomal_cmd_ex;
  process_update_cmd_cb = process_update_cmd_ex;
  ///////////////////////////

  databuf.clear();
  auto period = std::chrono::milliseconds(100);
  rclcpp::WallRate loop_rate(10ms);
  while (rclcpp::ok()) {
    if (port_type == PORT_SERIAL) {
      module_serial_process_thread();
      sendSerialData();
    }
    if (port_type == PORT_UDP) {
      module_udp_process_thread();
      sendUdpData();
    }
    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Error("cti_fpga_serial_node quit!");
  pthread_cancel(th_serial_recv);
  pthread_join(th_serial_recv, NULL);
  return 0;
}
