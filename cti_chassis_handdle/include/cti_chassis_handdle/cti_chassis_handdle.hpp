#ifndef CTI_CHASSIS_ACCEPT_H
#define CTI_CHASSIS_ACCEPT_H
#include "cti_chassis_handdle/cti_chassis_communicate_interface.hpp"
#include "cti_chassis_handdle/cti_chassis_communicate_realization.hpp"
#include "cti_chassis_handdle/user_cmd.hpp"

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
#include <sstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include "time.h"
#include <sys/time.h>
#include "jsoncpp/json/json.h"
#include <cstring>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include "cti_chassis_msgs/msg/update_info.hpp"
#include "cti_chassis_msgs/msg/cmd_answer.hpp"
#include "cti_chassis_msgs/msg/firm_ware_info.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <cti_msgs/msg/auto_transmission.hpp>
#include <cti_chassis_msgs/msg/navigation_log.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <cti_msgs/msg/gnss_rtk.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cti_msgs/msg/robot_localizer_state.hpp>
#include <cti_msgs/msg/dustbin_control.hpp>
#include <cti_msgs/msg/dustbin_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cti_chassis_msgs/msg/vehicle_state.hpp>
#include <std_msgs/msg/int64.hpp>
#include "cti_chassis_msgs/msg/dustbinid_state.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <cti_msgs/msg/dustbin_control_new.hpp>
#include <cti_msgs/msg/dustbin_state_new.hpp>
#include <cti_msgs/msg/dustbox_control.hpp>
#include <cti_msgs/msg/dustbox_state.hpp>
#include <cti_msgs/msg/data.hpp>
#include <cti_msgs/msg/data_array.hpp>
#include <cti_chassis_msgs/msg/range_raw_data.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <cti_chassis_msgs/msg/ult_v30_datas.hpp>

#define DEBUG_PRINT 0
#define cmax(x, y) ((x > y) ? (x) : (y))
#define cmin(x, y) ((x < y) ? (x) : (y))
#define PI 3.14159265358979323846
#define GRAV 9.7803
#define SWEEPER_ULT_FILTER_NUM 2

//??????????????????
#define PORT_SERIAL 1
#define PORT_UDP 2

class ChassisHanddle : public rclcpp::Node {
 public:
  explicit ChassisHanddle(std::shared_ptr<ChassisCmncBase> chassisCmncBase_)
      : chassisCmncBase(chassisCmncBase_), Node("cti_chassis_handle_node") {
    this->init();
  }

 private:
  void init();
  void initParam();
  void initSub();
  void initPub();
  void initTimer();
  void initLog(const std::string name);
  void initCtiLog();
  void initChassisCommunication();
  void spinNode();

  void getChassisData();
  void sendChassisData();

  int process_nomal_cmd(unsigned char cmd, unsigned int length,
                        unsigned char *data);

  double calcYawFromQuaternion(const tf2::Quaternion &q);

  std::string hex2string(uint8_t *data, unsigned int Len);

  //----------------------????????????---------------------------------------------//
  void poweroff_cmd_type_callback(
      const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

  void soft_stop_callback(const std_msgs::msg::Int8::SharedPtr msg);

  void cmd_vel_callback(
      const geometry_msgs::msg::TwistStamped::SharedPtr twistStamped);

  void boxUpDown_Callback(const std_msgs::msg::Int32::SharedPtr msg);

  void stmUpdate_Callback(
      const cti_chassis_msgs::msg::UpdateInfo::SharedPtr msg);

  void lightType_v3_0_Callback(const cti_msgs::msg::DataArray::SharedPtr msg);

  void position_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void checkversion_Callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

  void formatsdcard_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void cleanControlNew_Callback(
      const cti_msgs::msg::DustbinControlNew::SharedPtr msg);

  void cleanControlInfo_Callback(const cti_msgs::msg::DataArray::SharedPtr msg);

  void dustBoxControlInfo_Callback(
      const cti_msgs::msg::DataArray::SharedPtr msg);

  void sprayControl_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void wirelessCharge_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void exit_charging_Callback(const std_msgs::msg::UInt8::SharedPtr msg);

  void dustBoxAutopushControl_Callback(
      const std_msgs::msg::UInt8::SharedPtr msg);

  void localizerState_Callback(
      const cti_msgs::msg::RobotLocalizerState::SharedPtr msg);

  void errorcode_clean_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

  void battery_parameter_setting_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);

  void common_error_clear_callback(const std_msgs::msg::UInt64MultiArray::SharedPtr msg);  // ???????????????
  
  //-------------------------?????????-----------------------------------//
  void timer5Callback();

  void timerCallback();

  void timer2Callback();

  void mainTimerCallback();

  void timer_set_process_work_mode_Callback();

  double get_duration(double in_time);

  void timer_chat_timeout_Callbak();

  int process_update_cmd_ex(unsigned char cmd, unsigned char *data,
                            unsigned int length);

  void pub_rain_sensor_new(rain_sensor_t_new *data);

  void pub_dust_box_state_new(dust_box_to_motion_t_new *data);

  void pub_clean_state_new(clean_to_motion_t_new *data);

  void check_rough_res(time_sync_rough_msg_type *data);

  void send_fine_sync_cmd();

  void send_delay_sync_cmd();

  void pub_gps(recv_gps_data_type *data);

  void pub_rtk(msg_rtk_gps_data_1_0 *data);

  void pub_formatsdcard(send_format_sd_card_cmd_type *data);

  void pub_navigationlog(recv_navigation_log_status_type *data);

  void pub_odom(const recv_from_control_status_type *data);

  void pub_alt_3_0_(const msg_upa_pos_data_t *data);

  void pub_battery_3_0(
      const recv_battery_4_to_1_active_report_status_type_3_0_ *data);

  bool less_equa_compare(std::vector<int> vec1, std::vector<int> vec2);

  void getNumInString(std::string str);

  double gps_data_trans(double data);

  void pub_firmwareversion(recv_from_firmware_version_type *data);

  void compar_id_recv_send(recv_from_cmd_answer_type *data);

  void pub_chassiserror(recv_chassis_error_report_type *data);

  void pub_bat_report_type_3_0(msg_bat_report_type_3_0 *data);

  void pub_msg_battery_parameter_report_3_0(msg_battery_parameter_report_3_0 *data);

  void pub_msg_vehicle_kinematic_log_3_0(msg_vehicle_kinematic_log_3_0 *data);


  //----------------------ros??????????????????---------------------------------------//
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_soft_stop;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr box_sub;

  rclcpp::Subscription<cti_chassis_msgs::msg::UpdateInfo>::SharedPtr stm32_sub;

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr light_v3_0_sub;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr checkversion_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr formatsdcard_sub;

  rclcpp::Subscription<cti_msgs::msg::RobotLocalizerState>::SharedPtr
      localizerState_sub;

  rclcpp::Subscription<cti_msgs::msg::DustbinControlNew>::SharedPtr
      dustbin_control_sub_new;
  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dustbin_control_info_sub;

  rclcpp::Subscription<cti_msgs::msg::DataArray>::SharedPtr
      dust_box_control_info_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      wireless_charge_control_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      dust_box_autopush_control_sub;

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr
      power_control_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr exit_charging_sub;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sprayControl_sub;

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr errorcode_clean_sub;

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr battery_parameter_setting_sub;
  
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr common_error_clear_sub;;  // ???????????????  


  ////////////////////////////////////////////////////////////////////////////

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_4wd;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_calc;
  rclcpp::Publisher<cti_chassis_msgs::msg::UpdateInfo>::SharedPtr stm32_pub;
  rclcpp::Publisher<cti_msgs::msg::BatteryCellsState>::SharedPtr batcell_pub;
  rclcpp::Publisher<cti_msgs::msg::VehicleCtlRunInfo>::SharedPtr ctlinfo_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
  rclcpp::Publisher<cti_msgs::msg::RobotVersionDisplay>::SharedPtr firmvion_pub;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr
      alt_3_0_pub[ULT_3_0_TYPE_NUM];
  rclcpp::Publisher<cti_chassis_msgs::msg::RangeRawData>::SharedPtr
      raw_lin_range_pub;  // lin?????????????????????????????????
  rclcpp::Publisher<cti_chassis_msgs::msg::CmdAnswer>::SharedPtr cmd_answer_pub;
  rclcpp::Publisher<cti_msgs::msg::Sins>::SharedPtr sins_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imudata_pub;
  rclcpp::Publisher<cti_msgs::msg::Rtcm>::SharedPtr
      box_laser_pub;  //????????????????????????
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr
      chassis_error_pub;  //????????????????????????
  rclcpp::Publisher<cti_chassis_msgs::msg::NavigationLog>::SharedPtr
      navigation_log_pub;  //????????????????????????
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr
      baro_status_pub;  //?????????????????????
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      formatsdcard_pub;  // SD????????????????????????
  rclcpp::Publisher<cti_msgs::msg::GnssRTK>::SharedPtr gps_pub;  // gps????????????
  rclcpp::Publisher<cti_msgs::msg::GnssRTK>::SharedPtr rtk_pub;  // rtk????????????
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      compass_pub;  // compass????????????
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
      firmware_version_status_pub;  //??????????????????????????????
  rclcpp::Publisher<cti_chassis_msgs::msg::FirmWareInfo>::SharedPtr
      firmware_version_check_pub;  //???????????????????????????????????????????????????
  rclcpp::Publisher<cti_chassis_msgs::msg::VehicleState>::SharedPtr
      recv_chassis_info_pub;  //????????????????????????????????????
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      dust_box_autopush_pub;  //??????????????????????????????
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      boxlock_state_pub;  //?????????????????????
  rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
      rain_sensor_pub;  //???????????????????????????

  rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
      dust_box_state_info_pub;  // ????????????????????? cti_msgs/DataArray??????
  rclcpp::Publisher<cti_msgs::msg::DataArray>::SharedPtr
      dust_vehicle_state_info_pub;  // ??????????????????????????? cti_msgs/DataArray??????

  rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr
      chat_statue_pub;  //??????????????????(???????????????)
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
      lin_ult_data_pub;  // lin?????????????????????????????????
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
      lin_ult_data_v3_0_pub;  // lin?????????????????????????????????_3.0?????????
  rclcpp::Publisher<cti_chassis_msgs::msg::UltV30Datas>::SharedPtr
      new_lin_ult_data_v3_0_pub;  // ???lin?????????????????????????????????_3.0?????????

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shallow_sleep_pub;//?????????

  //----------------------ros???????????????-----------------------------------------//
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr timer2;
  rclcpp::TimerBase::SharedPtr timer_chat_timeout;
  rclcpp::TimerBase::SharedPtr timer_process_work_mode;
  rclcpp::TimerBase::SharedPtr main_timer;

  //--??????
  uint16_t light_type{0};

  //??????????????????
  int port_type{0};
  std::string port_type_name;
  std::string serial_port;
  // udp????????????
  std::string udp_ip;
  int udp_port{0};
  std::string udp_ip_dest;
  int udp_port_dest{0};

  double car_wheel_base;
  bool stm32_update_flag = false;
  unsigned int TIMEOUT{0};
  uint32_t seq_num{0};
  //--?????????
  double odom_x{0}, odom_x_4wd{0};
  double odom_y{0}, odom_y_4wd{0};
  double odom_th{0}, odom_th_4wd{0};
  nav_msgs::msg::Odometry odom, odom_4wd;
  //--IMU
  cti_msgs::msg::Sins sins_info;
  sensor_msgs::msg::Imu imu_info;
  //--??????
  double mileage{0};
  //--????????????
  cti_msgs::msg::BatteryState batteryState;
  cti_msgs::msg::BatteryCellsState BatCellsState;
  //--????????????
  cti_msgs::msg::RobotVersionDisplay RobotVersionDisplay;
  //--???????????????
  cti_msgs::msg::Range rangeDatas;
  sensor_msgs::msg::Range rangeData;
  //--????????????
  cti_msgs::msg::VehicleCtlRunInfo runinfo;
  //--??????ID??????????????????
  uint16_t cmd_id_global{0};
  //--????????????????????????map???map??????????????????
  std::map<uint16_t, double> cmd_send_map;
  //--????????????/?????????????????????
  uint32_t cmd_answer_success_cnt{0};
  uint32_t cmd_answer_fail_cnt{0};
  //--cmd send count
  uint32_t cmd_withid_send_cnt{0};
  uint32_t cmd_withid_recv_cnt{0};
  //--??????????????????
  cti_chassis_msgs::msg::CmdAnswer cmd_answer_cnt;
  //--timer3 ???????????????????????????
  uint8_t sendtime_loop_num{50};
  //--????????????????????????
  bool get_version_flag = false;
  //--??????????????????????????????????????????
  int8_t global_up_down_flag{0};
  int8_t global_switch_flag{0};
  double cmd_answer_timeout{0};
  // position global temp
  float global_pose[3] = {0};
  float global_q[4] = {0};

  //-- ???????????????
  int16_t global_k{0};       //??????????????????
  int16_t global_k_back{0};  //??????????????????
  //--??????????????????
  cti_msgs::msg::Rtcm box_laser;
  //--?????????????????????
  //--????????????log??????
  cti_chassis_msgs::msg::NavigationLog navigation_log;
  //--?????????????????????????????????
  float baro_raw_old{0};
  //--????????????????????????????????????
  std_msgs::msg::UInt16 baro_status;
  //--???????????????????????????
  uint8_t module_type_global{0};
  uint32_t module_error_code_global{0};
  uint8_t module_error_level_global{0};
  //--portIndex????????????
  uint8_t send_ctl_portIndex_cnt{0};
  uint8_t send_contraposition_portIndex_cnt{0};
  uint8_t send_poweroff_portIndex_cnt{0};
  uint8_t send_timenow_portIndex_cnt{0};
  //--sd????????????????????????
  std_msgs::msg::UInt8 formatsdcard_result;
  //--gps??????
  cti_msgs::msg::GnssRTK gps_data;
  //--rtk??????
  cti_msgs::msg::GnssRTK rtk_data;
  //--????????????????????????
  int32_t robotlocalizerstate_global{0};
  //--?????????????????????
  int8_t control_version_right = {0};  //???????????????????????? 0?????????????????????;
                                       // 1?????????????????????; -1?????????????????????;
  int max_control_board_version_head_{0};
  int min_control_board_version_head_{0};
  int max_control_board_version_mid_{0};
  int min_control_board_version_mid_{0};
  int max_control_board_version_end_{0};
  int min_control_board_version_end_{0};
  //--ptp????????????????????????
  bool need_resend_rough = false;
  time_sync_rough_msg_type sync_rough_msg_saved;
  bool check_rough_sync_res_timeoout;
  uint8_t check_rough_sync_res_cnt;
  uint8_t send_fine_sync_cnt{0};
  bool need_send_rough = true;
  bool need_send_fine_sync = false;
  //--?????????????????????
  bool check_clean_mechine_state;
  uint8_t recv_clean_mechine_motor_status;
  //--??????????????????????????????
  std_msgs::msg::Int8 firmwareVersionCheck;  //
  //--???????????????????????????
  int32_t box_type;
  //--???????????????????????????
  dusbin_rf_set_state_t dustbin_set_state;
  //--?????????????????????????????????
  bool set_dustbin_on_car = false;  // false:??????????????????true:????????????
  //--???????????????id?????????
  bool read_dustbin_id = false;
  //--?????????id????????????
  cti_chassis_msgs::msg::DustbinidState dustbinidstate;
  int pre_recv_dustbin_id;
  //--??????????????????
  std::string cti_run_ver;
  //--dataarray????????????????????????????????????
  motion_to_clean_t_new send_to_clean_cmd_new;
  //--??????????????????????????????
  vehicle_water_box_status vehicle_water_status;
  std::string config_file_path;
  std::deque<uint8_t> vehicle_water_tank_top;
  std::deque<uint8_t> vehicle_water_tank_bottom;
  int max_vehicle_water_tank_restore = 6;
  //--??????????????????????????????????????????????????????
  bool localization_limit = true;
  //--????????????????????????
  double box_chat_timeout_secs{0};
  double chassis_chat_timeout_secs{0};
  chat_state chassis_chat_state;
  chat_state box_chat_state;
  int dustbox_lora_id = -1;  // ???????????????id???, -1:??????id

  bool device_set_flag{false};
  msg_light_cmd_3_0 light_cmd_v3_0;
  //--????????????????????????
  motion_to_dust_box_t send_to_dust_box_cmd;
  battery_board_cmd_t send_battery_board;
  process_work_mode process_twork_mode_t;

  std::shared_ptr<ChassisCmncBase> chassisCmncBase;
};

#endif
