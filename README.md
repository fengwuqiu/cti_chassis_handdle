######################  
auther:wangpeng  
######################  

######### modify record##########  
This is the master branch!  
201910251033  
git push v2.9 statard version used on the car.  
add boxcmd and boxlocdcmd to cmd_vel  
201911181109  
git push v3.0 statard version used on the car.  
add rfid (3 rfid reader)  
201911211749  
git  push v3.1 standard version used on the car  
4 rfid readers, Imu, ult  
201911281864  
git push v3.2 standard version used on the car  
add multiple threads. reduce the rate of loop down to 100hz  
201912051836  
git push v3.3 standard version used on the car  
add position linear_speed baro baro_height serial_status  
201912191231  
git push v3.4 version used on the car  
add cti_chassis_data_node  
20191226  
git push v3.5 version used on the car  
change the limit of the void rfid  
add box laser data  
20200102  
git push v3.6 version used on the car  
add chassis error code  
delete some message upload no used  
enlarge the zise of log file  
20200109  
v3.7  
增加地盘重要信息上传并记录log  
20200305  
v3.8  
修改记录log的格式，修正部分log数据记录错误的问题   
底盘错误码，和地盘通讯的结构体修改，发布的话题修改。  
发布逻辑为接到错误码就发，平时不发。  
20200319  
v3.9  
发送的结构体中的portindex,每发一次+1,作为标志  
v4.0  
修改上传包的频率检测时间间隔为1S.同时增加记录频率的log.  
v4.1  
检查数据包上传频率检测，屏蔽了频率为60000多的情况  
串口状态信息的话题和消息类型该了，新建了消息类型，能够直观的看到各项的状态  
增加SD卡格式化命令接口，并发布格式化结果  
v4.2  
20200405  
下发时间修改为UTC时间。  
v4.3  
20200415  
下发时间修改为UTC时间。  
，gps和磁力计上发。  
串口接收检测if条件由==改为>=.  
接收线程的检测时间间隔改为100ms。  
crc检测和接收线程检测变为并行，而不是当接收线程失败时才检测crc。  
记录时间下发的log，用于检测是否有时间下发和反映节点是否正常。  
v4.4  
20200422  
触边信息上传，定位状态下传。  
/cmd_vel 和障碍物话题 增加时间戳检测  
cti-fpga-v4.5  
20200428  
【新增】版本兼容（测试版）  
【修改】gps话题名称  
cti-fpga-v4.6  
20200512  
修改了记log的机制，重要信息一直记，不重要的信息隔一段时间记一次  
取消原来的时间下发，增加了PTP时间同步  
记录运控上发206命令：运控重启  
cti-fpga-v4.7  
20200521  
增加了清扫车功能  
cti-fpga-v4.8  
20200527  
增加清扫车超声波话题  
增加车辆失控状态信息上传  
cti-fpga-v4.8.1  
20200617  
增加运控重启后再次进行时间粗同步  
内存卡格式化命令 下发的命令portIndex =  msg->data  
cti-fpga-v4.8.2  
20200702  
打开版本兼容功能,并发布状态  
解决手动单次运控版本查询bug,增加查询反馈所有信息发布到话题/cti/chassis_serial/FW_check_report  
修改cti-fpga-data 记录log的bug  
增加顶升不断升降测试功能,并在launch中增加开关,默认为false  
cti-fpga-v4.8.3  
20200706  
解决版本兼容中的bug  
cti-fpga-v4.8.4  
20200813  
在状态发布中/cti/fpga_serial/serial_status 中增加更多的信息显示  
cti-fpga-v4.8.5  
20200820  
增加清扫车命令下发和状态上传的log记录  
增加清扫车超声波滤波   
cti-fpga-v4.8.6    
20200925  
修改记录log的方式.所有运控上传的数据都记录到chassis_data.log;  
增加新的清扫车超声波  
增加清扫车是否在车上的状态下发  
cti-fpga-v4.8.7    
20201026  
增加lora模块对接功能  
cti-fpga-v4.8.8    
20201029  
lord对接命令回调对箱号 %=10000 操作  
cti-fpga-v4.8.9  
20201102  
在箱子类型回调中增加设定箱子在车上  
为了兼容固件未更新的清扫车,在设定5次未得到正确回应则做超时,停止设定  
cti-fpga-v4.8.9.1  
20201118  
增加清扫箱远程升级  
cti-fpga-v4.9.0  
20201118  
增加清扫箱远程升级  
更改快递箱在车上的识别方式,不使用rfid,使用平台数据.用于上雷达数据切割  
cti-fpga-v4.9.0.2  
20201130  
在将大于200的命令作为普通命令处理时,将frame复制一下再处理,避免对frame进行修改.  
且对210,206命令做普通命令处理时,记录log   
增加清扫箱id查询选项  
cti-fpga-v4.9.0.3  
20201201  
增加清扫箱默认箱号设置  
cti-fpga-v4.9.0.4  
20201204  
解决升级bug  
cti-fpga-v4.9.1   
20201210   
更新到车上的版本  
4.9.0.2-4.9.0.4的所有功能增加   
无线充电功能,兼容有无线充电和无无线充电  
cti-fpga-v4.9.2     
20201217  
设置清扫箱的状态 取消lora配对 时发送 1 (发送0会导致一直设置不上)  
cti-fpga-v4.9.3     
20201228  
解决设置清扫箱在车上的位时,会一直发关闭清扫箱的命令的bug.使用move_controller发的的上一个命令;  
解决设定清扫箱回调就会设定id而导致通讯不畅的问题,开机设定一次后对收到的id做缓存,如果从运控接收的和回调的不一样才进行设定;  
将add_fanspeed合并过来,主要是将清扫箱和清扫车的功能做了兼容  
cti-fpga-v4.9.4       
20210119  
修改log记录,采用简洁的标志记录；  
cti-fpga-v4.9.5  
20210201  
增加集尘箱无线充电状态上传  
cti-fpga-v4.9.5.4  
20210220  
增加集尘箱垃圾满溢,垃圾到顶距离,箱尾超声波,电池电压上传,自动倾倒垃圾时箱子超声波发出8.88(无效)   
cti-fpga-v4.9.5.5   
20210225    
send_set_dustbin_id.baud = -1; //一定要设置成-1 否则会修改运控和lora通信的波特率  
cti-fpga-v4.9.5.6   
20210303    
增加灯带固件升级    
cti-fpga-v4.9.5.7   
20210310    
增加固件升级重发次数 5->15次     
cti-fpga-v4.9.5.8   
20210325    
增加gps OTA  解决设置运控lora 波特率=1的bug
cti-fpga-v4.9.5.9   
20210331    
下发两个角度给运控
cti-fpga-v4.9.6.0   
20210331    
v6.2的车吸尘箱的超声波有一个在里面,单独发布出来  , 增加磁吸锁状态发布  
cti-fpga-v4.9.6.1   
20210331    
增加更多的吸尘箱信息(代码中屏蔽掉),箱子无线充电消息增加时间戳,充电自动休眠.  
cti-fpga-v4.9.6.2   
20210331    
增加推杆ota  
cti-fpga-v4.9.7.1  
20210425  
兼容 环卫车 箱子的状态 增加电池风机力矩等之后和之前的 
cti-fpga-v4.9.7.2  
20210426  
增加边刷伸展,增加便刷速度,使用新的话题来整体控制
cti-fpga-v4.9.7.3   
20210515  
增加边刷伸展的单独控制,用来进行边刷伸展寿命测试,兼容了运控的箱子状态上传的60个字节的错误结构体,增加雨量传感器   
cti-fpga-v4.9.7.4   
20210517  
通过新的消息类型 cti_msgs::DataArray来控制吸尘箱和环卫车,还有智能垃圾箱,gps 数据经纬度转换   
cti-fpga-v4.9.7.5   
20210603   
智能垃圾箱上报信息增加lora_id,箱子4g状态发送     
cti-fpga-v4.9.8   
20210616   
兼容6.0的车,增加边刷错误码,    
cti-fpga-v4.9.9   
20210622   
箱子的超声波发布的判断条件更改, launch放松对运控版本的限制    
cti-fpga-v4.10.0   
20210706
增加水量模拟计算,垃圾量,4g信号   
cti-fpga-v4.10.1   
20210713
增加定位和障碍物对地盘限速的开关,增加吸尘箱5g状态的查询,车头装饰灯的控制和反馈  
cti-fpga-v4.10.2   
20210729
增加箱子和车的电量
cti-fpga-v4.10.3   
20210802
修改改吸尘箱垃圾量计算  

//2021年11月19日 删除humble分支 将分支humble_wwwp修改为humble    








