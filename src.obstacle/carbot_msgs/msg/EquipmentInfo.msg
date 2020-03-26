Header header
uint16 Obt1_Dis #1号超声波距离 单位:cm
uint16 Obt2_Dis #2号超声波距离 单位:cm
uint16 Obt3_Dis #3号超声波距离 单位:cm
int8 FP1_Status #1号防跌落通信状态 0-在线 1-下线
int8 FP2_Status #2号防跌落通信状态 0-在线 1-下线
uint16 FP1_Dis #1号防跌落距离 单位:cm
uint16 FP2_Dis #2号防跌落距离 单位:cm
int8 FC_Alarm #前碰撞触发报警 0-正常 1-报警
int8 OA_Alarm #前避障触发报警 0-正常 1-报警
int8 ES_Alarm #急停触发报警 0-正常 1-报警
int8 FP_Alarm #防跌落触发报警 0-正常 1-报警
int8 OBT_Status #超声波通信状态 0-在线 1-下线
int8 LFW_Status #左前电机通信状态 0-在线 1-下线
int8 RFW_Status #右前电机通信状态 0-在线 1-下线
int8 LBW_Status #左后电机通信状态 0-在线 1-下线
int8 RBW_Status #右后电机通信状态 0-在线 1-下线
int8 Laser_Status #雷达通信状态 0-在线 1-下线
int16 TEMP_Chassis #车体温度 单位 摄氏度
int8 LampColor #三色灯颜色 0-关闭 1-红 2-绿 3-黄 4-闪红 5-闪绿 6-闪黄
int16 ObstacleWarning #障碍物报警 0-无障碍 1-水平激光雷达报警 2- -15度激光雷达报警 3-前超声波避障报警
float32 RunningSpeed #运动速度 m/s
