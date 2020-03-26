Header header
int8 EN_CF #使能前碰撞传感器 #0-失效 1-使能
int8 EN_BF #使能后碰撞传感器 #0-失效 1-使能
int8 EN_FOBT #使能前超声避障传感器 #0-失效 1-使能
int8 EN_ES #使能急停功能 #0-失效 1-使能
int8 EN_FP #使能防跌落 #0-失效 1-使能
int8 MOTOR_CW #电机控制字
int32 MOTOR_LWS #左侧电机转速
int32 MOTOR_RWS #右侧电机转速
uint8 FP_DIS #防跌落触发距离 单位:cm
int8 LED_CW #三色灯控制字 0-关闭 1-红 2-绿 3-黄 4-闪红 5-闪绿 6-闪黄
uint8 FOBT_DIS #前避障触发距离 单位:cm
int16 Tirediameter #车轮长度 单位:cm
float32 RadarDistance #雷达有效距离 单位:m