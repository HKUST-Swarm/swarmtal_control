uint32 CTRL_POS_COMMAND=0
uint32 CTRL_VEL_COMMAND=1
uint32 CTRL_ATT_COMMAND=2
uint32 CTRL_MISSION_LOAD_COMMAND=3
uint32 CTRL_MISSION_END_COMMAND=4
uint32 CTRL_TAKEOF_COMMAND=5
uint32 CTRL_LANDING_COMMAND=6
uint32 CTRL_HOVER_COMMAND=7
uint32 CTRL_ARM_COMMAND=8

uint32 CTRL_PLANING_TGT_COMMAND=10

uint32 CTRL_FORMATION_IDLE=11

#Passive hold in master's local frame
uint32 CTRL_FORMATION_HOLD_0=12

#Passive hold in master's natural frame
uint32 CTRL_FORMATION_HOLD_1=13

#Activate fly in master's local frame
uint32 CTRL_FORMATION_FLY_0=14

#Activate fly master's natural frame
uint32 CTRL_FORMATION_FLY_1=15

uint32 command_type
#For formation param1 is id
#Param 2 is submodule
#3, 4, 5 is x y z
#Param 6 is yaw

# For POS param1 2 3 is x y z * 10000 
# 4 is yaw*10000 if yaw =666666, then use last yawsp
# 567 is vel feedforward 8 9 10

# For VEL param1 2 3 is x y z * 10000 
# 4 is yaw*10000 if yaw =666666, then use last yawsp

# For ATT 
# param1 2 3 4 is roll pitch yaw vz * 10000 
# 5 (>0 use thrust else z is velz)
# 6 (>0 use yaw else yawrate) 
# For TAKEoff param 1 is takeoff height * 10000
# For arm param = 0 is disarm param>0 is arm
# For mission param1-7 is  mission id

int32 param1
int32 param2
int32 param3
int32 param4
int32 param5
int32 param6
int32 param7
int32 param8
int32 param9
int32 param10