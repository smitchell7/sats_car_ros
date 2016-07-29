# this defines the structure of CarCommand. Do not use this directly
# 
std_msgs/Header header
bool HL_active
bool HL_atk_mode

bool LL_active

# Vel mode. Set a velocity with an amplitude and period. 
# Vel mode is only active if HL and LL are active
bool VEL_mode

float32 command_velocity
float32 d_amplitude
float32 d_period # 0 means velocity is constant

# Curvature automatic
bool curv_auto
float32 curv


# PWM mode is used for characterization. Use velocity mode for normal operation. 
# PWM mode is given priority over the HL and LL
bool PWM_mode
# pwm values are used only if pwm mode is set
float32 pwmL 
float32 pwmR
