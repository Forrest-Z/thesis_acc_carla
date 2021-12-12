def init():
    global PID_v_P
    global PID_v_I
    global PID_v_D
    global PID_d_P
    global PID_d_I
    global PID_d_D
    global distance_list
    global distance_time
    global velocity_list
    global control_list

    global time_list
    global map_name

    global bot_target_speed
    global bot_target_vel
    global bot_velocity_list
    global bot_time_list
    global bot_speed

    global bot_speed_function_name
    global bot_speed_function_const
    global bot_speed_function_amplitude
    global bot_speed_function_freq

    map_name = 'maps/test_1.xodr'
    PID_v_P = 1
    PID_v_I = 1
    PID_v_D = 1
    PID_d_P = 1
    PID_d_I = 1
    PID_d_D = 1

    distance_list = []
    distance_time = []
    velocity_list = []
    control_list = []
    time_list = []

    bot_target_speed = 20
    bot_speed = 0

    bot_target_vel = []
    bot_velocity_list = []
    bot_time_list = []

    bot_speed_function_name = 'square'
    bot_speed_function_const = 10
    bot_speed_function_amplitude = 5
    bot_speed_function_freq = 0.05
