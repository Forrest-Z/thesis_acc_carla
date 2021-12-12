def init():
    global PID_v_P
    global PID_v_I
    global PID_v_D
    global PID_d_P
    global PID_d_I
    global PID_d_D
    global distance_list
    global velocity_list
    global control_list
    global bot_target_speed
    global bot_speed
    global time_list
    global map_name

    map_name = 'maps/test_1.xodr'
    PID_v_P = 1
    PID_v_I = 1
    PID_v_D = 1
    PID_d_P = 1
    PID_d_I = 1
    PID_d_D = 1
    distance_list = []
    velocity_list = []
    control_list = []
    time_list = []
    bot_target_speed = 20
    bot_speed = 0

