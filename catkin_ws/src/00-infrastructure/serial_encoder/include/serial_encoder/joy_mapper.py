import rospy

def MapJoy(joy_msg):
        publishControl(joy_msg)
        processButtons(joy_msg)

def MapAxes(joy):
    button_scalar = 0.2
    # Forward and Backward ax
    forward_ax = joy.axes[1]
    forward_bt = joy.axes[7]
    if abs(forward_ax) > 0.0 and abs(forward_bt) > 0.0:
        # Both ax and button push, not allowed
        forward_ctl = 0.0
    else:
        if abs(forward_ax) > 0.0:
            forward_ctl = forward_ax
        else:
            forward_ctl = forward_bt * button_scalar
        
    # Left and right ax
    left_ax = joy.axes[0]
    left_bt = joy.axes[6]
    if abs(left_ax) > 0.0 and abs(left_bt) > 0.0:
        # Both ax and button push, not allowed
        left_ctl = 0.0
    else:
        if abs(left_ax) > 0.0:
            left_ctl = left_ax
        else:
            left_ctl = left_bt * button_scalar

    # Rotation
    rot_ax = joy.axes[3]
    rot_l = joy.buttons[4]
    rot_r = joy.buttons[5]
    if rot_l == 1 and rot_r == 1:
        # Push both at same time, not allowed
        rot_bt = 0.0
    else:
        if rot_l == 1:
            rot_bt = 1.0
        else:
            if rot_r == 1:
                rot_bt = -1.0
            else:
                rot_bt = 0.0
    if abs(rot_ax) > 0.0 and abs(rot_bt) > 0.0:
        rot_ctl = 0.0
    else:
        if abs(rot_ax) > 0.0:
            rot_ctl = rot_ax
        else:
            rot_ctl = rot_bt * button_scalar

    return forward_ctl, left_ctl, rot_ctl
    # Generate joystick commands
    # check protocol type
    if self.protocol_auto_flag:
        joystick_cmd = JoyAuto()
        joystick_cmd.channel_0 = forward_ctl
        joystick_cmd.channel_1 = left_ctl
        joystick_cmd.channel_2 = rot_ctl
        joystick_cmd.channel_3 = -0.2
        joystick_cmd.channel_3 = 0.6
        joystick_cmd.channel_5 = 0.0
        joystick_cmd.channel_6 = 0.0
        joystick_cmd.channel_7 = float(self.joy.buttons[0])
        joystick_cmd.channel_8 = float(self.joy.buttons[1])
        self.cmd_auto = joystick_cmd
    else:
        joystick_cmd = JoyRemote()
        joystick_cmd.channel_0 = forward_ctl
        joystick_cmd.channel_1 = left_ctl
        joystick_cmd.channel_2 = rot_ctl
        joystick_cmd.channel_3 = 0.0
        joystick_cmd.button_0 = self.joy.buttons[0]
        joystick_cmd.button_1 = self.joy.buttons[1]
        self.cmd_remote = joystick_cmd

    #print joystick_cmd
    # Publish Joystick commands
    #self.pub_joy_auto.publish(joystick_cmd)

