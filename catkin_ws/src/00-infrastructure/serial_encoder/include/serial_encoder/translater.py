import math
import numpy as np

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

def mapPosto1(x):
    return 1 - (1 / math.exp(x))

def translateRemote(channel_0, channel_1, channel_2, channel_3, button_0, button_1):
    # Initialize data buffer
    data_list = []

    # fetch value from message
    data_list.append(np.uint8((channel_0 + 1) * 127))
    data_list.append(np.uint8((channel_1 + 1) * 127))
    data_list.append(np.uint8((channel_2 + 1) * 127))
    data_list.append(np.uint8((channel_3 + 1) * 127))
    data_list.append(int(button_0))
    data_list.append(int(button_1))

    # Normalize control values
    """
    v_x_norm = sigmoid(v_x)
    v_y_norm = sigmoid(v_y)
    omega_norm = (omega) / 360.0
    """
    # Rescale control values
    # data_list = np.uint8(np.array(data_list) * 256)

    return data_list

def translateAuto(channel_0, channel_1, channel_2, channel_3, channel_4, channel_5, channel_6, channel_7, channel_8):
    # Initialize data buffer
    data_list = []

    # fetch value from message
    data_list.append(np.uint8((channel_0 + 1) * 127))
    data_list.append(np.uint8((channel_1 + 1) * 127))
    data_list.append(np.uint8((channel_2 + 1) * 127))
    data_list.append(np.uint8((channel_3 + 1) * 127))
    data_list.append(np.uint8((channel_4 + 1) * 127))
    data_list.append(np.uint8((channel_5 + 1) * 127))
    data_list.append(np.uint8((channel_6 + 1) * 127))
    data_list.append(np.uint8((channel_7)))
    data_list.append(np.uint8((channel_8)))

    # Normalize control values
    """
    v_x_norm = sigmoid(v_x)
    v_y_norm = sigmoid(v_y)
    omega_norm = (omega) / 360.0
    """
    # Rescale control values
    # data_list = np.uint8(np.array(data_list) * 256)

    return data_list

def translateCTLtoRemote(v_x, v_y, omega, phi, button_0, button_1):
    # translate control message to joystick message
    data_list = []

    # Map control value to channel
    channel_0 = v_x
    channel_1 = v_y
    channel_2 = omega
    channel_3 = phi

    # Rescale control value
    data_list.append(np.uint8((channel_0 + 1) * 127))
    data_list.append(np.uint8((channel_1 + 1) * 127))
    data_list.append(np.uint8((channel_2 + 1) * 127))
    data_list.append(np.uint8((channel_3 + 1) * 127))
    data_list.append(int(button_0))
    data_list.append(int(button_1))


    return data_list

def translateCTLtoAuto(v_x, v_y, omega):
    # translate control message to joystick message
    data_list = []

    # Map control value to channel
    channel_0 = v_x
    channel_1 = v_y
    channel_2 = omega
    channel_3 = 0.0
    channel_4 = 0.0
    channel_5 = 0.0
    channel_6 = 0.0
    channel_7 = 0.0
    channel_8 = 0.0

    # Rescale control value
    data_list.append(np.uint8((channel_0 + 1) * 127))
    data_list.append(np.uint8((channel_1 + 1) * 127))
    data_list.append(np.uint8((channel_2 + 1) * 127))
    data_list.append(np.uint8((channel_3 + 1) * 127))
    data_list.append(np.uint8((channel_4 + 1) * 127))
    data_list.append(np.uint8((channel_5 + 1) * 127))
    data_list.append(np.uint8((channel_6 + 1) * 127))
    data_list.append(np.uint8((channel_7)))
    data_list.append(np.uint8((channel_8)))

    return data_list

def translateCTLtoMove(pos_get, pos_throw, speed_get, speed_throw, cmd_claw, cmd_throw):
    # translate control message to joystick message
    data_list = []

    # Map control value to channel
    channel_0 = 0.0
    channel_1 = 0.0
    channel_2 = 0.0
    channel_3 = pos_get
    channel_4 = pos_throw
    channel_5 = speed_get
    channel_6 = speed_throw
    channel_7 = cmd_claw
    channel_8 = cmd_throw

    # Rescale control value
    data_list.append(np.uint8((channel_0 + 1) * 127))
    data_list.append(np.uint8((channel_1 + 1) * 127))
    data_list.append(np.uint8((channel_2 + 1) * 127))
    data_list.append(np.uint8((channel_3 + 1) * 127))
    data_list.append(np.uint8((channel_4 + 1) * 127))
    data_list.append(np.uint8((channel_5 + 1) * 127))
    data_list.append(np.uint8((channel_6 + 1) * 127))
    data_list.append(np.uint8((channel_7)))
    data_list.append(np.uint8((channel_8)))

    return data_list

if __name__ == '__main__':
    #print translateAuto(0.5, 0.5, 0.5, 0.5, True, False)
