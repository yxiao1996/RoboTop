import math
import numpy as np

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

def mapPosto1(x):
    return 1 - (1 / math.exp(x))

def translate(channel_0, channel_1, channel_2, channel_3, button_0, button_1):
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

def translateCTLtoJoy(v_x, v_y, omega, phi=0):
    # translate control message to joystick message
    data_list = []

    # Map control value to (0, 1)
    vx_norm = sigmoid(v_x)
    vy_norm = sigmoid(v_y)
    omega_norm = sigmoid(omega)
    phi_norm = sigmoid(phi)

    # Map control value to channel
    channel_0 = vx_norm
    channel_1 = vy_norm
    channel_2 = omega_norm
    channel_3 = phi_norm

    # Rescale control value
    data_list.append(np.uint8(channel_0 * 255))
    data_list.append(np.uint8(channel_1 * 255))
    data_list.append(np.uint8(channel_2 * 255))
    data_list.append(np.uint8(channel_3 * 255))

    return data_list

if __name__ == '__main__':
    print translate(0.5, 0.5, 0.5, 0.5, True, False)