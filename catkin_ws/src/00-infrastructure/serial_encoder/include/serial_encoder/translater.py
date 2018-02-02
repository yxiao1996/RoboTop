import math
import numpy as np

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

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

if __name__ == '__main__':
    print translate(0.5, 0.5, 0.5, 0.5, True, False)