import matplotlib.pyplot as plt
import numpy as np

def gen_bar(data, size_w=256, size_h=20):
    bar = np.zeros((size_h, size_w))
    for i in range(size_h):
        for j in range(size_w):
            if data < 127:
                if j > data and j <= 127:
                    bar[i][j] = 1
            else:
                if j < data and j > 127:
                    bar[i][j] = 1

    return bar

def plot_bar(bars, axes, display_encoder=True):
    assert len(bars) == len(axes)
    if display_encoder == True:
        name_pre = "encoder"
    else:
        name_pre = "decoder"
    for i, bar in enumerate(bars):
        axes[i].imshow(bar)
        axes[i].set_title(name_pre+" pole {}".format(i))

    #plt.pause(0.01)


#np.random.seed(19680801)
#data = np.random.random((1, 50, 50))
#data = np.random.randint(0, 2, (256, 20, 256), dtype='int')
#fig, axes = plt.subplots(3, 1)

#for i in range(256):
#    bar1 = gen_bar(i)
#    bars = [bar1, bar1, bar1]
#    plot_bar(bars, axes)

#for i in range(len(data)):
    #ax.cla()
#    axes[1].imshow(data[i])
#    axes[1].set_title("frame {}".format(i))
    # Note that using time.sleep does *not* work here!
#    plt.pause(0.1)
