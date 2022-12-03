# Python program to convert
# numpy array to image
  
# import required libraries
import os
import numpy as np
from PIL import Image as im
from matplotlib import pyplot as plt


def _normalize(x):
  return (x - x.min())/(x.max() - x.min())
  
# define a main function
def main():
    array = np.load("npybags/bicycle1_0003.npy")[:, :, 4]
    # array = _normalize(array)
    # data = im.fromarray((array * 255).astype(np.uint8))
    print(np.ptp(array))
    # data.save('gfg_dummy_pic.png')
    plt.matshow(array)
    plt.show()

  
# driver code
if __name__ == "__main__":
  main()