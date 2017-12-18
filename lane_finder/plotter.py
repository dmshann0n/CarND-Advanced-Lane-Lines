import logging as log

import cv2
import matplotlib.pyplot as plt
import numpy as np

WIDTH = HEIGHT = 3

class Plotter():
    def __init__(self, show_plots):
        self.show_plots = show_plots

    def plot_chart(self, data):
        if not self.show_plots:
            return

        plt.plot(data)
        plt.show()

    def plot_images(self, *imgs):
        if not self.show_plots:
            return

        f, axes = plt.subplots(len(imgs), 1, figsize=(WIDTH, HEIGHT * len(imgs)))
        f.tight_layout()

        # wrap the axes in a list so I don't have to
        # special case below
        if not isinstance(axes, (list, np.ndarray)):
            axes = [axes]

        for i, img in enumerate(imgs):
            if callable(img):
                axes[i].imshow(img())
            else:
                axes[i].imshow(img)

        plt.show()
