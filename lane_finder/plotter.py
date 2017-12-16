import logging as log

import cv2
import matplotlib.pyplot as plt

WIDTH = HEIGHT = 3

class Plotter():
    def __init__(self, show_plots):
        self.show_plots = show_plots

    def plot(self, *imgs):
        if not self.show_plots:
            return

        f, axes = plt.subplots(len(imgs), 1, figsize=(WIDTH, HEIGHT * len(imgs)))
        f.tight_layout()

        for i, img in enumerate(imgs):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            axes[i].imshow(img)

        plt.show()
