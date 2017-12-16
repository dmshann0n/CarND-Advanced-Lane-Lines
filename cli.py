import logging

from clize import run
import cv2

from lane_finder import camera_calibration, plotter

PLOT = plotter.Plotter(False)
INIT = False

def init_logging():
    logging.basicConfig(
        format='%(asctime)s %(message)s',
        level=logging.DEBUG)

def get_plotter(show_plots=False):
    PLOT.show_plots = show_plots
    return PLOT

def test_calibrate_camera():
    init_logging()
    plot = get_plotter()

    calibrator = camera_calibration.CameraCalibration(plot)
    calibrator.config()

    PLOT.show_plots = True
    img = cv2.imread('./test_images/straight_lines1.jpg')
    calibrator.undistort(img)

def full_run(show_plots=False):
    init_logging()

    """ steps:
        1) camera calibration
        2) distortion correction
        3) color/gradient threshold
        4) perspective transform
        5) detect lane lines
        6) determine curvature
        7) DRAW LINES!!
        8) measure our position in the lane
    """

if __name__ == '__main__':
    run(full_run, test_calibrate_camera)
