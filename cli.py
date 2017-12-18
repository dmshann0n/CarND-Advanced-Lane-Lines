#!/usr/bin/env python3

import logging as log
import os

from clize import run
import cv2
from moviepy.editor import VideoFileClip

from lane_finder import camera_calibration, plotter, lane_finder

PLOT = plotter.Plotter(False)
INIT = False

CAMERA_CALIBRATION_PICKLE_PATH = 'camera_calibration.P'

def init_logging():
    log.basicConfig(
        format='%(asctime)s %(message)s',
        level=log.INFO)

def get_plotter(show_plots=False):
    PLOT.show_plots = show_plots
    return PLOT

def test_calibrate_camera():
    init_logging()
    plot = get_plotter()

    calibrator = camera_calibration.CameraCalibration(plot)
    calibrator.config()

    PLOT.show_plots = True
    img = cv2.cvtColor(cv2.imread('test_images/straight_lines1.jpg'), cv2.COLOR_BGR2RGB)
    calibrator.undistort(img)

def pickle_camera_calibration(path=None):
    if not path:
        path = CAMERA_CALIBRATION_PICKLE_PATH

    init_logging()
    plot = get_plotter(False)

    calibrator = camera_calibration.CameraCalibration(plot)
    calibrator.config()
    calibrator.to_pickle(path)


def _config_camera_calibrator(show_plots):
    plot = get_plotter()
    calibrator = camera_calibration.CameraCalibration(plot)

    if os.path.exists(CAMERA_CALIBRATION_PICKLE_PATH):
        calibrator.from_pickle(CAMERA_CALIBRATION_PICKLE_PATH)

    return calibrator

def single_image(img_path=None, show_plots=False):
    if not img_path:
        img_path = 'test_images/straight_lines1.jpg'

    init_logging()

    calibrator = _config_camera_calibrator(show_plots)
    finder = lane_finder.LaneFinder(calibrator, get_plotter(show_plots))

    img = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2RGB)
    finder.find_lanes(img)

def full_run(video_path=None, show_plots=False):
    if not video_path:
        video_path = 'project_video.mp4'

    init_logging()

    calibrator = _config_camera_calibrator(show_plots)
    finder = lane_finder.LaneFinder(calibrator, get_plotter(show_plots))

    log.debug(f'Processing video {video_path}')
    clip = VideoFileClip(video_path)
    updated = clip.fl_image(finder.find_lanes)

    split_path = video_path.split('.')
    new_file = "".join([split_path[0], '_with_lanes.', split_path[1]])
    updated.write_videofile(new_file, audio=False)

if __name__ == '__main__':
    run(full_run,
        test_calibrate_camera,
        pickle_camera_calibration,
        single_image)
