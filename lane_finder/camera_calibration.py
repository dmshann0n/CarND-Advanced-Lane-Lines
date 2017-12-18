""" Abstractions and utilities for calculating the distortion of a camera
    given a set of chessboard images

    Heavily based on:
    http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
    """

import glob
import logging as log
import os
import pickle

import cv2
import numpy as np

DEFAULT_PATH = './camera_cal'
DEFAULT_FILE_MASK = '*.jpg'

# ideally this would be part of the meta-data
# for the incoming images, but since they're standardized
# in this set I'll just hack it!
CORNERS_X = 9
CORNERS_Y = 6

class CameraCalibration:
    def __init__(self, plot, path=None, mask=None):
        if not path:
            path = DEFAULT_PATH

        if not mask:
            mask = DEFAULT_FILE_MASK

        self.plot = plot
        self.path = path
        self.mask = mask

        self._configured = False
        self._mtx = None
        self._dist = None

    def _get_files(self):
        """ Returns a list of image files matching the search path and mask """

        glob_path = os.path.join(self.path, self.mask)
        return glob.glob(glob_path)

    def undistort(self, img):
        """ Given an image, uses calibration settings to undistort """

        img = img.copy()

        if not self._configured:
            self.config()

        h, w = img.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            self._mtx,
            self._dist,
            (w, h), 1, (w, h))

        dst = cv2.undistort(img, self._mtx, self._dist, None, new_mtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        self.plot.plot_images(img, dst)

        return dst

    def to_pickle(self, path):
        log.debug(f'Pickling config to {path}')

        if not self._configured:
            self.config()

        data = {'mtx': self._mtx, 'dist': self._dist}
        pickle.dump(data, open(path, 'wb'))

    def from_pickle(self, path):
        log.debug(f'Loading config from {path}')

        config_pickle = pickle.load(open(path, 'rb'))
        self._mtx = config_pickle['mtx']
        self._dist = config_pickle['dist']
        self._configured = True

    def config(self):
        """ Configures this instance for undistoring images based on the
            provided test images """

        log.debug(f'Searching for camera calibration samples at {self.path}')
        files = self._get_files()

        log.debug(f'Found {len(files)} images')

        # prepare object points
        points = np.zeros((CORNERS_Y * CORNERS_X, 3), np.float32)
        points[:, :2] = np.mgrid[0:CORNERS_X, 0:CORNERS_Y].T.reshape(-1, 2)

        obj_points = []
        img_points = []
        size = None

        for img_path in files:
            log.debug(f'Finding points in {img_path}')

            img = cv2.imread(img_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # record the size of this image.. they are all the
            # same size
            if not size:
                size = gray.shape[::-1]
                log.debug(f'Using size of {size} for calibration')

            found, corners = cv2.findChessboardCorners(gray, (CORNERS_X, CORNERS_Y), None)

            if not found:
                log.debug(' .. Unable to find corners .. ')
                continue

            obj_points.append(points)
            img_points.append(corners)

            self.plot.plot_images(
                img,
                lambda: cv2.drawChessboardCorners(img, (CORNERS_X, CORNERS_Y), corners, found))

        _, mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None)

        self._mtx = mtx
        self._dist = dist
        self._configured = True
