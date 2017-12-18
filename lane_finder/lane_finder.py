import logging as log

import cv2
import numpy as np


""" steps:
    1) camera calibration               ✅
    2) distortion correction            ✅
    3) color/gradient threshold         ✅
    4) perspective transform            ✅
    5) detect lane lines                ✅
    6) determine curvature              ✅
    7) DRAW LINES!!                     ✅
    8) measure our position in the lane ✅
"""

def annotate_image(img, points):
    return cv2.polylines(
        img.copy(),
        [np.array(points, dtype=np.int32)],
        True,
        [255, 0, 0], 2)

def calc_fit_x(y, x, plot_y):
    fit = np.polyfit(y, x, 2)
    fit_x = fit[0]*plot_y**2 + fit[1]*plot_y + fit[2]
    return fit_x

class Line:
    """ Stores state for a single detected line (i.e., from one frame """
    def __init__(self, fit_x, plot_y, search_area):
        self.fit_x = fit_x
        self.plot_y = plot_y
        self.search_area = search_area

        self.points = np.transpose(np.vstack([fit_x, plot_y]))

        # take the LAST index -- which is going to be 620 (the bottom of the img)
        # then take the first ordinal of that so we get the actual x coord
        self.base_x = self.points[-1][0]

    # this is done somewhat more gracefully for finding lane position..
    Y_METERS_PER_PX = 30/720 # meters per pixel in y dimension
    X_METERS_PER_PX = 3.7/700 # meters per pixel in x dimension

    def curve(self):
        """ Returns the curve of this line in pixels """
        y_eval = np.max(self.plot_y)

        fit_m = np.polyfit(self.plot_y * self.Y_METERS_PER_PX, self.fit_x * self.X_METERS_PER_PX, 2)
        return ((1 + (2 * fit_m[0] * y_eval * self.Y_METERS_PER_PX + fit_m[1])**2)**1.5) / np.absolute(2*fit_m[0])

class LineHistory:
    """ Maintains state for the last MAX_QUEUE lines """

    MAX_QUEUE = 10
    AVG_THRESHOLD = .1

    def __init__(self):
        # python arrays can fake queue features (albeit slowly)
        # shouldn't matter with only 10ish items in it
        self._queue = []

        self._base_x_avg_start = None
        self._base_x_avg_end = None

    def base_x_reasonable(self, base_x_test):
        if not self._base_x_avg_start or not self._base_x_avg_end:
            # if there's no history then we'll assume it's safe!
            return True

        return base_x_test > self._base_x_avg_start and base_x_test < self._base_x_avg_end

    def _pre_calc(self):
        """ Pre-calculates data about history for use in later frames """

        base_x = []
        for item in self._queue:
            base_x.append(item.base_x)

        base_x_avg = np.mean(base_x)

        self._base_x_avg_start = base_x_avg * (1 - self.AVG_THRESHOLD)
        self._base_x_avg_end = base_x_avg * (1 + self.AVG_THRESHOLD)

    def has_last(self):
        return len(self._queue) > 0

    def last(self):
        return self._queue[-1]

    def push(self, line):
        self._queue.append(line)

        if len(self._queue) > self.MAX_QUEUE:
            self._queue.pop(0)

        self._pre_calc()



class LaneFinder:
    """ Calculates the location of a lane given a single frame using data
    extracted from the image and previous lane state """

    # dimensions for a distortion corrected frame
    # these are used to determine the initial search trapezoid
    TOP_Y = 400
    BOTTOM_LEFT_X = 220
    BOTTOM_RIGHT_X = 1030
    ANGLE_LEFT = 90 - 33.62
    ANGLE_RIGHT = 90 - 31.91
    UNDISTORTED_HEIGHT = 621

    def __init__(self, calibrator, plot):
        self.calibrator = calibrator
        self.plot = plot

        self.perspective_mtrx = None
        self.perspective_mtrx_rev = None

        self.init_perspective_xforms()

        self.last = None
        self.left_history = LineHistory()
        self.right_history = LineHistory()

    def init_perspective_xforms(self):
        """ Initializes the matrices for perspective warping
            and reversing the perspective warp """

        height = self.UNDISTORTED_HEIGHT

        src = np.array(self._calc_start_region_of_interest(height), dtype=np.float32)
        dst = np.array(self._calc_birds_eye_region(height), dtype=np.float32)

        self.perspective_mtrx = cv2.getPerspectiveTransform(src, dst)
        self.perspective_mtrx_rev = cv2.getPerspectiveTransform(dst, src)

    def _convert_to_color_gradient_threshold(self, img):
        """ Copies the image and converts it into a version with
            color and gradient thresholding for lane detection """
        copy = img.copy()
        hls = cv2.cvtColor(copy, cv2.COLOR_RGB2HLS)
        s_channel = hls[:, :, 2]

        gray = cv2.cvtColor(copy, cv2.COLOR_RGB2GRAY)
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobel_x = np.absolute(sobel_x)
        scaled_sobel = np.uint8(255*abs_sobel_x/np.max(abs_sobel_x))

        thresh_min = 20
        thresh_max = 100

        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1

        s_thresh_min = 170
        s_thresh_max = 255
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1

        combined_binary = np.zeros_like(sxbinary)
        combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1

        self.plot.plot_images(
            lambda: np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255,
            combined_binary)

        return combined_binary


    def _calc_start_region_of_interest(self, height):
        """ Returns the points on the polygon (trapezoid) that is used
            to outline the region of interest for lane finding. This function
            does not use any previous frame data.

            The region extends from an area in front of the hood of the vehicle
            to an area short of the horizon. """

        bottom_y = height

        # my trig is a bit rusty, but trying to find the offset
        # based on the angle created by our camera's perspective
        # (which is a hand-tuned constant for this exercise)
        length = bottom_y - self.TOP_Y
        offset_left = length * np.tan(np.radians(self.ANGLE_LEFT))
        offset_right = length * np.tan(np.radians(self.ANGLE_RIGHT))

        return [
            [self.BOTTOM_LEFT_X + offset_left, self.TOP_Y], # top left
            [self.BOTTOM_RIGHT_X - offset_right, self.TOP_Y], # top right
            [self.BOTTOM_RIGHT_X, bottom_y],  # bottom right
            [self.BOTTOM_LEFT_X, bottom_y], # bottom left
        ]

    def _calc_birds_eye_region(self, height):
        bottom_y = height

        return [
            [self.BOTTOM_LEFT_X, 0], # top left
            [self.BOTTOM_RIGHT_X, 0], # top right
            [self.BOTTOM_RIGHT_X, bottom_y],  # bottom right
            [self.BOTTOM_LEFT_X, bottom_y], # bottom left
        ]

    def _warp_to_perspective(self, img, mtrx):
        img_size = (img.shape[1], img.shape[0])
        return cv2.warpPerspective(img, mtrx, img_size, flags=cv2.INTER_LINEAR)

    def _transform_to_birds_eye(self, img):
        return self._warp_to_perspective(img, self.perspective_mtrx)

    def _transform_from_birds_eye(self, img):
        return self._warp_to_perspective(img, self.perspective_mtrx_rev)

    def _detect_lane_lines(self, img):

        # TODO: a lot of this came out of sample udacity code and needs to be DRY'd up

        height = img.shape[0]


        search_img = None
        # if we have history, let's use the last line 'search plots'
        # as a mask. this should be the common case!
        if self.left_history.has_last() and self.right_history.has_last():
            mask = np.zeros_like(img)

            cv2.fillPoly(
                mask,
                np.int_([self.left_history.last().search_area]),
                255)

            cv2.fillPoly(
                mask,
                np.int_([self.right_history.last().search_area]),
                255)

            search_img = cv2.bitwise_and(img, mask)

        else:
            # otherwise use our default image
            search_img = img

        histogram = np.sum(search_img[height//2:, :], axis=0)
        self.plot.plot_chart(histogram)

        out_img = np.dstack((img, img, img))*255

        midpoint = np.int(histogram.shape[0]/2)
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint

        log.debug(f'Starting from left x={left_x_base}, right x={right_x_base}')

        num_windows = 9
        window_height = height // num_windows

        nonzero = img.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        margin = 50
        min_pixels = 50

        left_x_current = left_x_base
        right_x_current = right_x_base

        left_lane_indices = []
        right_lane_indices = []

        for window in range(num_windows):
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height

            win_xleft_low = left_x_current - margin
            win_xleft_high = left_x_current + margin
            win_xright_low = right_x_current - margin
            win_xright_high = right_x_current + margin

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                (nonzero_x >= win_xleft_low) &  (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                (nonzero_x >= win_xright_low) &  (nonzero_x < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_indices.append(good_left_inds)

            right_lane_indices.append(good_right_inds)

            # If you found > min_pixels pixels, recenter next window on their mean position
            if len(good_left_inds) > min_pixels:
                left_x_current = np.int(np.mean(nonzero_x[good_left_inds]))

            if len(good_right_inds) > min_pixels:
                right_x_current = np.int(np.mean(nonzero_x[good_right_inds]))

        left_lane_indices = np.concatenate(left_lane_indices)
        right_lane_indices = np.concatenate(right_lane_indices)

        left_x = nonzero_x[left_lane_indices]
        left_y = nonzero_y[left_lane_indices]
        right_x = nonzero_x[right_lane_indices]
        right_y = nonzero_y[right_lane_indices]

        plot_y = np.linspace(0, height-1, height)

        left_fit = calc_fit_x(left_y, left_x, plot_y)
        right_fit = calc_fit_x(right_y, right_x, plot_y)

        left_line_search = self._convert_fit_to_poly(left_fit, plot_y, margin)
        right_line_search = self._convert_fit_to_poly(right_fit, plot_y, margin)

        window_img = np.zeros_like(out_img)
        cv2.fillPoly(window_img, np.int_([left_line_search]), (0, 255, 0))
        cv2.fillPoly(window_img, np.int_([right_line_search]), (0, 255, 0))

        search_lines_img = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

        self.plot.plot_images(search_lines_img)

        left_line = Line(left_fit, plot_y, left_line_search)
        right_line = Line(right_fit, plot_y, right_line_search)

        if not self.left_history.base_x_reasonable(left_line.base_x):
            log.debug(f'left x out of range ({left_line.base_x})')
            return None

        if not self.right_history.base_x_reasonable(right_line.base_x):
            log.debug(f'right x out of range ({left_line.base_x})')
            return None

        full_poly = np.hstack([
            np.array([left_line.points]),
            np.array([np.flipud(right_line.points)])])

        final_img = np.zeros_like(out_img)
        cv2.fillPoly(final_img, np.int_([full_poly]), (0, 255, 0))

        preview_img = cv2.addWeighted(out_img, 1, final_img, 0.3, 0)
        self.plot.plot_images(preview_img)

        self.left_history.push(left_line)
        self.right_history.push(right_line)

        return final_img

    def _convert_fit_to_poly(self, fit_x, plot_y, margin):

        left_line_window1 = np.array(
            [np.transpose(np.vstack([fit_x - margin, plot_y]))])

        left_line_window2 = np.array(
            [np.flipud(np.transpose(np.vstack([fit_x + margin, plot_y])))])

        return np.hstack((left_line_window1, left_line_window2))

    def _calculate_last_lane_position(self, undistorted):
        left, right = self.left_history.last(), self.right_history.last()

        total_lane = right.base_x - left.base_x
        lane_midpoint = (( right.base_x - left.base_x ) / 2 ) + left.base_x
        img_midpoint = undistorted.shape[1] // 2

        diff_pixels = img_midpoint - lane_midpoint

        # assuming 3.7m width of lane here..
        return 3.7 * (diff_pixels / total_lane)


    def find_lanes(self, frame):

        undistorted = self.calibrator.undistort(frame)

        self.plot.plot_images(
            undistorted,
            self._convert_to_color_gradient_threshold(undistorted))

        birds_eye = self._transform_to_birds_eye(undistorted)

        binary_warped = self._convert_to_color_gradient_threshold(birds_eye)

        height = undistorted.shape[0]

        self.plot.plot_images(
            undistorted,
            lambda: annotate_image(undistorted, self._calc_start_region_of_interest(height)),
            lambda: annotate_image(birds_eye, self._calc_birds_eye_region(height)))

        lane_overlay = self._detect_lane_lines(binary_warped)

        if lane_overlay is not None:
            lane_img = self._transform_from_birds_eye(lane_overlay)
            self.last = lane_img
        else:
            lane_img = self.last

        font = cv2.FONT_HERSHEY_SIMPLEX

        lane_position = self._calculate_last_lane_position(undistorted)
        left_curvature = self.left_history.last().curve()
        right_curvature = self.right_history.last().curve()

        cv2.putText(undistorted, f'Lane Position: {lane_position:.2f}m from mid',(10, 100), font, 1.2, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(undistorted, f'Curvature: {left_curvature:.2f}m {right_curvature:.2f}m',(10, 150), font, 1.2, (255, 255, 255), 2, cv2.LINE_AA)

        result = cv2.addWeighted(undistorted, 1, lane_img, 0.3, 0)
        self.plot.plot_images(result)
        return result
