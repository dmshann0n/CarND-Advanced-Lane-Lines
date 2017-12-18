## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/distortion_correction.png "Undistorted"
[image2]: ./output_images/histogram.png "Histogram sample for lane finding"
[image3]: ./output_images/threshold_normal_perspective.png "Binary example, normal perspective"
[image4]: ./output_images/perspective_before_gradient.png "Birds-eye perspective transform"
[image5]: ./output_images/perspective_transform.png "Combined perspective + binary example"
[image6]: ./output_images/lanes_in_perspective.png "Lanes with lines in perspective"
[image7]: ./output_images/lane_identified_in_perspective.png "Lane filled"
[image8]: ./output_images/straight_combined.png "Output"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.

My project is driven by cli.py:

```
Usage: ./cli.py command [args...]

Commands:
  full-run
  test-calibrate-camera
  pickle-camera-calibration
  single-image
```

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for camera calibration is in lane_finder/camera_calibration.py. The class `CameraCalibration` encapsulates configuration, settings, and future undistortion. Because configuration is a time consuming operation, the `CameraCalibration` class also allows it's configuration to be pickled (via `to_pickle`).

Calling `CameraCalibration.config` creates the matrix and distortion coefficients. For each image located in the camera_cal directory, the corners of the chessboard are identified and the object points and corners are appended to a list (`obj_points` and `img_points` respectively) for the entire set. After iterating through the set, cv2.calibrateCamera is called and the class sets itself to configured.

The `CameraCalibration.undistort` method uses this configuration to undistort images using the cv2.undistort function and then crops the image based on the rectangular outline returned. If the camera calibration is not yet configured, it can be configured lazily.

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image1]

Note the second has been cropped after 'undistortion'.

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps at lines 137 through 167 in `lane_finder.py`, method: `_convert_to_color_gradient_threshold`).  Here's an example of my output for this step:

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The `LaneFinder` class creates matrix for perspective transform and the reverse transform upon initialization (in the `init_perspective_xforms` method). These matrices are later used by `_transform_to_birds_eye` and `_transform_from_birds_eye` -- methods that wrap the call to `cv2.warpPerspective` using the appropriate transformation matrix.

The methods `_calc_start_region_of_interest` and `_calc_birds_eye_region` generate the appropriate polygons for mapping via `cv2.getPerspectiveTransform`. The angles of the trapezoid generated in `_calc_start_region_of_interest` are based on measurement of the test image -- this is a somewhat brittle approach, as it doesn't correct for the position of the vehicle in the lane. However, for this exercise it provided reasonable results.

The code for my perspective transform includes a function called `warper()`, which appears in lines 1 through 8 in the file `example.py` (output_images/examples/example.py) (or, for example, in the 3rd code cell of the IPython notebook).  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:


I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Lane line pixels were identified first by taking a histogram of an image formatted as such:

![alt text][image5]

The resulting histogram was plotted as follows:

![alt text][image2]

The peaks of this histogram were used as a starting point for calculating windows, which was based on the lesson sample. After finding the window, the pixels in the windows were used to fit a line for each lane. (The `left_line_search` and `right_line_search` variables could be used to narrow the region of interest for future lane searches, but I didn't get to finish that!)

Lane finding history is stored in the `LaneHistory` class. There is an instance of this class for both left and right lane history. `LaneHistory` objects are responsible for identifying lanes that fall out of the margin of error (10% change from average). 

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The calculation for curvature of the lane is in `Line.curve` in `lane_finder.py`. This closely follows the sample -- it takes the polynomial for the line and the y plots, while converting it to approximate meters (using the conversion from 

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Given the lines identified above, filling the identified lane area was a matter of merging the left and right lines into a polygon, and filling that polygon. The return value of `_detect_lane_lines` is an image containing the filled polygon in the warped perspective. To apply it to the current 'world space' frame the perspective change is reversed using the previously initialized reverse matrix.

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video_with_lanes.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

There are many opportunities to improve this implementation.

* The lane history collected is underutilized. It could be used to look for a lane in a specific area, which helps handle issues from different lighting or road coloring (e.g., areas of asphalt vs concrete) that disrupt the lane finding. This issue was observed in the challenge video.

* The lighting conditions in the hardest challenge video posed a problem, as both the driver and the camera were blinded at times by direct sunlight. This obscured the road. Human drivers have similar challenges but are able to adjust their perspective/wear sunglasses :)

* The hardest challenge video also had curves present where the right lane line was obscured by the vehicle. The pipeline could predict where the lane is based on the left lane and past history where there are brief gaps of vision.
