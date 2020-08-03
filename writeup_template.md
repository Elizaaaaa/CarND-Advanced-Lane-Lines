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

[image1]: ./output_images/01-undistorted_image.png "Undistorted"
[image2]: ./output_images/02-binary_image.png "Binary"
[image3]: ./output_images/03-wrapped_image.png "Warp"
[image4]: ./output_images/04-first_polynomial.png "First Poly"
[image5]: ./output_images/05-search_with_polynomial.png "Poly"
[image6]: ./output_images/06-curvature.png "Curvature"
[image7]: ./output_images/07-curvature_in_meter.png "Curvature Meter"
[image8]: ./output_images/08-map_real_workd.png "Map"
[video1]: ./video_output/project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

All my implemations are located in `lane_finder.py`

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in "./examples/example.ipynb" (or in lines # through # of the file called `some_file.py`).  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

`LaneFinder().pipeline()`

#### 1. Undistort the original image

`LaneFinder().distortion_correction()`

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Generate a binary image

Combined color gradients and magnitude threshholds. Got a binary image like this:

![alt text][image2]

After running the project_video.mp4, I found that the lane finder is not working well when light is changing. To fix this, I added another thresholds: HLS - S channel. 
As a result the lane finder is able to find the lane when light is changing.

#### 3. Apply birds-eye view

`LaneFinder().rectify_binary_image()`

The function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
src = np.float32(
    [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
    [((img_size[0] / 6) - 10), img_size[1]],
    [(img_size[0] * 5 / 6) + 60, img_size[1]],
    [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 585, 460      | 320, 0        | 
| 203, 720      | 320, 720      |
| 1127, 720     | 960, 720      |
| 695, 460      | 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image3]

#### 4. Fit the polynomial

`LaneFinder().fit_polynomial()`

Use sliding windows to find the first polynomial:

![alt text][image4]

The lane will not change rapidly, so we can assume the polynomials are similar between frames. After having the first polynomial, the function can search the current polynomial based on the existing one.

![alt text][image5]

#### 5. Calculate the curvature

`LaneFinder().calculate_curvature()`

Calculate the curvature based on the line which found by fit_polynomial().

![alt text][image6]

However, this number is based on pixels. It needs to be mapped to meters.
Here I set the mapping factor as:

```
self.ym_per_pix = 30/720 
self.xm_per_pix = 3.7/700
```

Get a reasonable curvature after mapping. This value can be used to check lines' sanity.

![alt text][image7]


#### 6. Map the output to real world

`LaneFinder().map_to_real_world()`

Use Minv to map the result back to original image's transform. Then use cv2.addWeight() to overlay the result to the image:

![alt text][image8]

---

### Pipeline (video)

#### 1. Link to your final video output.  

Here's a [link to my video result](./video_output/project_video.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The LaneFinder worked pretty stable in the project_video.mp4. But I can observe that the lane position is having issue in challenges video. 
Applying offset check will help detection in challeng_video. And applying sanity checks & averaging should help the harder_challenge_video. I didn't got time to implement them this time. Will look into it later.