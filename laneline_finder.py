import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

# Define a class to receive the characteristics of each line detection
class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        #polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]  
        #radius of curvature of the line in some units
        self.radius_of_curvature = None 
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #difference in fit coefficients between last and new fits
        self.diffs = np.array([0,0,0], dtype='float') 
        #x values for detected line pixels
        self.allx = None  
        #y values for detected line pixels
        self.ally = None  

class LaneFinder():
# * Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
    def camera_calibration(self):
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
        objpoints = []
        imgpoints = []
        images = glob.glob('./camera_cal/calibration*.jpg')
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
        return mtx, dist
# * Apply a distortion correction to raw images.
    def distortion_correction(self, img, mtx, dist):
        undist = cv2.undistort(img, mtx, dist, None, mtx)
        return undist
# * Use color transforms, gradients, etc., to create a thresholded binary image.
    def generate_binary_image(self, img, s_thresh=(170,255), sx_thresh=(20,100)):
       
# * Apply a perspective transform to rectify binary image ("birds-eye view").
    def rectify_binary_image(self):
        return
# * Detect lane pixels and fit to find the lane boundary.
    def detect_lane_boundary(self):
        return
# * Determine the curvature of the lane and vehicle position with respect to center.
    def calculate_curvature(self):
        return
# * Warp the detected lane boundaries back onto the original image.
    def mat_to_real_world(self):
        return
# * Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
    def draw_outputs(self):
        return
    
def run_pipeline(lane_finder, img):
    mtx, dist = lane_finder.camera_calibration()
    undist_img = lane_finder.distortion_correction(img, mtx, dist)
    cv2.imwrite('01-undistorted_image.jpg', undist_img)
    binary_img = lane_finder.generate_binary_image(undist_img)
    cv2.imshow('img', binary_img)
    cv2.waitKey(0)
    return



if __name__ == '__main__':
    print("Executing: Lane Line Finder.")
    lane_finder = LaneFinder()

    testimg = cv2.imread('./test_images/test1.jpg')
    run_pipeline(lane_finder, testimg)

    # Step through the list and search for chessboard corners
    