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
    def generate_binary_image(self, img, sobel_kenel=3, sob_thresh=(20, 100), mag_thresh=(30, 100), dir_thresh=(0.7, 1.3)):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kenel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kenel)
        # Calculate x y gradient
        absx = np.absolute(sobelx)
        absy = np.absolute(sobely)
        scaledx = np.uint8(255*absx/np.max(absx))
        scaledy = np.uint8(255*absy/np.max(absy))
        gradx = np.zeros_like(scaledx)
        grady = np.zeros_like(scaledy)
        gradx[(sob_thresh[0]<=scaledx)&(scaledx<=sob_thresh[1])] = 1
        grady[(sob_thresh[0]<=scaledy)&(scaledy<=sob_thresh[1])] = 1
        # Calculate magnitutude
        mag = np.sqrt(sobelx**2 + sobely**2)
        scaled_mag = np.max(mag) / 255
        scaled_sobel = (mag/scaled_mag).astype(np.uint8) 
        mag_binary = np.zeros_like(scaled_sobel)
        mag_binary[(mag_thresh[0]<=scaled_sobel)&(scaled_sobel<=mag_thresh[1])] = 1
        # Calculate dir gradient
        absdir = np.arctan2(absy, absx)
        dir_binary = np.zeros_like(absdir)
        dir_binary[(dir_thresh[0]<=absdir)&(absdir<=dir_thresh[1])] = 1
        # Combine together
        combined = np.zeros_like(dir_binary)
        combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

        return combined
       
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
    gray = cv2.cvtColor(undist_img, cv2.COLOR_BGR2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    absx = np.absolute(sobelx)
    scaledx = np.uint8(255*absx/np.max(absx))
    gradx = np.zeros_like(scaledx)
    gradx[(20<=scaledx)&(scaledx<=100)] = 1
    binary_img = lane_finder.generate_binary_image(undist_img)
    plt.imshow(binary_img, cmap='gray')
    plt.show()
    return



if __name__ == '__main__':
    print("Executing: Lane Line Finder.")
    lane_finder = LaneFinder()

    testimg = cv2.imread('./test_images/test1.jpg')
    run_pipeline(lane_finder, testimg)
    