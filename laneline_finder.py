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
    def __init__(self):
        # Transormation
        self.mtx = None
        self.dist = None
        self.M = None
        self.calibrated = False
        # Polynomial
        self.left_fit = None
        self.right_fit = None
        self.ploty = None
        self.has_first_fit = False
        # Curvature
        self.left_curverad = None
        self.right_curverad = None
# * Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
    def camera_calibration(self, nx=9, ny=6):
        objp = np.zeros((ny*nx,3), np.float32)
        objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)
        objpoints = []
        imgpoints = []
        images = glob.glob('./camera_cal/calibration*.jpg')
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nx,ny),None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
        self.mtx = mtx
        self.dist = dist
# * Apply a distortion correction to raw images.
    def distortion_correction(self, img):
        if not self.calibrated:
            self.camera_calibration()
        undist = cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
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
    def rectify_binary_image(self, img):
        img_size = (img.shape[1], img.shape[0])
        if self.calibrated:
            # Has the calculated M, warp directly
            return cv2.warpPerspective(img, self.M, img_size, flags=cv2.INTER_LINEAR)
            
        # Get the perspective transformation
        src = np.float32(
                        [[(img_size[0] / 2) - 63, img_size[1] / 2 + 100],
                        [((img_size[0] / 6) - 20), img_size[1]],
                        [(img_size[0] * 5 / 6) + 60, img_size[1]],
                        [(img_size[0] / 2 + 65), img_size[1] / 2 + 100]])
        dst = np.float32(
                        [[(img_size[0] / 4), 0],
                        [(img_size[0] / 4), img_size[1]],
                        [(img_size[0] * 3 / 4), img_size[1]],
                        [(img_size[0] * 3 / 4), 0]])
        M = cv2.getPerspectiveTransform(src, dst)
        self.M = M
        self.calibrated = True
        warped_img = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)
        return warped_img

    def find_lane_pixels(self, img, nwindows=9, margin=80, minpix=50, visualize=True):
        out_img = np.dstack([img, img, img])

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        if self.has_first_fit:
            left_fitx = self.left_fit[0]*nonzeroy**2+self.left_fit[1]*nonzeroy+self.left_fit[2]
            right_fitx = self.right_fit[0]*nonzeroy**2+self.right_fit[1]*nonzeroy+self.right_fit[2]
            left_lane_inds = (left_fitx-margin <= nonzerox) & (nonzerox <= left_fitx+margin)
            right_lane_inds = (right_fitx-margin <= nonzerox) & (nonzerox <= right_fitx+margin)
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]

            return leftx, lefty, rightx, righty, out_img

        histogram = np.sum(img[img.shape[0]//2:,:], axis=0)
        midpoint = np.int(histogram.shape[0] // 2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint
        window_height = np.int(img.shape[0]//nwindows)

        leftx_current = left_base
        rightx_current = right_base
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = img.shape[0] - (window+1)*window_height
            win_y_high = img.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            if visualize:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            good_left_inds = ((win_y_low<=nonzeroy)&(nonzeroy<=win_y_high)&(win_xleft_low<=nonzerox)&(nonzerox<=win_xleft_high)).nonzero()[0]
            good_right_inds = ((win_y_low<=nonzeroy)&(nonzeroy<=win_y_high)&(win_xright_low<=nonzerox)&(nonzerox<=win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if (len(good_left_inds)>minpix):
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if (len(good_right_inds)>minpix):
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            pass

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        self.has_first_fit = True

        return leftx, lefty, rightx, righty, out_img
# * Detect lane pixels and fit to find the lane boundary.
    def fit_polynomial(self, img, visualize=True):
        leftx, lefty, rightx, righty, out_img = self.find_lane_pixels(img)
        self.left_fit = np.polyfit(lefty, leftx, 2)
        self.right_fit = np.polyfit(righty, rightx, 2)
        left_fit = self.left_fit
        right_fit =  self.right_fit

        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        self.ploty = ploty

        try:
            left_fitx = left_fit[0]*ploty**2+left_fit[1]*ploty+left_fit[2]
            right_fitx = right_fit[0]*ploty**2+right_fit[1]*ploty+right_fit[2]
        except TypeError:
            print('fit_polynomial: failed to fit a line!')

        if visualize:
            out_img[lefty, leftx] = [255,0,0]
            out_img[righty, rightx] = [0,0,255]
            return out_img, left_fitx, right_fitx, ploty
        
        return out_img
# * Determine the curvature of the lane and vehicle position with respect to center.
    def calculate_curvature(self):
        y_eval = np.max(self.ploty)
        self.left_curverad = (1+(2*self.left_fit[0]*y_eval+self.left_fit[1])**2)**(3/2) / np.absolute(2*self.left_fit[0])
        self.right_curverad = (1+(2*self.right_fit[0]*y_eval+self.right_fit[1])**2)**(3/2) / np.absolute(2*self.right_fit[0])
        return
# * Warp the detected lane boundaries back onto the original image.
    def mat_to_real_world(self):
        return
# * Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
    def draw_outputs(self):
        return
    
def run_pipeline(lane_finder, img):
    undist_img = lane_finder.distortion_correction(img)
    binary_img = lane_finder.generate_binary_image(undist_img)
    warped_img = lane_finder.rectify_binary_image(binary_img)
    laned_image, left_fitx, right_fitx, ploty = lane_finder.fit_polynomial(warped_img)
    lane_finder.calculate_curvature()

    # Visualization
    curv_info =  "Left curverad: {}, Right curverad: {}".format(lane_finder.left_curverad, lane_finder.right_curverad)
    f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 10))
    f.tight_layout()
    ax1.imshow(img)
    ax2.imshow(binary_img)
    ax3.imshow(warped_img)
    ax4.imshow(laned_image)
    ax4.plot(left_fitx, ploty, color='yellow')
    ax4.plot(right_fitx, ploty, color='yellow')
    ax4.set_title(curv_info, fontsize=15, loc='right')
    plt.show()
    return

def save_test_imgs():
    # Opens the Video file
    cap= cv2.VideoCapture('project_video.mp4')
    i=0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == False:
            break
        cv2.imwrite('test_images/project_video/'+str(i)+'.jpg',frame)
        i+=1
 
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print("Executing: Lane Line Finder.")
    
    lane_finder = LaneFinder()

    testimg = cv2.imread('./test_images/project_video/0.jpg')
    run_pipeline(lane_finder, testimg)
    testimg = cv2.imread('./test_images/project_video/10.jpg')
    run_pipeline(lane_finder, testimg)
    testimg = cv2.imread('./test_images/project_video/20.jpg')
    run_pipeline(lane_finder, testimg)