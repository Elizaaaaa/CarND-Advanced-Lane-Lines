 img = np.copy(img)
        r = img[:,:,0]
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        schannel = hls[:,:,2]
        sobelx = cv2.Sobel(r, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))

        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(sx_thresh[0]<=scaled_sobel)&(scaled_sobel<=sx_thresh[1])] = 1

        s_binary = np.zeros_like(schannel)
        s_binary[(s_thresh[0]<=schannel)&(schannel<=s_thresh[1])] = 1
        return color_binary


gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
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
        scaled_sobel = np.uint8(mag/scaled_mag)
        mag_binary = np.zeros_like(scaled_sobel)
        mag_binary[(mag_thresh[0]<=scaled_sobel)&(scaled_sobel<=mag_thresh[1])] = 1
        # Calculate dir gradient
        absdir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
        dir_binary = np.zeros_like(absdir)
        dir_binary[(dir_thresh[0]<=absdir)&(absdir<=dir_thresh[1])] = 1
        # Combine together
        combined = np.zeros_like(dir_binary)
        combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1


        f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 10))
        f.tight_layout()
        ax1.imshow(mag_binary)
        ax2.imshow(dir_binary)
        ax3.imshow(gradx)
        ax4.imshow(grady)
        plt.show()
