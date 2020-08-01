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