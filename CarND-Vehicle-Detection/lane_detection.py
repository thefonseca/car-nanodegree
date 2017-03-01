import cv2
import glob
import matplotlib.pyplot as plt
import numpy as np

def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    '''
    Applies Sobel kernel threshold to an image.
    '''
    
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    # 2) Take the derivative in x or y given orient = 'x' or 'y'
    if orient == 'x':
        deriv = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    else:
        deriv = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
    
    # 3) Take the absolute value of the derivative or gradient
    deriv = np.absolute(deriv)
    
    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    deriv = ((deriv / np.max(deriv)) * 255).astype(np.uint8)
    
    # 5) Create a mask of 1's where the scaled gradient magnitude 
    # is > thresh_min and < thresh_max
    grad_binary = np.zeros_like(deriv)
    grad_binary[(deriv > thresh[0]) & (deriv < thresh[1])] = 1
    return grad_binary

def mag_thresh(img, sobel_kernel=3, mag_thresh=(0, 255)):
    '''
    Applies gradient magnitude threshold to an image.
    '''
    
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    # 2) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    
    # 3) Calculate the magnitude 
    mag = np.sqrt((np.square(sobelx) + np.square(sobely)))
    
    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    mag = ((mag / np.max(mag)) * 255).astype(np.uint8)
    
    # 5) Create a binary mask where mag thresholds are met
    mag_binary = np.zeros_like(mag)
    mag_binary[(mag > mag_thresh[0]) & (mag < mag_thresh[1])] = 1
    
    return mag_binary

def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    '''
    Applies gradient direction threshold to an image.
    '''
    
    # 1) Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    # 2) Take the gradient in x and y separately
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    
    # 3) Take the absolute value of the x and y gradients
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    
    # 4) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient 
    arctan = np.arctan2(abs_sobely, abs_sobelx)
    
    # 5) Create a binary mask where direction thresholds are met
    dir_binary = np.zeros_like(arctan)
    dir_binary[(arctan > thresh[0]) & (arctan < thresh[1])] = 1
    return dir_binary

def color_threshold(img):
    '''
    Applies color threshold to an image, using RGB and HLS colorspaces.
    '''
    
    r_channel = img[:,:,0]
    
    # 1) Convert to HLS colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
    h_channel = hsv[:,:,0]
    l_channel = hsv[:,:,1]
    s_channel = hsv[:,:,2]
    
    # 5) Create a binary mask where color thresholds are met
    binary = np.zeros_like(s_channel)
    binary[(h_channel > 15) & (h_channel < 100) & (s_channel > 120)] = 1
    binary[(r_channel > 220) & (r_channel < 255)] = 1
    
    return binary

def binary_image(img):
    '''
    Combine all threshold methods and morphological transformations
    to generate a binary image.
    '''
    
    # Apply each of the thresholding functions
    gradx = abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(10,150))
    grady = abs_sobel_thresh(img, orient='y', sobel_kernel=3, thresh=(10,150))
    mag_binary = mag_thresh(img, sobel_kernel=3, mag_thresh=(30,100))
    dir_binary = dir_threshold(img, sobel_kernel=3, thresh=(0.7, 1.3))
    color_binary = color_threshold(img)

    # combine thresholds
    binary = np.zeros_like(color_binary)
    binary[(gradx == 1) & (grady == 1) | (color_binary == 1) | ((mag_binary == 1) & (dir_binary == 1))] = 1

    # morphological transformations
    kernel = np.ones((3,3),np.uint8)
    binary = cv2.dilate(binary, kernel, iterations=2)
    binary = cv2.erode(binary, kernel, iterations=3)
    #binary = cv2.dilate(binary, kernel, iterations=2)
    return binary


def birds_eye(img, mtx, dist, show_lines=False):
    
    # 1) Undistort using mtx and dist
    undistorted = cv2.undistort(img, mtx, dist)
    
    src = np.array([[img.shape[1]/6 - 10, img.shape[0]],
                [img.shape[1]/2 - 48, img.shape[0]/1.6],
                [img.shape[1]/2 + 50, img.shape[0]/1.6],
                [img.shape[1] - 160, img.shape[0]]], np.float32)
    
    dst = np.array([[src[0,0] + 100, img.shape[0]],
                    [src[0,0] + 100, 0],
                    [src[-1,0] - 100, 0],
                    [src[-1,0] - 100, img.shape[0]]], np.float32)
    
    # d) use cv2.getPerspectiveTransform() to get M, the transform matrix
    M = cv2.getPerspectiveTransform(src, dst)

    # e) use cv2.warpPerspective() to warp your image to a top-down view
    warped = cv2.warpPerspective(undistorted, M, undistorted.shape[:2][::-1])
    
    if show_lines:
        if len(warped.shape) == 2:
            warped = np.dstack((warped, warped, warped))

        if np.max(warped) > 1:
            max_color = 255
        else:
            max_color = 1

        cv2.polylines(img, np.array([src], np.int32), True, (max_color,0,0), 3)
        cv2.polylines(warped, np.array([dst], np.int32), True, (max_color,0,0), 3)
    
    return warped, M

def lane_bases(binary_image):
    '''
    Find best guesses for lane bases using a histogram.
    '''

    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_image[int(binary_image.shape[0]/2):,:], axis=0)
    
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    return leftx_base, rightx_base

def apply_constraints(left_fit, right_fit, 
                      left_lane_inds, right_lane_inds, 
                      max_x, max_y):
    '''
    Checks and adjusts lane lines according to road geometry constraints.
    '''
    
    left_base = left_fit[0]*max_y**2 + left_fit[1]*max_y + left_fit[2]
    right_base = right_fit[0]*max_y**2 + right_fit[1]*max_y + right_fit[2]
    
    # Lanes should not have inverted curvatures
    if left_fit[0] < 0 and right_fit[0] > 0 or left_fit[0] > 0 and right_fit[0] < 0:
        #print('Lane detection discarded: coefficients with inverted signals')
        
        # Choose the lane with more pixels as the fit for the two lanes
        if len(left_lane_inds) > len(right_lane_inds):
            right_fit[:2] = left_fit[:2]
            right_fit[-1] = right_base - right_fit[0]*(max_y)**2 - right_fit[1]*max_y
        else:
            left_fit[:2] = right_fit[:2]
            left_fit[-1] = left_base - left_fit[0]*(max_y)**2 - left_fit[1]*max_y
    
    lane_width = 750
    max_lane_width = 850
    min_lane_width = 650
    
    # Lanes width should respect a range of values 
    if right_base - left_base > max_lane_width or right_base - left_base < min_lane_width:
        #print('Lane detection discarded: lane width={} pixels'.format(right_base - left_base))
        
        # Choose the lane nearest to the image center as the fit for the two lanes
        if abs(left_base - max_x/2) < abs(right_base - max_x/2):
            right_fit[:2] = left_fit[:2]
            right_fit[-1] = (left_base + lane_width) - right_fit[0]*(max_y)**2 - right_fit[1]*max_y
        else:
            left_fit[:2] = right_fit[:2]
            left_fit[-1] = (right_base - lane_width) - left_fit[0]*(max_y)**2 - left_fit[1]*max_y
    
    return left_fit, right_fit

def lane_polyfit(binary_image, nwindows=9, margin=100, minpix=50, left_fit=None, right_fit=None):
    '''
    Fits one second order polynomial for each lane line.  
    '''
    
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    
    # Optimization: if a previous left and right fits are passed as parameters, 
    # the function skips the sliding window search
    if left_fit is not None and right_fit is not None:
    #and left_fit[0] != right_fit[0] and left_fit[1] != right_fit[1]:
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))  
    
    else:
        #print("using windows to search lane pixels...")
        # Current positions to be updated for each window
        left_base, right_base = lane_bases(binary_image)
        leftx_current, rightx_current = left_base, right_base
        
        # Set height of windows
        window_height = np.int(binary_image.shape[0]/nwindows)
        
        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_image.shape[0] - (window+1)*window_height
            win_y_high = binary_image.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
                
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    left_fit = None
    right_fit = None
    
    # Fit a second order polynomial to each
    if len(lefty) > 0 and len(leftx) > 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    
    if len(righty) > 0 and len(rightx) > 0:
        right_fit = np.polyfit(righty, rightx, 2)
        
    if left_fit is None:
        left_fit = right_fit
    if right_fit is None:
        right_fit = left_fit
    
    # Perform sanity checks
    left_fit, right_fit = apply_constraints(left_fit, right_fit, 
                                            left_lane_inds, right_lane_inds,
                                            binary_image.shape[1] - 1,
                                            binary_image.shape[0] - 1)
        
    return left_fit, right_fit, left_lane_inds, right_lane_inds


def lane_image(binary_image, nwindows=4, margin=100, minpix=50, 
               left_fit=None, right_fit=None, show_lane_pixels=False):
    '''
    Creates an image with lane boundaries marked.
    '''
    
    nonzero = binary_image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    left_fit, right_fit, left_lane_inds, right_lane_inds = lane_polyfit(binary_image, 
                                                                        nwindows=6, 
                                                                        margin=100, 
                                                                        minpix=50, 
                                                                        left_fit=left_fit, 
                                                                        right_fit=right_fit)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_image.shape[0]-1, binary_image.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    if show_lane_pixels:    
        # Create an image to draw on and an image to show the selection window
        out_img = np.zeros_like(np.dstack((binary_image, binary_image, binary_image)))#*255
        window_img = np.zeros_like(out_img)

        # Color in left and right line pixels
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [1, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 1]
        
    else:
        window_img = np.zeros_like(np.dstack((binary_image, binary_image, binary_image)))

    # Generate a polygon to illustrate the lane boundaries
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx, ploty])))])
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx, ploty]))])
    line_pts = np.hstack((left_line_window2, right_line_window1))
    
    # Draw the lane onto the warped blank image
    cv2.fillPoly(window_img, np.int_([line_pts]), (0,1,0))
    
    if show_lane_pixels:
        result = cv2.addWeighted(out_img, 1, window_img, 0.4, 0)
        result[result > 1.] = 1.
    else:
        result = window_img
    
    return result, left_fit, right_fit


def curvature(left_fit, right_fit, x_range=1280, y_range=720, 
              xm_per_pix=3.7/700, ym_per_pix = 30/720):
    '''
    Calculate lane lines curvature and vehicle shift from the lane center.
    '''
    
    ploty = np.linspace(0, y_range-1, num=y_range)# to cover same y-range as image
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    camera_center = x_range/2
    lane_center = (right_fitx[-1] + left_fitx[-1])/2
    # shift in meters
    shift = (lane_center - camera_center) * xm_per_pix 

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, left_fitx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, right_fitx*xm_per_pix, 2)
    
    y_eval = y_range-1
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    
    return left_curverad, right_curverad, shift



def find_lane(img, mtx, dist, left_fit=None, right_fit=None):
    '''
    Returns an image with lane boundaries, radius of curvature and shift from lane center.
    '''
    
    binary = binary_image(img)
    binary_warped, M = birds_eye(binary, mtx, dist)
    lane, left_fit, right_fit = lane_image(binary_warped, left_fit=left_fit, right_fit=right_fit)
    
    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(lane, np.linalg.inv(M), (img.shape[1], img.shape[0]))
    
    if np.max(img) > 1:
        img = img/255.
        
    result = cv2.addWeighted(img.astype(np.float64), 1., newwarp, 0.3, 0)
    result[result > 1.] = 1.
    
    binary_small = cv2.resize(binary_warped, None, fx=.3, fy=.3, interpolation = cv2.INTER_CUBIC)
    binary_small = np.dstack((binary_small, binary_small, binary_small))
    result[:binary_small.shape[0], result.shape[1] - binary_small.shape[1]:] = binary_small
    
    left_curverad, right_curverad, shift = curvature(left_fit, right_fit)
    mean_curvature = np.mean((left_curverad, right_curverad))
    
    cv2.fillConvexPoly(result,np.array([(0,0),(0, binary_small.shape[0]-1),
                                        (result.shape[1] - binary_small.shape[1],
                                         binary_small.shape[0]-1),
                                        (result.shape[1] - binary_small.shape[1],0)]),(0,0,0)) 
    
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(result, 'Curvature: {:.01f}m'.format(mean_curvature), (50,60), font, 4, (1,1,1), 3)
    cv2.putText(result, 'Center shift: {:.01f}m'.format(shift), (50,130), font, 4, (1,1,1), 3)
    
    # Combine the result with the original image
    return result, left_fit, right_fit


def moving_average(a, n=20):
    '''
    Moving average of last n array elements.
    '''
    
    if len(a) == 0:
        return None
    
    if len(a) < n:
        return a[-1]
    
    ret = np.cumsum(a, axis=0, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return (ret[n-1:] / n)[-1]

def smooth_fit(left_fit, right_fit, left_fits, right_fits):
    '''
    Returns smooth (moving average) values for left and right 
    lines coefficients.
    '''
    
    left_fits.append(left_fit)
    right_fits.append(right_fit)
        
    return (moving_average(np.array(left_fits)), 
            moving_average(np.array(right_fits)))