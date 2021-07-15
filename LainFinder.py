import pickle
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
from moviepy.editor import VideoFileClip


class Line():
    def __init__(self, side):
        # was the line detected in the last iteration?
        self.ploty = None
        self.side = side
        self.detected = False  
        # x values of the last n fits of the line
        self.recent_xfitted = [] 
        #average x values of the fitted line over the last n iterations
        self.bestx = []     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = []  
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
        self.boxes = []
        self.margin = 50
        self.nwindows = 30
        self.minpix = 100
        self.error = None
        self.current_xfitted = None
        self.r = 0
        self.offset = 0
        self.saturation = False
        
    def fit_polynomial(self, warped):
        """
        warped has size: height, width; single channel, binary image
        """
        midpoint = warped.shape[1]//2
        if self.side == 'l':
            histogram = np.sum(warped[int(warped.shape[0]/3*2):, :midpoint], axis=0)
            x_base = np.argmax(histogram).astype(int)
        else:
            histogram = np.sum(warped[int(warped.shape[0]/3*2):, midpoint:], axis=0)
            x_base = np.argmax(histogram).astype(int) + midpoint
        window_height = warped.shape[0]//self.nwindows
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        x_current = x_base
        lane_inds = []
        self.boxes = []
        shift = 0
        for window in range(self.nwindows):
            win_y_low = warped.shape[0] - (window+1)*window_height
            win_y_high = warped.shape[0] - window*window_height
            win_x_low = x_current - self.margin  
            win_x_high = x_current + self.margin
            self.boxes.append(((win_x_low, win_y_low), (win_x_high,win_y_high)))
            good_inds = ((nonzeroy>win_y_low) & (nonzeroy<win_y_high)
                        & (nonzerox>win_x_low) & (nonzerox<win_x_high)).nonzero()[0]
            lane_inds.append(good_inds)
            if len(lane_inds)>2 and (len(lane_inds[-1])>self.minpix) and (len(lane_inds[-2])>self.minpix) and (len(lane_inds[-3])>self.minpix):
                shift = (nonzerox[lane_inds[-1]].mean() - nonzerox[lane_inds[-3]].mean())/2

            if good_inds.shape[0] > self.minpix:
                x_current = int(nonzerox[good_inds].mean())
            elif good_inds.shape[0] == 0:
                x_current += round(shift)
        try:
            lane_inds = np.concatenate(lane_inds)
        except ValueError:
            pass
        self.allx = nonzerox[lane_inds]
        self.ally = nonzeroy[lane_inds] 

        if self.ploty is None:
            self.ploty = np.linspace(0, warped.shape[0]-1, warped.shape[0])
        
        try:
            self.current_fit = np.polyfit(self.ally, self.allx, 2)
            self.recent_xfitted.append(xs:=(self.current_fit[0]*self.ploty**2 + self.current_fit[1]*self.ploty + self.current_fit[2]))
            self.current_xfitted = xs
            self.detected = True
            self.error = False
        except:
            self.error = True
            self.detected = False
        

            

    def search(self, warped):
        """
        warped has size: height, width; single channel, binary image
        """
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        if self.best_fit:
            lane_inds = (np.abs(nonzerox - (self.best_fit[-1][0]*nonzeroy**2 + self.best_fit[-1][1]*nonzeroy + self.best_fit[-1][2]))<self.margin).nonzero()[0]
        else:
            lane_inds = (np.abs(nonzerox - (self.current_fit[0]*nonzeroy**2 + self.current_fit[1]*nonzeroy + self.current_fit[2]))<self.margin).nonzero()[0]
        self.last_xfit = self.bestx[-1]
        self.allx = nonzerox[lane_inds]
        self.ally = nonzeroy[lane_inds]
        
        try:
            self.current_fit = np.polyfit(self.ally, self.allx, 2)
            self.recent_xfitted.append(xs:=(self.current_fit[0]*self.ploty**2 + self.current_fit[1]*self.ploty + self.current_fit[2]))
            self.current_xfitted = xs
            self.detected = True
            self.error = False
        except:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            self.detected = False
            self.error = True
    
    
    def smoothing(self, n):
        if len(self.recent_xfitted)>=n:
            xs = self.recent_xfitted[-n:]
            self.bestx.append(np.mean(xs,axis=0))
            self.best_fit.append(np.polyfit(self.ploty, self.bestx[-1], 2))
        else:
            if self.recent_xfitted:
                self.bestx.append(self.recent_xfitted[-1])
                self.best_fit.append(self.current_fit)



def corners_unwarp(img, mtx=mtx, dist=dist, M=M):

    img_size = (img.shape[1], img.shape[0])
    undst = cv2.undistort(img, mtx, dist, None, mtx)

    warped = cv2.warpPerspective(undst, M, 
                                img_size, flags=cv2.INTER_LINEAR)
    return warped


def binary(image, h=(1,60),s=(120,255), sob_thresh=(12,100), w=170,sobel_kernel=5, blur_kernel=9):
    
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 1, ksize=sobel_kernel)
    
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    scaled_sobel = cv2.GaussianBlur(scaled_sobel, (blur_kernel, blur_kernel), 0)
    H = hls[:,:,0]
    S = hls[:,:,2]
    combined = np.zeros_like(scaled_sobel)
    
    combined[
        ((scaled_sobel >= sob_thresh[0]) & (scaled_sobel <= sob_thresh[1]))
             &
      ((image[:,:,0] > w)|((S>s[0])&(S<s[1])))
            ] = 1
    return combined

def saturation(image, s=(120,255)):
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    S = hls[:,:,2]
    combined = np.zeros_like(S)
    
    combined[
        ((S>s[0])&(S<s[1]))
            ] = 1
    return combined

def binary_unwarp(image, func=binary):
    undst = cv2.undistort(image, mtx, dist, None, mtx)
    binary_unwarp = func(image) #*255
#     channels = np.dstack((b, b, b))
    warped = corners_unwarp(binary_unwarp)
    return warped, undst


def mark_image(undist, invM):
    color_warp = np.zeros_like(undist).astype(np.uint8)

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = pts_right = np.array([])
    if left_line.bestx and right_line.bestx:
        pts_left = np.array([np.transpose(np.vstack([left_line.bestx[-1], left_line.ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_line.bestx[-1], left_line.ploty])))])
        pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, invM, (undist.shape[1], undist.shape[0])) 
        # Combine the result with the original image
        result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
        return result
    else:
        return undist

def mark_poly(warped):
    out_img = np.dstack((warped,warped,warped))*255
    
    # highlight selected pixels
    out_img[left_line.ally, left_line.allx] = [255, 0, 0]
    out_img[right_line.ally, right_line.allx] = [0, 0, 255]
    
    
    
    # show pixel search areas
    window_img = np.zeros_like(out_img)
    if left_line.boxes:
        for (win_x_low, win_y_low), (win_x_high,win_y_high) in left_line.boxes:
            cv2.rectangle(out_img,(win_x_low, win_y_low),(win_x_high,win_y_high),(0,255,0), 2)
    else:
        left_line_window1 = np.array([np.transpose(np.vstack([left_line.last_xfit-left_line.margin, left_line.ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_line.last_xfit+left_line.margin, left_line.ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
    if right_line.boxes:
        for (win_x_low, win_y_low), (win_x_high,win_y_high) in right_line.boxes:
            cv2.rectangle(out_img,(win_x_low, win_y_low),(win_x_high,win_y_high),(0,255,0), 2)
    else:
        right_line_window1 = np.array([np.transpose(np.vstack([right_line.last_xfit-right_line.margin, right_line.ploty]))])
        right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_line.last_xfit+right_line.margin, right_line.ploty])))])
        right_line_pts = np.hstack((right_line_window1, right_line_window2))
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
        
    
    
    # show fitted poly if any
    if not left_line.error:
        lxs = left_line.current_xfitted.astype(int)
        lys = left_line.ploty.astype(int)
        lys = lys[(lxs> 0) & (lxs <out_img.shape[1])]
        lxs = lxs[(lxs> 0) & (lxs <out_img.shape[1])]
        out_img[lys, lxs] = [255,255,0]
    if not right_line.error:
        rxs = right_line.current_xfitted.astype(int)
        rys = right_line.ploty.astype(int)
        rys = rys[(rxs> 0) & (rxs <out_img.shape[1])]
        rxs = rxs[(rxs> 0) & (rxs <out_img.shape[1])]
        out_img[rys, rxs] = [255,255,0]
    
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    
    return result
        
        
        
def check():
    diffs = right_line.current_xfitted - left_line.current_xfitted
    if left_line.error or right_line.error:
        if not left_line.error:
            left_line.recent_xfitted.pop()
        if not right_line.error:
            right_line.recent_xfitted.pop()
        left_line.detected = False
        right_line.detected = False
        return False, diffs
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/700 # meters per pixel in x dimension

    left_fit = [left_line.current_fit[0]*xm_per_pix/(ym_per_pix)**2, xm_per_pix/ym_per_pix*left_line.current_fit[1]]
    right_fit = [right_line.current_fit[0]*xm_per_pix/(ym_per_pix)**2, xm_per_pix/ym_per_pix*right_line.current_fit[1]]

    y_eval = np.max(left_line.ploty)*ym_per_pix

    left_curverad = (1+(2*left_fit[0]*y_eval+left_fit[1])**2)**(3/2)/np.abs(2*left_fit[0])  
    right_curverad = (1+(2*right_fit[0]*y_eval+right_fit[1])**2)**(3/2)/np.abs(2*right_fit[0]) 
    
    if (diffs.std()>300) or \
    ((diffs.min() < image.shape[1]/6)) or \
    (diffs.max() > image.shape[1]*0.6) or \
    (((diffs.max()-diffs.min())>100)): 
        left_line.recent_xfitted.pop()
        right_line.recent_xfitted.pop()
        left_line.detected = False
        right_line.detected = False
        return False, diffs
    return True, diffs
        
def  calculate_r_offset(warped, checked):
    # positive offset means car in left to the lane's midline
    if checked:
        ym_per_pix = 30/720 # meters per pixel in y dimension
        xm_per_pix = 3.7/700 # meters per pixel in x dimension
        offset = ((left_line.bestx[-1][-1] + right_line.bestx[-1][-1])/2 - warped.shape[0]/2) * xm_per_pix

        left_fit = [left_line.best_fit[-1][0]*xm_per_pix/(ym_per_pix)**2, xm_per_pix/ym_per_pix*left_line.best_fit[-1][1]]
        right_fit = [right_line.best_fit[-1][0]*xm_per_pix/(ym_per_pix)**2, xm_per_pix/ym_per_pix*right_line.best_fit[-1][1]]

        y_eval = np.max(left_line.ploty)*ym_per_pix

        left_curverad = (1+(2*left_fit[0]*y_eval+left_fit[1])**2)**(3/2)/np.abs(2*left_fit[0])  
        right_curverad = (1+(2*right_fit[0]*y_eval+right_fit[1])**2)**(3/2)/np.abs(2*right_fit[0]) 
        r = left_curverad*np.sign(left_fit[0]) if len(left_line.allx) >= len(right_line.allx) else right_curverad*np.sign(right_fit[0])
        left_line.r = r
        left_line.offset = offset
        return r, offset
    else:
        return left_line.r,left_line.offset

def combine_add_text(overlay, marked_warped, r, offset, checked, diffs):
    r = left_line.r
    offset = left_line.offset
    left = np.concatenate((overlay, marked_warped), axis=0)
    right = np.zeros((1440,640,3),dtype=np.uint8)
    if np.abs(r)>5000:
        text1 = 'Nearly straight'
        text4 = ''
    else:
        text1 = f"""Curvature radius :{np.abs(r):.1f}m"""
        text4 = f"""Turning {'Left' if r < 0 else 'Right'}"""
    
    text2 = f"""Car is {np.abs(offset):.1f}m"""
    text3 = f"""{'Left' if offset > 0 else 'Right'} to the lane center"""
    cv2.putText(right, text1, (50,100), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(right, text4, (50,200), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(right, text2, (50,400), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(right, text3, (50,500), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.5, (255,255,255), 2, cv2.LINE_AA)
    if left_line.error:
        cv2.putText(right, 'Left Line Error', (50,700), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.5, (255,255,255), 2, cv2.LINE_AA)
    if left_line.error:
        cv2.putText(right, 'Right Line Error', (50,900), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.5, (255,255,255), 2, cv2.LINE_AA)
    if not checked:
        cv2.putText(right, 'Lane Detection Failed', (50,1100), cv2.FONT_HERSHEY_SIMPLEX, 
                       1.5, (255,255,255), 2, cv2.LINE_AA)
    else:
        cv2.putText(right, f'diff max {diffs.max():.2f}', (50,1000), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(right, f'diff min {diffs.min():.2f}', (50,1100), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (255,255,255), 2, cv2.LINE_AA)

    if left_line.saturation:
        cv2.putText(right, f'saturation', (50,1400), cv2.FONT_HERSHEY_SIMPLEX, 
                       1.5, (255,255,255), 2, cv2.LINE_AA)
    result = np.concatenate((left, right), axis=1)
    
    return result

# Final all in one pipeline function for video processing
def process(image, plot=False):
    warped, undist = binary_unwarp(image)
    if plot:
        plt.figure(figsize=(20,14))
        plt.title('undist')
        plt.imshow(undist)
        plt.show()
        plt.figure(figsize=(20,14))
        plt.title('warped')
        plt.imshow(warped)
        plt.show()
    left_line.boxes = []
    right_line.boxes = []
    if left_line.detected:
        left_line.search(warped)
    else:
        left_line.fit_polynomial(warped)
    if right_line.detected:
        right_line.search(warped)
    else:
        right_line.fit_polynomial(warped)
    checked, diffs = check()
    if not checked:
        warped, undist = binary_unwarp(image, func=saturation)
        left_line.boxes = []
        right_line.boxes = []
        left_line.search(warped)
        
        right_line.search(warped)

        checked, diffs = check()
        if checked:
            left_line.saturation=True
        
    if checked:# done
        left_line.smoothing(2)
        right_line.smoothing(2)
    marked_warped = mark_poly(warped)
    
    overlay = mark_image(undist, invM)
    
    r, offset = calculate_r_offset(warped, checked)
    result = combine_add_text(overlay, marked_warped, r, offset,checked, diffs)
    left_line.saturation = False
    

    if plot:
        plt.figure(figsize=(20,14))
        plt.title('marked_warped')
        plt.imshow(marked_warped)
        plt.show()
        plt.figure(figsize=(20,14))
        if checked:
            plt.title('overlay')
            plt.imshow(overlay)
            plt.show()
    
    return result


left_line = Line('l')
right_line = Line('r')

# Bird View unwarping matrix calculation
# src = np.float32([[578,462],[243,694],[1060,694],[715,462]]) #project and harder
src = np.float32([[602,475],[330,692],[1092,692],[717,475]]) # challenge 

dst = np.float32(
            [[img.shape[1]//3.5,0],[img.shape[1]//3.5,img.shape[0]-1],
            [(img.shape[1]-img.shape[1]//3.5),img.shape[0]-1],[(img.shape[1]-img.shape[1]//3.5),0]])
M = cv2.getPerspectiveTransform(src, dst)
invM = cv2.getPerspectiveTransform(dst, src)


# process the input video and save the output video
white_output = 'test_videos_output/'+'out1.mp4'
clip1 = VideoFileClip('./challenge_video.mp4')
white_clip = clip1.fl_image(process) 
white_clip.write_videofile(white_output, audio=False)
