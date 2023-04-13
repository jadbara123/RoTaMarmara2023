import cv2
import numpy as np
import math
import gi

gi.require_version('Gtk', '2.0')

left_lane_list = []
right_lane_list = []

nwindows = 40
margin = 80
minpix = 5

class LaneDetection():
    def __init__(self) -> None:
        self.old_left = 0
        self.old_right = 0

    def main(self,img):
        #roi
        warped_img, M_inv = self.region_of_interest(img)
        #maske
        color_combined=self.combine_threshold(warped_img)
        binary_warped=np.copy(color_combined)
        #egri uydurma
        out_img, left_fit, right_fit, left_fitx, right_fitx,refangle = self.fit_polynomial2(binary_warped)

        return out_img,refangle
  
    def region_of_interest(self,img):  
        img_size = (img.shape[1], img.shape[0])
        offsetx = 95
        offsety = 54 #SAKARYAAAAAAAAAA
        # Source points taken from images with straight lane lines, these are to become parallel after the warp transform
        src = np.float32([[0, img.shape[0]-offsety], [offsetx, offsety],
                        [img.shape[1]-offsetx, offsety], [img.shape[1], img.shape[0]-offsety]
                        ])
        # Destination points are to be parallel, taken into account the image size
        dst = np.float32([
            [offsetx, img.shape[0]],             # bottom-left corner
            [offsetx, 0],                       # top-left corner
            [img_size[0]-offsetx, 0],           # top-right corner
            [img_size[0]-offsetx, img_size[1]]  # bottom-right corner
        ])
        # Calculate the transformation matrix and it's inverse transformation
        M = cv2.getPerspectiveTransform(src, dst)
        M_inv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(img, M, img_size)
        return warped, M_inv



    def fit_polynomial2(self,binary_warped):
        leftx, lefty, rightx, righty, out_img,refangle= self.find_lane_pixels(binary_warped)
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
            self.old_left = left_fit
            right_fit = np.polyfit(righty, rightx, 2)
            self.old_right = right_fit
        except:
            left_fit = self.old_left
            right_fit = self.old_right
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except TypeError:
            print('The function failed to fit a line!')
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty


        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [0, 0, 255]
        
        

        return out_img, left_fit, right_fit, left_fitx, right_fitx,refangle

    def find_lane_pixels(self,binary_warped):
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        midpoint = int(histogram.shape[0]//2)
        
        leftx_base = np.argmax(histogram[:midpoint])
        
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint



        window_height = int(binary_warped.shape[0]//nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin    

            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(0,255,0), 2) 

            x = (win_xleft_low+win_xleft_high)/2
            y = (win_y_low+win_y_high)/2
            #cv2.circle(out_img,(int(x),int(y)),2,(255,255,255),1)
            a = (win_xright_low+win_xright_high)/2
            b = (win_y_low+win_y_high)/2
            #cv2.circle(out_img,(int(a),int(b)),2,(255,255,255),1)

            z = [x,y]
            c = [a,b]
            left_lane_list.append(z)
            right_lane_list.append(c)
            
            #cv2.line(out_img,(int(a),int(b)),(int(x),int(y)),(255,255,255),1)
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
        
       
        xm,ym = self.get_mean(left_lane_list)
        am,bm = self.get_mean(right_lane_list)
        out_img,ref_angle = self.calculate_ref_points(out_img,xm,am)
                   
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            pass
        
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        

        return leftx, lefty, rightx, righty, out_img,ref_angle 
    def get_mean(self,array):
        xm = 0
        ym=0
        for i in range(len(array)):
            #x-ler  
            xm +=array[i][0]
            ym +=array[i][1]
        xm = xm/len(array)
        ym = ym/len(array)
        if len(array) >=nwindows :
            array.clear()       
        return int(xm),int(ym)   
    def calculate_ref_points(self,out_img,xm,am):

        #am = 640, xm =0
        if_none_refpoint = 250
        if xm <20:
            print("sol serit gg")
            ref_angle = np.arctan(((out_img.shape[1]/2)+if_none_refpoint-am)/(out_img.shape[0]/2))
            cv2.circle(out_img,(int(am-if_none_refpoint),int(out_img.shape[0]/2)),4,(255,100,100),30)
        elif 660>am>620:    
            print("sag serit gg")
            ref_angle = np.arctan(((out_img.shape[1]/2)-if_none_refpoint-xm)/(out_img.shape[0]/2))
            cv2.circle(out_img,(int(xm+if_none_refpoint),int(out_img.shape[0]/2)),4,(255,100,100),30)
        else:
            ref_angle = -np.arctan(((am+xm)/2-out_img.shape[1]/2)/(out_img.shape[0]/2))
            cv2.circle(out_img,(int((am+xm)/2),int(out_img.shape[0]/2)),4,(100,50,255),30)
            

        cv2.circle(out_img,(int(xm),int(out_img.shape[0]/2)),4,(255,0,255),30)
        cv2.circle(out_img,(int(am),int(out_img.shape[0]/2)),4,(255,0,255),30)
        
        cv2.circle(out_img,(int(out_img.shape[1]/2),int(out_img.shape[0]/2)),4,(0,0,255),30)
        return out_img,ref_angle


    def combine_threshold(self,img):
        
        def grey(image):
        #convert to grayscale
            image = np.asarray(image)
            return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        #Apply Gaussian Blur --> Reduce noise and smoothen image
        def gauss(image):
            return cv2.GaussianBlur(image, (5, 5), 0)

        #outline the strongest gradients in the image --> this is where lines in the image are

        grey = grey(img)
        grey = np.array(grey, dtype = 'uint8')
        grey = gauss(grey)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        abs = np.mean(np.array(grey))
        alpha = 14*(1/abs)**(1/4)-4.2
        #print('_________________________'+str(abs)+'    '+str(alpha))
        gray_img = cv2.convertScaleAbs(grey, 0, alpha)
        gray_cuntor = cv2.convertScaleAbs(grey, 0, 1.5)
        edges = cv2.Canny(gray_cuntor,0,50)
        ret, thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)
        final = cv2.bitwise_and(thresh, edges)
        cv2.imshow('final', final)
        return final 
