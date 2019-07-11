import cv2
if __name__=='__main__':    
    full_grid = cv2.imread("map_office_raw.pgm")    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))            
    closed1 = cv2.morphologyEx(full_grid, cv2.MORPH_CLOSE, kernel,iterations=1)       
    open1 = cv2.morphologyEx(closed1, cv2.MORPH_OPEN, kernel,iterations=1)       
    cv2.imwrite("map_office_new.pgm",open1)    #cv2.waitKey(0)
