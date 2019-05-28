import cv2 as cv

if __name__=='__main__':
    full_grid = cv.imread("wandaf1.pgm")
    cv.imshow("wanda",full_grid)
    cv.waitKey(0)