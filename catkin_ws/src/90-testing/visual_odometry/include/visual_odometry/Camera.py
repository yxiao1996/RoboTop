import yaml
import cv2
import numpy as np
class Camera():
    def __init__(self):
        f = open('/home/pi/.ros/camera_info/camera.yaml')
        self.config = yaml.load(f)
        self.w = self.config["image_width"]
        self.h = self.config["image_height"]
        self.camera_matrix = self.config["camera_matrix"]
        self.dist_coeff = self.config["distortion_coefficients"]
        self.R = [[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]]
        self.t = [0]

    def undistort(self, img):
        # set imgae parameters
        h, w = img.shape[:2]
        mtx = np.array(self.camera_matrix['data']).reshape([3, 3])
        dist = np.array(self.dist_coeff['data'])

        # undistort image
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        return dst

if __name__ == '__main__':
    img = cv2.imread('./test0.png')
    c = Camera()
    dst = c.undistort(img)
    cv2.imshow("test", dst)
    cv2.waitKey(0)
    print c.w, c.h, c.camera_matrix, c.dist_coeff
