import numpy as np
import cv2
from os import path
from cv2 import aruco
import numpy as np
import sys
from collections import defaultdict


class ArucoMarker():
    aruco_dict = None
    test_image_folder_path = None
    aruco_dict_type = None
    loaded_correction_matrix = defaultdict()
    def __init__(self):
        self.aruco_dict_type = aruco.DICT_ARUCO_ORIGINAL
        self.aruco_dict = aruco.Dictionary_get(self.aruco_dict_type)
        self.test_image_folder_path = path.join(path.dirname(path.realpath(__file__)), "../testImages")
        self.arucoMarkerPoseBaseDir = path.dirname(path.realpath(__file__))
        sys.path.insert(0, self.arucoMarkerPoseBaseDir)
        

    def create_marker(self):
        import matplotlib.pyplot as plt
        import matplotlib as mpl
        fig = plt.figure()
        nx = 4
        ny = 3
        for i in range(1, nx*ny+1):
            ax = fig.add_subplot(ny,nx, i)
            img = aruco.drawMarker(self.aruco_dict,i, 700)
            plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
            ax.axis("off")
        filePath = path.join(self.test_image_folder_path, "arucoMarker01.png")
        plt.savefig(filePath)
        return filePath

    def detect_marker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        return frame_markers, corners, ids
    
    def visualize_results(self, frame_markers, corners, ids):
        import matplotlib.pyplot as plt
        plt.figure()
        plt.imshow(frame_markers)
        for i in range(len(ids)):
            c = corners[i][0]
            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
        plt.legend()
        plt.show()
    
    def  pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, markerLength):
        '''
        frame - Frame from the video stream
        matrix_coefficients - Intrinsic matrix of the calibrated camera
        distortion_coefficients - Distortion coefficients associated with your camera

        return:-
        frame - The frame with the axis drawn on it
        '''

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()


        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)
        rotation = []
        translation = []
            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, matrix_coefficients,
                                                                        distortion_coefficients)
                rotation.append(rvec)
                translation.append(tvec)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners) 

                # Draw Axis
                cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

        return frame, rotation, translation, ids

    def tutorial_01_create_and_read_marker(self):
        arucoMarker = ArucoMarker()
        filePath = arucoMarker.create_marker()
        frame_markers, corners, ids = arucoMarker.detect_marker(cv2.imread(filePath))
        arucoMarker.visualize_results(frame_markers, corners, ids)

    def tutorial_02_estimate_position(self, frame):
        #https://docs.opencv.org/4.5.4/d5/dae/tutorial_aruco_detection.html
        frame = cv2.imread(path.join(self.test_image_folder_path, "singleMarkersOriginal.jpg"))
        frame_markers, corners, ids = self.detect_marker(frame)

        #estimate position
        camMat = np.array([[14492.753623188406, 0, 1024],[0, 14492.753623188406, 1224],[0, 0, 1]])#camera matrix for f = 50mm with chipsize = 0.00345 mm and 2048x2448px
        distCoeffs = np.ndarray([0]) #distortion coefficients
        marker_length = 30.00 #mm

        pose = cv2.aruco.estimatePoseSingleMarkers(corners,marker_length,camMat,distCoeffs)  
        self.visualize_results(frame_markers, corners, ids)

    def load_correction_coefficients(self, markerPostfixName):
        if(markerPostfixName in self.loaded_correction_matrix):
            matrixCoeff = self.loaded_correction_matrix[markerPostfixName][0]
            disCoeff = self.loaded_correction_matrix[markerPostfixName][1]
        else:
            matrixCoeff = np.load(path.join(self.arucoMarkerPoseBaseDir, "calibrationFiles", "calibration_matrix_"+markerPostfixName+".npy"))
            disCoeff = np.load(path.join(self.arucoMarkerPoseBaseDir, "calibrationFiles", "distortion_coefficients_"+markerPostfixName+".npy"))
            self.loaded_correction_matrix[markerPostfixName] = ([matrixCoeff, disCoeff])
        return [matrixCoeff, disCoeff]
    
    def print_distance_to_aruco_marker(self, frame, markerLength, markerPostfixName):
        [matrixCoeff, disCoeff] = self.load_correction_coefficients(markerPostfixName)
        frame, rotationVector, translationVector, ids = ArucoMarker.pose_esitmation(frame, self.aruco_dict_type, matrixCoeff, disCoeff, markerLength)
        #for i in range(len(translationVector)):
            #print("Distance to marker {0}: x: {1:.2f}, y: {2:.2f}, z: {3:.2f} ".format(ids[i], 
            #translationVector[i][0][0][0],
            #translationVector[i][0][0][1],
            #translationVector[i][0][0][2]))
        return frame, rotationVector, translationVector, ids


    def tutorial_03_aruco_marker_pose_estimation(self, frame, markerLength, markerPostfixName):

        #ff = cv2.imread(path.join(self.test_image_folder_path, "arucoMarker01.png"))
        frame, rotationVector, translationVector, ids = self.print_distance_to_aruco_marker(frame, markerLength, markerPostfixName)
        return frame, rotationVector, translationVector, ids


    def tutorial_03_aruco_marker_pose_estimation_camera_feed(self):
        import time
        video = cv2.VideoCapture(0)
        # time.sleep(2.0)

        while True:
            ret, frame = video.read()

            if not ret:
                break
            
            output,_,_,_= self.print_distance_to_aruco_marker(frame, 0.1, "simulation")
            cv2.imshow('Estimated Pose', output)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        video.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    arucoMarker = ArucoMarker()
    # arucoMarker.tutorial_01_create_and_read_marker()
    # arucoMarker.tutorial_02_estimate_position()
    # arucoMarker.tutorial_03_aruco_marker_pose_estimation()
    arucoMarker.tutorial_03_aruco_marker_pose_estimation_camera_feed()
    