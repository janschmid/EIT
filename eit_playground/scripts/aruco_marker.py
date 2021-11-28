import numpy as np
import cv2
from os import path
from cv2 import aruco
import numpy as np
import sys
from collections import defaultdict
import math
# from tf.transformations import quaternion_from_euler

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

    def get_global_pos_and_euler_angles(self, frame, markerLength, markerPostfixName):
        frame, rotationVector, translationVector, ids = self.print_distance_to_aruco_marker(frame, markerLength, markerPostfixName)
        eulerAngle=corrected_pos=None
        if(len(rotationVector)>=1):
            eulerAngle = ArucoMarker.rotationVectorToEulerAngles(rotationVector[0][0][0])
            corrected_pos = ArucoMarker.correct_rotation_matrix(translationVector[0][0][0],eulerAngle[0])
            corrected_pos.append(translationVector[0][0][0][2])
        return frame, eulerAngle, corrected_pos


    def tutorial_03_aruco_marker_pose_estimation_camera_feed(self):
        import time
        video = cv2.VideoCapture(0)
        # time.sleep(2.0)
        arucoMarker = ArucoMarker()
        while True:
            ret, frame = video.read()

            if not ret:
                break
            eulerAngle, corrected_global_position = arucoMarker.get_global_pos_and_euler_angles(frame, 0.07, "laptop")
            if (eulerAngle is not None and
                corrected_global_position is not None and
                len(eulerAngle)>0 and len(corrected_global_position)>0):
                print("{0}, {1}".format(eulerAngle,corrected_global_position))
            
            cv2.imshow('Estimated Pose', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        video.release()
        cv2.destroyAllWindows()

    def rotate(origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy

    def correct_rotation_matrix(translationVector, angle):
        origin = [0,0]
        if(len(translationVector)==3):
            qx, qy = ArucoMarker.rotate(origin, translationVector[0:2],angle)
            return [qx,qy]
        return None
            # print("rotation:{1}, Translation: {0}, eulerAngle: {2}".format(translationVector, (qx,qy), eulerAngle[2]))
            # print(qx, qy)
    
    def isRotationMatrix(R) :

        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
	 
    def rotationVectorToEulerAngles(R):
        R, _ = cv2.Rodrigues(R)
        if not (ArucoMarker.isRotationMatrix(R)):
            return
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

# if __name__ == "__main__":
#     arucoMarker = ArucoMarker()
#     # arucoMarker.tutorial_01_create_and_read_marker()
#     # arucoMarker.tutorial_02_estimate_position()
#     # arucoMarker.tutorial_03_aruco_marker_pose_estimation()
#     arucoMarker.tutorial_03_aruco_marker_pose_estimation_camera_feed()
    