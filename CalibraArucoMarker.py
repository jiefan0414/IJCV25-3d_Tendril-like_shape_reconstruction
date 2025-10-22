"""
Framework   : OpenCV Aruco
Description : Calibration of camera and using that for finding pose of multiple markers
Status      : Working
References  :
    1) https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
    2) https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html
    3) https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
    4) Aruco Marker Generator: https://chev.me/arucogen/
"""
import numpy as np
import cv2
import cv2.aruco as aruco


class CalArucoPose():
    def __init__(self):
        print('aruco marker pose code!')
        self.cap = cv2.VideoCapture('/Users/jie/Documents/Coding/MATLAB_Jie/MyPAPER/ReconByPoject/Data/1st/202011182/202011182L.MOV')

        self.camMtx = np.array([[1.756278855719401e+03, 0.00000000e+00, 9.819760611747568e+02],
                                [0.00000000e+00, 1.755414209871876e+03, 5.026336668977876e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        """"#Pentex camera 1
        self.camMtx = np.array([[1.756278855719401e+03, 0.00000000e+00, 9.819760611747568e+02],
                                [0.00000000e+00, 1.755414209871876e+03, 5.026336668977876e+02],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        #Pentex camera 2
        self.camMtx = np.array([[1.746034183153780e+03, 0.00000000e+00, 9.897610371275571e+02],
                                 [0.00000000e+00, 1.743897159523071e+03, 5.168615018982824e+02],
                                 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        #Pentex camera 3
       self.camMtx = np.array([[1.766976985094071e+03, 0.00000000e+00, 9.805866647001145e+02],
                                 [0.00000000e+00, 1.763953084472804e+03, 5.218461347969713e+02],
                                 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])"""

        self.markerLength = 0.024
###------------------- Parameters need to be modified-----------------
if __name__ == '__main__':
    demoARUCO = CalArucoPose()
    start_frame = 101
    stop_frame = 105
    desired_Id = 628
    frame_id = 0

    saved_filename = 'CamExtrPara_Corners1.csv'
    demoARUCO.savepath = '/Users/jie/Documents/Coding/MATLAB_Jie/MyPAPER/ReconByPoject/Data/1st/202011182/'
    demoARUCO.saveData = []

    ###------------------ ARUCO TRACKER ---------------------------
    while(demoARUCO.cap.isOpened()):

        ret, frame = demoARUCO.cap.read()

        if frame_id < start_frame:
            frame_id = frame_id + 1
            continue

        # operations on the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        # parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients

            rvec, tvec , _ = aruco.estimatePoseSingleMarkers(corners, demoARUCO.markerLength, demoARUCO.camMtx, None)
            #(rvec-tvec).any() # get rid of that nasty numpy value array error            

            #### Draw a square around the markers and Axis
            for i in range(0, ids.size):
                # draw axis for the aruco markers
                aruco.drawAxis(frame, demoARUCO.camMtx, None, rvec[i], tvec[i], 0.05)
            aruco.drawDetectedMarkers(frame, corners)

            #### Save .CSV
            index_marker_desired = np.argwhere(ids.flatten()== desired_Id).flatten()
            if index_marker_desired.shape[0]:
                # print(index_marker_desired.shape
                print('frame id    : ', frame_id)
                print('rotation    : ', rvec[index_marker_desired].flatten())
                print('translation : ', tvec[index_marker_desired].flatten(), '\n')

                # -- Obtain the rotation matrix tag->camera
                R_Cam2Tag, _ = cv2.Rodrigues(rvec[index_marker_desired].flatten())
                R_Tag2Cam = R_Cam2Tag.T
                print('rotation mat Tag2Cam: ', R_Tag2Cam, '\n')
                print('trans vec Tag2Cam   : ', -tvec[index_marker_desired].flatten(), '\n')

                corner_marker_desired = np.squeeze(corners[index_marker_desired[0]], axis=0)
                print('corner_marker_desired: ', corner_marker_desired,'\n')
                data_save = np.hstack((frame_id, desired_Id, corner_marker_desired.flatten(), -tvec[index_marker_desired].flatten(), R_Tag2Cam.flatten()))
                demoARUCO.saveData.append(data_save)

            else:
                print('no detection of desired marker!\n')

            # code to show ids of the marker found
            strg = ''
            for i in range(0, ids.size):
                strg += str(ids[i][0])+', '

            cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        # display the resulting frame
        cv2.imshow('frame',frame)

        if frame_id == stop_frame:
            np.savetxt(demoARUCO.savepath + saved_filename, demoARUCO.saveData, delimiter=",", newline='\n')
            print('saved!')
            break

        if cv2.waitKey(0):
            print('Current frame is finished:', frame_id)
            print('------------------------------------------------------------')
        frame_id = frame_id + 1
        

        # When everything done, release the capture
    demoARUCO.cap.release()
    cv2.destroyAllWindows()





