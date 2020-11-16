import cv2 as cv
import numpy as np

#Load the dictionary that was used to generate the markers.
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters =  cv.aruco.DetectorParameters_create()

frame = cv.imread("rover.png")


# Detect the markers in the image
markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
print(tuple(markerCorners[0][0][0]), tuple(markerCorners[0][0][2]))
print("markerIds", markerIds)

color = (0, 0, 255) 
thickness = 2

rectangle = cv.rectangle(frame,
                         tuple(markerCorners[0][0][0]),
                         tuple(markerCorners[0][0][2]),
                         color,
                         thickness)

cv.imshow("frame", frame)
cv.waitKey(0)
cv.destroyAllWindows()
