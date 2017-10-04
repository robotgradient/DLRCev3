import cv2
import cv2.aruco as aruco


dictionary =aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard_create(3,4,.025,.0125,dictionary)
#aruco.drawMarker( dict, id of the marker, size of the image in pixels)
marker0=aruco.drawMarker(dictionary,0,600)
marker1=aruco.drawMarker(dictionary,1,600)
marker2=aruco.drawMarker(dictionary,2,600)
marker3=aruco.drawMarker(dictionary,3,600)
marker4=aruco.drawMarker(dictionary,4,600)
marker5=aruco.drawMarker(dictionary,5,600)
#number of chessboard markers and size in metesrs
img=board.draw((200*3,200*4))#number of pixesls
cv2.imwrite("Marker_id_0.JPEG",marker0)
cv2.imwrite("Marker_id_1.JPEG",marker1)
cv2.imwrite("Marker_id_2.JPEG",marker2)
cv2.imwrite("Marker_id_3.JPEG",marker3)
cv2.imwrite("Marker_id_4.JPEG",marker4)
cv2.imwrite("Marker_id_5.JPEG",marker5)
cv2.waitKey()

