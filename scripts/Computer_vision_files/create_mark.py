import cv2
import cv2.aruco as aruco


dictionary =aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard_create(3,4,.025,.0125,dictionary)
marker1=aruco.drawMarker(dictionary,4,400)
marker2=aruco.drawMarker(dictionary,5,400)
#number of chessboard markers and size in metesrs
img=board.draw((200*3,200*4))#number of pixesls
cv2.imwrite("Marker_id_4.JPEG",marker1)
cv2.imwrite("Marker_id_5.JPEG",marker2)
cv2.imshow("marker",img)
cv2.waitKey()

