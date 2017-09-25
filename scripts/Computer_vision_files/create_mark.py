import cv2
import cv2.aruco as aruco


dictionary =aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard_create(3,4,.025,.0125,dictionary)
#number of chessboard markers and size in metesrs
img=board.draw((200*3,200*4))#number of pixesls
cv2.imwrite("Marker_3_4.JPEG",img)
cv2.imshow("marker",img)
cv2.waitKey()

