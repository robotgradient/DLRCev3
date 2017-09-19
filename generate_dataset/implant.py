import cv2
import numpy as np 
from time import sleep
import sys
 

cap=cv2.VideoCapture(1)
#input_name="frame_no_boxes"
#input_path="{}.{}".format(input_name,"jpg")

if len(sys.argv)>=2:
	image_name=sys.argv[1]
else:
	input_name="lego_pieces"
input_complete_name="{}{}".format(input_name,".jpeg")
mask_name="Mask_{}".format(input_complete_name)

while True:
	ret,frame=cap.read()
	cv2.imshow("select background",frame)
	if cv2.waitKey(10) & 0xFF==27:
		break
#cv2.namedWindow('mask')
mask=cv2.imread(mask_name)
legos=cv2.imread(input_complete_name)
#mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
#th,mask= cv2.threshold(mask,170,255,cv2.THRESH_BINARY)
merge=cv2.bitwise_and(mask,legos)


mask2=cv2.bitwise_not(mask)
#mask2= cv2.cvtColor(mask2, cv2.COLOR_BGR2GRAY)


with open("heloloasdfaafsda.txt", 'r') as outf:
	x=eval(outf.read())

print(mask2.shape)
#randpieces=np.random.randint(len(x)+1)
randpieces=len(x)
np.random.shuffle(x)
working_data=x[:randpieces]
print (randpieces,len(working_data))



flag=1
images_data=[]
box_data=[]
for reps in range(20):
	Newmask=np.ones(mask.shape,dtype=np.uint8)
	Newmask.fill(255)
	merge2=np.zeros(merge.shape,dtype=np.uint8)
	xnot=[]
	ynot=[]
	boxelements=[]
	for num in range(len(working_data)):
		#mask3=mask2[working_data[num][1]:working_data[num][3],working_data[num][0]:working_data[num][2]]
		
		#mask3=mask2[0:320,0:240,:]
		
		width=working_data[num][2]-working_data[num][0]
		height=working_data[num][3]-working_data[num][1]
		
		if num==0:
			newx=np.random.randint(frame.shape[1]-width)
			newy=np.random.randint(frame.shape[0]-height)
			xnot.extend(np.arange(newx,newx+width))
			ynot.extend(np.arange(newy,newy+height))
		else:
			while True:
				flag=1
				xnot=set(xnot)
				ynot=set(ynot)
				newx=np.random.randint(frame.shape[1]-width)
				newy=np.random.randint(frame.shape[0]-height)
				xoccupy=np.arange(newx,newx+width)
				yoccupy=np.arange(newy,newy+height)
				for i in xoccupy:
					for j in yoccupy:
						
						if (i in xnot) and (j in ynot):
							flag=0
							break
					if flag==0:
						break
				if flag==1:
					xnot=list(xnot)
					ynot=list(ynot)
					xnot.extend(np.arange(newx,newx+width))
					ynot.extend(np.arange(newy,newy+height))
					break

		masktomap=mask2[working_data[num][1]:working_data[num][3],working_data[num][0]:working_data[num][2],:]
		mergemask=merge[working_data[num][1]:working_data[num][3],working_data[num][0]:working_data[num][2],:]
		boxelements.append([newx,newy,newx+width,newy+height])
		Newmask[newy:newy+height,newx:newx+width,:]=np.copy(masktomap)
		merge2[newy:newy+height,newx:newx+width,:]=np.copy(mergemask)

	Newmask=cv2.morphologyEx(Newmask,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
	frame2=cv2.bitwise_and(Newmask,frame)
	frame2=cv2.bitwise_or(frame2,merge2)
	kernel = np.ones((5,5),np.float32)/25
	frame2 = cv2.filter2D(frame2,-1,kernel)
	#for box in boxelements:
	#	cv2.rectangle(frame2,(box[0],box[1]),(box[2],box[3]),(0,255,0),2)
	cv2.imshow("maskmerge",merge2)
	cv2.imshow("outputimages",frame2)
	cv2.waitKey(10)
	images_data.append(frame2)
	box_data.append(boxelements)
	image_name="{}_data_{}{}".format(input_name,reps,".jpeg")
	bounding_name="BB_{}_{}".format(image_name,reps)
	np.save(bounding_name,boxelements)
	cv2.imwrite(image_name,frame2)
images_data=np.array(images_data)
box_data=np.array(box_data)
print(images_data.shape,box_data.shape)
		#Newmask=mask2

	#print(mask2[working_data[num][0]:working_data[num][2],working_data[num][1]:working_data[num][3]])
#morphoimage=cv2.morphologyEx(img,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, size))

#cv2.imshow("old mask",mask2)	




#How to output the data



