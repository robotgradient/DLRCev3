import cv2
import numpy as np 
from time import sleep
import sys
from tqdm import tqdm
 
if len(sys.argv)>=2:
	n_backgrounds=int(sys.argv[1])
else:
	n_backgrounds=13

if len(sys.argv)>=3:
	n_lego=int(sys.argv[2])
else:
	n_lego=9

if len(sys.argv)>=4:
	lego_name=sys.argv[3]
else:
	lego_name="lego_pieces"

BB_name="BB_{}".format(lego_name)
Mask_name="Mask_{}".format(lego_name)
numberofdata=0
print (n_lego,n_backgrounds)
BBOXES_OUTPUT_PATH="/home/dlrc/datasets/object_detection_dataset_v3/bboxes"
IMAGES_OUTPUT_PATH="/home/dlrc/datasets/object_detection_dataset_v3/images"
# TODO LABELS_OUTPUT_PATH=""

import os

os.makedirs(BBOXES_OUTPUT_PATH, exist_ok=True)
os.makedirs(IMAGES_OUTPUT_PATH, exist_ok=True)

files = os.listdir(".")
background_files = filter(lambda x: "background" in x and "jpeg" in x, files)


for numl in tqdm(range(n_lego)):
	# create names
	Lego_file="{}{}{}".format(lego_name,numl,".jpeg")
	BB_file="{}{}{}".format(BB_name,numl,".txt")
	Mask_file="{}{}{}".format(Mask_name,numl,".jpeg")

	for numb,back_file in enumerate(background_files):

		#Open files
		mask=cv2.imread(Mask_file)
		legos=cv2.imread(Lego_file)
		merge=cv2.bitwise_and(mask,legos)
		mask2=cv2.bitwise_not(mask)
		#frame=cv2.imread(back_file)
		with open(BB_file, 'r') as outf:
			x=eval(outf.read())

		
		#numberlegoperimages=np.random.randint(1,len(x)+1)
		numberlegoperimages=1
		working_data=x[:numberlegoperimages]

		flag=1
		images_data=[]
		box_data=[]
		#repetitions
		for reps in tqdm(range(1000)):
			frame=cv2.imread(back_file)
			Newmask=np.ones(mask.shape,dtype=np.uint8)
			Newmask.fill(255)
			merge2=np.zeros(merge.shape,dtype=np.uint8)
			xnot=[]
			ynot=[]
			boxelements=[]
			np.random.shuffle(x)
			numberlegoperimages=np.random.randint(1,len(x)+1)
			working_data=x[:numberlegoperimages]
			for num in range(len(working_data)):
				width=working_data[num][2]-working_data[num][0]
				height=working_data[num][3]-working_data[num][1]
				
				'''if num==0:
					newx=np.random.randint(frame.shape[1]-width)
					newy=np.random.randint(frame.shape[0]-height)

					# The resize ops assumption of total scale is 3 from top botton
					scale=abs(working_data[num][1]-newy)*3/480
					if newy > working_data[num][1]:
						heightscaled=round(height*scale)
						widthscaled=round(width*scale)
					elif newy==working_data[num][1]:
						heightscaled=height
						widthscaled=width
					else:
						heightscaled=round(height/scale)
						widthscaled=round(width/scale)

					xnot.extend(np.arange(newx,newx+widthscaled))
					ynot.extend(np.arange(newy,newy+heightscaled))
				else:'''
				while True:
					flag=1
					xnot=set(xnot)
					ynot=set(ynot)
					newx=np.random.randint(1,frame.shape[1]-width)
					newy=np.random.randint(1,frame.shape[0]-height)
					scale=1+abs(working_data[num][1]-newy)*2/480
					if newy > working_data[num][1]:
						heightscaled=round(height*scale)
						widthscaled=round(width*scale)
					elif newy==working_data[num][1]:
						heightscaled=height
						widthscaled=width
					else:
						heightscaled=round(height/scale)
						widthscaled=round(width/scale)
					xoccupy=np.arange(newx,newx+widthscaled)
					yoccupy=np.arange(newy,newy+heightscaled)
					if newx+widthscaled>=640 or newy+heightscaled>=480:
						flag=0
					else:
						for i in xoccupy:
							for j in yoccupy:
								
								if (i in xnot) and (j in ynot):
									flag=0
									break
							if flag==0:
								break
					if flag==1:
						#print("newy, oldy,newlimit, scale",newy,working_data[num][1],newy+heightscaled,scale)
						xnot=list(xnot)
						ynot=list(ynot)
						xnot.extend(np.arange(newx,newx+widthscaled))
						ynot.extend(np.arange(newy,newy+heightscaled))
						break

				masktomap=mask2[working_data[num][1]:working_data[num][3],working_data[num][0]:working_data[num][2],:]
				mergemask=merge[working_data[num][1]:working_data[num][3],working_data[num][0]:working_data[num][2],:]
				
				#Here scaled the mask
				boxelements.append([newx,newy,newx+widthscaled,newy+heightscaled])
				masktomap2=cv2.resize(masktomap,(widthscaled,heightscaled))
				mergemask2=cv2.resize(mergemask,(widthscaled,heightscaled))

				#print("mask, Newmask, scale",masktomap2.shape,(heightscaled,widthscaled)\
				transformation=np.random.randint(4)
				if transformation==1:
					#Total mirror
					Newmask[newy+heightscaled-1:newy-1:-1,newx+widthscaled-1:newx-1:-1,:]=np.copy(masktomap2)
					merge2[newy+heightscaled-1:newy-1:-1,newx+widthscaled-1:newx-1:-1,:]=np.copy(mergemask2)
				elif transformation==0:
					#nothing happens
					Newmask[newy:newy+heightscaled,newx:newx+widthscaled,:]=np.copy(masktomap2)
					merge2[newy:newy+heightscaled,newx:newx+widthscaled,:]=np.copy(mergemask2)
				elif transformation==2:
					#mirror horizontal
					Newmask[newy+heightscaled-1:newy-1:-1,newx:newx+widthscaled,:]=np.copy(masktomap2)
					merge2[newy+heightscaled-1:newy-1:-1,newx:newx+widthscaled,:]=np.copy(mergemask2)
				else:
					#mirror vertical
					Newmask[newy:newy+heightscaled,newx+widthscaled-1:newx-1:-1,:]=np.copy(masktomap2)
					merge2[newy:newy+heightscaled,newx+widthscaled-1:newx-1:-1,:]=np.copy(mergemask2)


				#cv2.rectangle(frame,(newx,newy),(newx+widthscaled,newy+heightscaled),[0,255,0],thickness=2)
				

			Newmask=cv2.morphologyEx(Newmask,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
			frame2=cv2.bitwise_and(Newmask,frame)
			frame2=cv2.bitwise_or(frame2,merge2)
			kernel = np.ones((5,5),np.float32)/25
			frame2 = cv2.filter2D(frame2,-1,kernel)
	
			#Uncomment to obtain the data in a matrix format
			#images_data.append(frame2)
			#box_data.append(boxelements)

			output_name="{}/Data_{}{}".format(IMAGES_OUTPUT_PATH,numberofdata,".jpeg")
			bounding_name="{}/BB_Data_{}".format(BBOXES_OUTPUT_PATH,numberofdata)
			np.save(bounding_name,boxelements)
			cv2.imwrite(output_name,frame2)

			numberofdata+=1
		#images_data=np.array(images_data)
		#box_data=np.array(box_data)
		#print(images_data.shape,box_data.shape)
		#Newmask=mask2

	#print(mask2[working_data[num][0]:working_data[num][2],working_data[num][1]:working_data[num][3]])
#morphoimage=cv2.morphologyEx(img,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, size))

#cv2.imshow("old mask",mask2)	




#How to output the data
