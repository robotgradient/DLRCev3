import cv2
import numpy as np 
from time import sleep
import sys
 
if len(sys.argv)>=2:
	n_backgrounds=int(sys.argv[1])
else:
	n_backgrounds=1

if len(sys.argv)>=3:
	n_lego=int(sys.argv[2])
else:
	n_lego=1

if len(sys.argv)>=4:
	lego_name=sys.argv[3]
else:
	lego_name="lego_pieces"

BB_name="BB_{}".format(lego_name)
Mask_name="Mask_{}".format(lego_name)

numberofdata=0
print (n_lego,n_backgrounds)
for numl in range(n_lego):
	# create names
	Lego_file="{}{}{}".format(lego_name,numl,".jpeg")
	BB_file="{}{}{}".format(BB_name,numl,".txt")
	Mask_file="{}{}{}".format(Mask_name,numl,".jpeg")
	for numb in range(n_backgrounds):

		back_file="{}{}{}".format("background",numb,".jpeg")

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
		for reps in range(20):
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
					newx=np.random.randint(frame.shape[1]-width)
					newy=np.random.randint(frame.shape[0]-height)
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
				

				Newmask[newy:newy+heightscaled,newx:newx+widthscaled,:]=np.copy(masktomap2)
				merge2[newy:newy+heightscaled,newx:newx+widthscaled,:]=np.copy(mergemask2)
				#cv2.rectangle(frame,(newx,newy),(newx+widthscaled,newy+heightscaled),[0,255,0],thickness=2)
				

			Newmask=cv2.morphologyEx(Newmask,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
			frame2=cv2.bitwise_and(Newmask,frame)
			frame2=cv2.bitwise_or(frame2,merge2)
			kernel = np.ones((5,5),np.float32)/25
			frame2 = cv2.filter2D(frame2,-1,kernel)
			cv2.imshow("outputimages",frame2)
			cv2.waitKey(10)

			#Uncomment to obtain the data in a matrix format
			#images_data.append(frame2)
			#box_data.append(boxelements)

			output_name="{}/Data_{}{}".format("Generated_data",numberofdata,".jpeg")
			bounding_name="{}/BB_Data_{}".format("Generated_data",numberofdata)
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