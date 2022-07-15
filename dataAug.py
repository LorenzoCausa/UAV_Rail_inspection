import cv2
import os
import sys
from os.path import isfile, join
import random
import  numpy as np

pathIn="original_images/"
pathOut="augmented_images"

files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f))]
#for sorting the file names properly
files.sort()
print("augmenting...")
for i in range(len(files)):
    filename=pathIn + files[i]
    #reading each files
    img = cv2.imread(filename)
    cv2.imwrite(os.path.join(pathOut,"original"+str(i)+".jpg"),img)
    
    height, width, layers = img.shape
    size = (width,height)
    #print("adding: ",filename)

    for j in range(20):
        augmented=img

        if(random.random()<0.5):
            augmented = cv2.flip(augmented, 0)
            #print('flip0')

        if(random.random()<0.5):
            augmented = cv2.flip(augmented, 1)
            #print('flip1')

        if(random.random()<0.5):
            augmented = cv2.flip(augmented, -1)
            #print('flip-1')
    
        #if(random.random()<1):
            #contrast_coeff = random.random()+0.1
            #print(contrast_coeff)
            #augmented= augmented*contrast_coeff

            #print('contrast')

        if(random.random()<0.75):
            im_height,im_width,_=img.shape
            dsize = (int(im_width*((random.random()/2)+0.5)), int(im_height*((random.random()/2)+0.5)))
            augmented = cv2.resize(augmented, dsize, interpolation = cv2.INTER_AREA)
            #print('resize')

        if(random.random()<0.1):
            augmented = cv2.cvtColor(augmented, cv2.COLOR_BGR2GRAY)
            #print('gray')
    
        cv2.imwrite(os.path.join(pathOut,"superAugmented"+str(j)+"_"+str(i)+".jpg"),augmented)

    

    #brightness_coeff=random.randint(0, 50)
    #print(brightness_coeff)
    #augmented = img + brightness_coeff
    #cv2.imwrite(os.path.join(pathOut,"augmented_brightness_"+str(i)+".jpg"),augmented)