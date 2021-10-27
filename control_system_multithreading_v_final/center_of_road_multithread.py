#!/usr/bin/env python
# coding: utf-8

# In[9]:


import io
import time
import cv2
import imutils
import cv2
import numpy as np
import threading

def padestrain_detect(img_curr,show_res):
    # Initializing the HOG person
    # detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # Reading the Image
    #image = cv2.imread('ex.jpg')
    image = img_curr   
    # Resizing the Image
    image = imutils.resize(image,
                           width=min(400, image.shape[1]))

    # Detecting all the regions in the 
    # Image that has a pedestrians inside it
    (regions, _) = hog.detectMultiScale(image, 
                                        winStride=(4, 4),
                                        padding=(4, 4),
                                        scale=1.05)
   
    # Drawing the regions in the Image
    for (x, y, w, h) in regions:
        if show_res:
            cv2.rectangle(image, (x, y), 
                          (x + w, y + h), 
                          (0, 0, 255), 2) 
        #需要测试 这里是arbitray value， w是人的宽度，h是高度
        if ((w > 40) & (h >140)):
            print("people is coming")
            if show_res:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image,'people is coming',(10,20), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    return image


# In[ ]:


# print the image with given title using plt
def printImg(Img,title,color = 'viridis'):
    plt.title(title)
    plt.imshow(Img.astype(np.uint8)[:,:,::-1],cmap = color)
    plt.xticks([]), plt.yticks([]) 
    plt.show()
# open image with given title using cv
def openImgBox(Img,title):
    cv2.imshow(title,Img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def shrink(img):
    hight,width,channel = img.shape
    img = cv2.resize(img,(int(width/7),int(hight/7)), interpolation =cv2.INTER_AREA)
    #printImg(img,"img")

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    #img = cv2.blur(img,(5,5))
    return img

def customized_k(img):
    #img = cv2.imread('road_jara.jpg')
    #printImg(img,"img")

    #printImg(img,'img')

    Z = img.reshape((-1,3))

    # convert to np.float32
    Z = np.float32(Z)

    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 5
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))
    return res2

def segment(res2):
    height,width,channel = res2.shape
    center_pix = res2[int(3*height/4),int(width/2)]
    new_res = np.zeros((height,width,channel))
    for i in range(height):
        for j in range(width):
            if res2[i,j,0] == center_pix[0]:
                new_res[i,j] = [0,255,255]
    #start_point_ori = [int(width/2),int(3*height/4)]
    #end_point_ori = start_point_ori
    #printImg(new_res,"segmented")
    return new_res

def left_right(img,buffer):
    left = 0
    right = 0
    height,width,channel = img.shape
    for i in range(int(height/2), height):
        for j in range(width):
            if img[i,j,1] == [255]:
                if j < (int(width/2)):
                    left = left +1
                else:
                    right = right +1
    print(str(left) + ',' + str(right))
    if ((left - right) > buffer):
        #print('go left')
        return 2 # should turn right
    elif ((right - left) > buffer):
        #print ('go right')
        return 1 # should turn left
    else:
        #print("good")
        return 0
    
def left_right2(img,buffer):
    left = 0
    right = 0
    height,width,channel = img.shape
    for i in range(int(height/2), height):
        for j in range(width):
            if in_range(img[i,j],8,img[int(3*height/4),int(width/2)]):
                if j < (int(width/2)):
                    left = left +1
                else:
                    right = right +1
    if ((left - right) > buffer):
        print('go left')
    elif ((right - left) > buffer):
        print ('go right')
    else:
        print("good")
    print(str(left) + ',' + str(right))
        
def get_arrow(step,img):
    new_res = img
    height,width,channel = img.shape
    start_point = [int(width/2),int(3*height/4)]
    end_point = start_point
    curr_hi = int(3*height/4)
    curr_wi = int(width/2)
    rang = int((int(3*height/4) - int(height/3))/step)
    for i in range(37):
        curr_hi = curr_hi - step
        if new_res[curr_hi,curr_wi,1] == 255.0:
            end_point[1] = curr_hi 
        elif new_res[curr_hi,curr_wi + 1,1] == 255.0:
            end_point[0] = curr_wi + 1
            curr_wi = curr_wi + 1
            #make turn right
        elif new_res[curr_hi,curr_wi - 1,1] == 255.0:
            end_point[0] = curr_wi - 1
            curr_wi = curr_wi - 1
            #make turn left
        elif new_res[curr_hi,curr_wi + 3,1] == 255.0:
            end_point[0] = curr_wi + 3
            curr_wi = curr_wi + 3
        elif new_res[curr_hi,curr_wi - 3,1] == 255.0:
            end_point[0] = curr_wi - 3
            curr_wi = curr_wi - 3
        elif new_res[curr_hi,curr_wi + 5,1] == 255.0:
            end_point[0] = curr_wi + 5
            curr_wi = curr_wi + 5
        elif new_res[curr_hi,curr_wi - 5,1] == 255.0:
            end_point[0] = curr_wi - 5
            curr_wi = curr_wi - 5
    return end_point

import time
def rdDetect(img):
    #printImg(img,'original image')
    start_time = time.time()
    res0 = shrink(img)
    #printImg(res0,'shrinked image')
    res1 = customized_k(res0)
    #printImg(res1,'K-means segmentation')
    rea = segment(res1)
    LR_res = left_right(rea,80)
    #printImg(rea,'segmented')
    end_point = get_arrow(1,rea)
    height,width,channel = rea.shape
    start_point = (int(width/2),int(3*height/4))
    end_point = (end_point[0],end_point[1])
    img = cv2.arrowedLine(res0, start_point, end_point,(0,0,255), 3, tipLength = 0.5)
    #printImg(img,"final result")
    print("--- %s seconds ---" % (time.time() - start_time))
    return img, LR_res


# In[10]:


def init_cam():
    frameWidth = 640
    frameHeight = 480
    cap = cv2.VideoCapture(0)
    cap.set(3,frameWidth)
    cap.set(4,frameHeight)
    return cap


# In[11]:
stop_thread = False
COR_res = 0

def capture_res(cap,show_res = True): # if needed, make it to true
    success,img = cap.read()
    img, LR_res = rdDetect(img)
    if 0:
        if (LR_res == 0):
            print("center")
        elif LR_res == 1:
            print("deviation to right")
        elif LR_res == 2:
            print("deviation to left")
    if show_res:
        img_res = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
        cv2.imshow('result',img_res)
    return LR_res
    
def COR_detection():
    global COR_res
    cap = init_cam()
    while True:
        LR_res = capture_res(cap,True)
        COR_res = LR_res
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
        if (stop_thread):
            cap.release()
            break

# In[12]:
def listener():
    global stop_thread
    global COR_res
    while True:
        if stop_thread:
            break
        if (COR_res == 0):
            print("center")
        elif COR_res == 1:
            print("deviation to right")
        elif COR_res == 2:
            print("deviation to left")



if __name__ == "__main__":
    p1 = threading.Thread(target=COR_detection)
    p2 = threading.Thread(target=listener)
    p1.start()
    p2.start()

    try:
        while True:
            time.sleep(0.1)
            #print("waiting")
            
    except KeyboardInterrupt:
        stop_thread = True
        p1.join()
        p2.join()
        print("system stopped")
    
    
        



