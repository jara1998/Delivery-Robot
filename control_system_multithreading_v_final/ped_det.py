#!/usr/bin/env python
# coding: utf-8

# In[4]:


import io
import time
import cv2
import imutils

detect_res = False

def padestrain_detect(img_curr, show_res):
    global detect_res
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
    detect_res = False
    # Drawing the regions in the Image
    for (x, y, w, h) in regions:
        cv2.rectangle(image, (x, y), 
                      (x + w, y + h), 
                      (0, 0, 255), 2) 
        #需要测试 这里是arbitray value， w是人的宽度，h是高度
        if ((w > 40) & (h >140)):
            detect_res = True
            if (show_res):
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image,'people is coming',(10,20), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
            
    return image, detect_res


# In[2]:


def init_cam():
    frameWidth = 640
    frameHeight = 480
    cap = cv2.VideoCapture(0)
    cap.set(3,frameWidth)
    cap.set(4,frameHeight)
    return cap


# In[3]:


def capture_res(cap, show_res=True):
    success,img = cap.read()
    img, detect_res = padestrain_detect(img, show_res)
    if (show_res):
        cv2.imshow('result',img)
    return detect_res


# In[5]:


# demo
#cap = init_cam()
#while True:
    #capture_res(cap)
    #if cv2.waitKey(1)&0xFF == ord('q'):
        #break


# In[ ]:
if __name__ == "__main__":
    cap = init_cam()
    while True:
        detect_res = capture_res(cap, show_res=True)
        print(detect_res)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
        



