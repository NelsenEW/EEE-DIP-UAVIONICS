import cv2
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys

url="http://192.168.1.1:80"
CAMERA_BUFFER_SIZE=4096
stream=urlopen(url + "/stream.jpg")
bts=b''
i=0

while True:    
    try:
        bts+=stream.read(CAMERA_BUFFER_SIZE)
        jpghead=bts.find(b'\xff\xd8')
        jpgend=bts.find(b'\xff\xd9')
        if jpghead>-1 and jpgend>-1:
            jpg=bts[jpghead:jpgend+2]
            bts=bts[jpgend+2:]
            img=cv2.imdecode(np.frombuffer(jpg,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
            img=cv2.resize(img,(640,480))
            cv2.imshow("ESP32 CAM OPENCV stream",img)
        k=cv2.waitKey(1)
    except Exception as e:
        print("Error:" + str(e))
        bts=b''
        stream=urlopen(url)
        continue
    # Press 'a' to take a picture
    if k & 0xFF == ord('a'):
        cv2.imwrite(str(i) + ".jpg", img)
        print(f"Save image filename: {i}.jpg")
        i=i+1
    # Press 'q' to quit
    if k & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()