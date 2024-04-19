import cv2 as cv
import base64
import numpy as np 



#imag = base64.b64decode(input(), altchars = None, validate = False)

nparr = np.fromstring(base64.b64decode(input()), np.uint8)
image = cv.imdecode(nparr, cv.IMREAD_COLOR)





imageHSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)

low_hsv_green = (45, 50, 50)
high_hsv_green = (90, 255, 255)

green_hsv_mask = cv.inRange(imageHSV, low_hsv_green, high_hsv_green)

cnt, _ = cv.findContours(green_hsv_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

ans = 0

if cnt:
    for c in cnt:
        area = cv.contourArea(c)
        if abs(area) < 25:
            continue
        else:
            ans+=1
            
#cv.imshow(green_hsv_mask)
#cv.waitKey()
print(ans)
