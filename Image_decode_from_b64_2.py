import base64
import numpy as np
import cv2

# Входные данные представляют собой изображение в формте JPEG,
# закодированное в формат Base64.

# Получаем входные данные в формате Base64.
b64_jpeg = input()

# Декодируем данные, получаем изображение в формте JPEG.
raw_jpeg = base64.b64decode(b64_jpeg)

# Декодируем изображение для дальнейшей работы в OpenCV.
nparr = np.frombuffer(raw_jpeg, dtype=np.uint8)
image = cv2.imdecode(nparr, flags=1)

# Далее можно обрабатывать изображение, которое
imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

low_hsv_green = (45, 50, 50)
high_hsv_green = (90, 255, 255)

green_hsv_mask = cv2.inRange(imageHSV, low_hsv_green, high_hsv_green)

cnt, _ = cv2.findContours(green_hsv_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

ans = 0

if cnt:
    for c in cnt:
        area = cv2.contourArea(c)
        if abs(area) < 25:
            continue
        else:
            ans+=1
            
#cv.imshow(green_hsv_mask)
#cv.waitKey()
print(ans)
