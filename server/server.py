import requests
import cv2
import numpy as np
import scipy.signal


IOT_MODE = False

# def hex2int(hex: str) -> int:
#     res = 0
#     for c in hex:
#         res = (res << 4) | ({"0": 0, "1": 1, "2": 2, "3": 3, "4":4, "5": 5, "6": 6, "7": 7, "8": 8, "9": 9, "A": 10, "B": 11, "C": 12, "D":13, "E":14, "F":15}[c])
#     return res


def get_motion(prev, curr):
    diff = cv2.absdiff(prev, curr)
    _, mask = cv2.threshold(diff, 50, 1, cv2.THRESH_BINARY)

    mask = scipy.signal.convolve2d(mask, np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]], dtype=np.uint8), mode="same")
    _, mask = cv2.threshold(mask, 7, 1, cv2.THRESH_BINARY)
    return mask * 255


if IOT_MODE:
    vc = cv2.VideoCapture(0)
else:
    vc = None
previous_image = None

while True:
    if IOT_MODE:
        res = requests.get("http://192.168.122.100/").text.encode()
        # print(res[:100])
        # with open("f.bmp", "wb") as f:
        #     f.write(res)
        image = cv2.imdecode(np.frombuffer(res, np.uint8), cv2.IMREAD_GRAYSCALE)
    else:
        success, image = vc.read()
        if not success:
            continue
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if previous_image is not None:
        cv2.imshow("Out", get_motion(previous_image, image))
    previous_image = image
    cv2.waitKey(1)
    