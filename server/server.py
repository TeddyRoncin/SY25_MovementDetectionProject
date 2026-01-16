import requests
import cv2
import numpy as np
import socket
import scipy.signal
from typing import override
import selenium
import selenium.webdriver
from selenium.webdriver.common.by import By
import base64
import time

# IP, PORT = "192.168.122.100", 80
IP, PORT = "127.0.0.1", 8080
URL = f"http://{IP}" + (f":{PORT}" if PORT != 80 else "")


def get_motion(prev, curr):
    diff = cv2.absdiff(prev, curr)
    _, mask = cv2.threshold(diff, 50, 1, cv2.THRESH_BINARY)

    mask = scipy.signal.convolve2d(mask, np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]], dtype=np.uint8), mode="same")
    _, mask = cv2.threshold(mask, 7, 1, cv2.THRESH_BINARY)
    return mask * 255

class ImageGrabber:
    def grab(self):
        raise NotImplemented(f"{self.__class__} should implement grab")
    
class VideoCaptureImageGrabber(ImageGrabber):
    def __init__(self):
        self.vc = cv2.VideoCapture(URL)
    @override
    def grab(self):
        success, image = self.vc.read()
        if not success:
            return None
        return image
    
class SeleniumImageGrabber(ImageGrabber):
    def __init__(self):
        chrome_options = selenium.webdriver.ChromeOptions()
        chrome_options.add_argument("--headless")
        self.driver = selenium.webdriver.Chrome(chrome_options)
    @override
    def grab(self):
        self.driver.get(URL)
        # self.driver.get("file:///home/teddy/Downloads/c357face2de95f03e31c27cecec2ef63.jpg")
        image = self.driver.find_element(By.CSS_SELECTOR, "img")
        png_bytes = image.screenshot_as_png
        image = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
        image = cv2.flip(image, -1)
        image = cv2.resize(image, (960, 720))
        return image
    def __del__(self):
        self.driver.quit()
    
class RequestsImageGrabber(ImageGrabber):
    @override
    def grab(self):
        res = requests.get(URL).text.encode()
        # with open("f.bmp", "wb") as f:
        #     f.write(res)
        image = cv2.imdecode(np.frombuffer(res, np.uint8), cv2.IMREAD_GRAYSCALE)
        return image
    
class SocketImageGrabber(ImageGrabber):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((IP, PORT))
    @override
    def grab(self):
        self.sock.send(b"blabla")
        res = b""
        fully_received_header = False
        while len(res) < 320 * 240 + 1078:
            res += self.sock.recv(1024)
            if not fully_received_header:
                end_of_header = res.find(b"\n\n")
                if end_of_header != -1:
                    res = res[end_of_header + 2 :]
                    fully_received_header = True
        image = cv2.imdecode(np.frombuffer(res, np.uint8), cv2.IMREAD_GRAYSCALE)
        image = cv2.flip(image, -1)
        image = cv2.resize(image, (960, 720))
        return image

class CameraImageGrabber(ImageGrabber):
    def __init__(self):
        self.vc = cv2.VideoCapture(0)
    @override
    def grab(self):
        success, image = self.vc.read()
        if not success:
            return None
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image


ig = SocketImageGrabber()
previous_image = None

last_time = 0

while True:
    image = ig.grab()
    # cv2.imshow("hey ?", image)
    if previous_image is not None:
        cv2.imshow("Out", get_motion(previous_image, image))
    previous_image = image
    cv2.waitKey(1)
    new_time = time.time()
    print(f"FPS : {1 / (new_time - last_time):.1f}")
    last_time = new_time
    