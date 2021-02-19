#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import sys
import numpy as np


# Output Image to display
img = cv2.imread("img_0.png")

output = img[:, 0:640]

cv2.imshow('image',output)

cv2.waitKey(0)
