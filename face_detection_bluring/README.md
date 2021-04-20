# Face-detecting-and-Blurring


## For face detecting:

This program is done in Python, developed on Pycharm and uses the OpenCV library

This Python program detects faces from a live video and draws a blue rectangle around them
The program then saves the video under the name face_detector_output_file in avi format (.avi)

To run the program you must have OpenCV installed on your machine 
OpenCV is an open source computer vision and machine learning software library

If it's not installed, you can type the below command in terminal to install
 "pip install opencv-python"

This is the link for face detection classifier file for faces:
https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
The above is just for reference, this file is included in this repository

To compile the program in the terminal type 
 "python face_detector.py"
 
 
## For face blurring:
 
This program is done in Python, developed on Pycharm and uses the OpenCV library

This Python program detects faces from a live video and blurs them
The program then saves the video under the name blur_faces_output_file in avi format (.avi)

To run the program you must have OpenCV installed on your machine 
OpenCV is an open source computer vision and machine learning software library

If it's not installed, you can type the below command in terminal to install
 "pip install opencv-python"

This is the link for face detection classifier file for faces:
https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
The above is just for reference, this file is included in this repository

To compile the program in the terminal type 
"python face_blurring.py"

## For streaming:
 
System need ffmpeg installed
"sudo apt update"
"sudo apt install ffmpeg"
