# Python program detects faces in a live video
# The program then saves the video under the name face_detector in avi format (.avi)
# to compile in terminal type "python face_detector.py"

#import openCV into project
import cv2


# To use a video file as input
# code is a little different
#video = cv2.VideoCapture("me.mp4")

# This will return a live video from the webcam on your computer.
video = cv2.VideoCapture(0)

# We need to check if camera opened
# if it did not, program sends error message
if not video.isOpened():
    print("Error reading video file")


# need to set the resolutions
# .get(3) and .get(4) gives 640x480 by default
frame_width = int(video.get(3))
frame_height = int(video.get(4))

size = (frame_width, frame_height)


# VideoWriter object will create a frame of above defined image
# The output is stored in 'face_detector_output_file.avi' file.
result = cv2.VideoWriter('face_detector_output_file.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'), 10, size)

while (True):

    # read in the video and capture frame-by-frame
    # ret checks return at each frame
    ret, frame = video.read()

    # Color has a bunch of extra data, so to make it easier, convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Load the cascade for face training data
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # 1.3-means that it can scale 30% down to try and match the faces better.
    scaleFactor = 1.3
    # specifies how many neighbors, or detections, each candidate rectangle should have to retain it
    minNeighbors = 10

    # minSize allows you to define the minimum possible object size measured in pixels.
    # Objects smaller than this parameter are ignored.
    # use the face_cascade object to detect faces in the Image
    faces = face_cascade.detectMultiScale(gray, scaleFactor, minNeighbors, minSize = (20, 20))

    # rectangle will use these to locate and draw rectangles around the detected objects in the input image/video.
    for (x, y, w, h) in faces:

        # changes the color of the line drawn
        # BGR- cuz that's default in OpenCV
        lineColor = (255, 0, 0)
        # thickness of the line
        lineThickness = 2
        # (x, y), (x + w, y + h) are the four pixel locations for the detected face(s).
        # function to draw the rectangles where a face was detected
        cv2.rectangle(frame, (x, y), (x + w, y + h), lineColor, lineThickness)

        #     # prints locations of faces
        #     print("detected faces in these pixel locations x:", x,"y:", y, "x+w:", x+w, "y+h+:", y+h)

    # if there is a frame read in
    if ret == True:

        # Write the frame into the file face_detector_output_file.avi'
        result.write(frame)

        # Display the updated frame as a video stream
        cv2.imshow('Frame', frame)

        # Press the ESC key to exit the loop
        # 27 is the code for the ESC key
        if cv2.waitKey(1) == 27:
            break

    # Break the loop
    else:
        break

# When everything done, release the video capture and video write objects
video.release()
result.release()

# Destroy the window that was showing the video stream
cv2.destroyAllWindows()

#print succes message to console
print("The video was successfully saved")