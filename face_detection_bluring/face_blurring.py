# Python program detects faces in a live video
# The program then saves the video under the name face_detector in avi format (.avi)
# to compile in terminal type "python face_blurring.py"

#import openCV into project
import cv2

#import time into project to sync framerate
import time
#import subprocess into project to create and manage stream pipe
import subprocess as sp

# This will return a live video from the webcam on your computer.
video = cv2.VideoCapture(0)

# We need to check if camera opened
# if it is not, program sends error message
if not video.isOpened():
    print("Error opening camera")
    exit()


# need to set the resolutions
# .get(3) and .get(4) gives 640x480 by default
frame_width = int(video.get(3))
frame_height = int(video.get(4))
size = (frame_width, frame_height)

##What the maximum fps can be
TARGET_FPS = 5

##stream url including stream key
rtmp_url = "rtmp://live.justin.tv/app/live_670152613_4f0j8ldoKHml5nQJ7fRSG3of5mkWeg"

##Command for stream pipe
command = ["ffmpeg",
	   "-y",
	   "-f", "rawvideo",
	   "-vcodec", "rawvideo",
	   "-pix_fmt", "bgr24",
	   "-s", "{}x{}".format(frame_width, frame_height),
	   "-r", str(TARGET_FPS),
	   "-i", "-",
	   "-c:v", "libx264",
	   "-pix_fmt", "yuv420p",
	   "-preset", "ultrafast",
	   "-f", "flv",
	   rtmp_url]
##Open pipe
proc = sp.Popen(command, stdin=sp.PIPE)

##Start frame rate tracker
frame_start = time.time()

#Main loop
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
    minNeighbors = 6

    # minSize allows you to define the minimum possible object size measured in pixels.
    # Objects smaller than this parameter are ignored.
    # use the face_cascade object to detect faces in the Image
    faces = face_cascade.detectMultiScale(gray, scaleFactor, minNeighbors, minSize = (20, 20))

    # rectangle will use these to locate and draw rectangles around the detected objects in the input image/video.
    for (x, y, w, h) in faces:
        # blurred faces
        # Select the detected face area
        face_color = frame[y:y + h, x:x + w]
        # Blur the detected face by applying a Gaussian Blur
        blur = cv2.GaussianBlur(face_color, (51, 51), 0)
        frame[y:y + h, x:x + w] = blur

    # if there is a frame read in
    if ret:

        # Display the updated frame as a video stream
        ## I don't think we need this past testing
        cv2.imshow('Frame', frame)

        # Write out to pipe
        proc.stdin.write(frame.tobytes())

        # Press the ESC key to exit the loop
        # 27 is the code for the ESC key
        if cv2.waitKey(1) == 27:
            break

    # Break the loop
    else:
        break

    # Wait to preserve frame rate (Might be able to change to sleep or something)
    while time.time() - frame_start < 1 / TARGET_FPS:
        pass
    frame_start = time.time()

# When everything done, release the video capture and video write objects
video.release()
result.release()

# Destroy the window that was showing the video stream
cv2.destroyAllWindows()
