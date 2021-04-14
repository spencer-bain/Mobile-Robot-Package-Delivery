# Python program detects faces in a live video
# The program then saves the video under the name face_detector in avi format (.avi)
# to compile in terminal type "python face_blurring.py"

#import openCV into project
import cv2

#import time into project to sync framerate
import time
#import threading into project
import threading
#import heapq into project for sorting
import heapq
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
TARGET_FPS = 10

#stream url including stream key
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

##Constant to easily change number of threads used
NUM_PROCESS_THREADS = 4

##Variable for inputs to threads and lock
current_frames = [None] * NUM_PROCESS_THREADS
current_frames_lock = threading.Lock()

##Variables for status of system and lock
done = False
sudden_done = False
done_lock = threading.Lock()

##Variable for outputs of threads and lock
output_frames = [None] * NUM_PROCESS_THREADS
output_frames_lock = threading.Lock()


def driver_thread():
    """
    Function of driver thread
    """
    ##Gain access to global variables above
    global done
    global sudden_done

    ##Variables for tracker order of frames
    f_num = 1
    f_num_to_output = 1
    frame_buffer = []
    all_read = False

    ##Start frame rate tracker
    frame_start = time.time()

    while True:
        ## Press esc to stop
        if cv2.waitKey(1) == 27:
            done_lock.acquire()
            sudden_done = True
            done_lock.release()
            break

        ##Get frames for threads
        output_frames_lock.acquire()
        for i in range(len(output_frames)):
            if output_frames[i]:
                heapq.heappush(frame_buffer, output_frames[i])
                output_frames[i] = None
        output_frames_lock.release()

        ## If not at end of video stream (Fail safe)
        if not all_read:
            current_frames_lock.acquire()
            ##Look through thread inputs
            for i in range(len(current_frames)):
                ##If no input yet
                if not current_frames[i]:
                    # read in the video and capture frame-by-frame
                    # ret checks return at each frame
                    ret, frame = video.read()

                    ## If error during read (end of stream)
                    if not ret:
                        done_lock.acquire()
                        done = True
                        done_lock.release()
                        all_read = True
                        break

                    ##Send frame to thread
                    current_frames[i] = (f_num, frame)
                    f_num += 1
            current_frames_lock.release()

        ##Write frame if ready
        if frame_buffer:
            if frame_buffer[0][0] == f_num_to_output:
                frame = heapq.heappop(frame_buffer)
                # Display the updated frame as a video stream
                ## I don't think we need this past testing
                cv2.imshow('Frame', frame[1])
                # Write out to pipe
                proc.stdin.write(frame[1].tobytes())
                f_num_to_output += 1
        elif all_read:
            return

        ##Sync frame rate
        while time.time() - frame_start < 1 / TARGET_FPS:
            pass
        frame_start = time.time()

def process_thread(id):
    """
    Function of processing threads
    """
    ##Gain access to global variables above
    global done
    global sudden_done
    while True:
        #Stop immediately if sudden_done
        done_lock.acquire()
        if sudden_done:
            done_lock.release()
            return
        done_lock.release()

        ##Stop if done, continue if no frame to operate on
        current_frames_lock.acquire()
        if not current_frames[id]:
            current_frames_lock.release()
            done_lock.acquire()
            if done:
                done_lock.release()
                return
            done_lock.release()
            continue

        ##Read frame
        f_num, frame = current_frames[id]
        current_frames_lock.release()

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

        ##Send frame back to driver
        output_frames_lock.acquire()
        output_frames[id] = (f_num, frame)
        output_frames_lock.release()

        ##Mark ready for next frame
        current_frames_lock.acquire()
        current_frames[id] = None
        current_frames_lock.release()

##Make driver thread and start
driver_thread = threading.Thread(target=driver_thread)
driver_thread.start()

##Make process threads and start
p_threads = []
for i in range(NUM_PROCESS_THREADS):
    new_thread = threading.Thread(target=process_thread, args=(i, ))
    p_threads.append(new_thread)
    new_thread.start()

##Wait for driver thread
driver_thread.join()

##Wait for process threads
for t in p_threads:
    t.join()


# When everything done, release the video capture and video write objects
video.release()
result.release()

# Destroy the window that was showing the video stream
cv2.destroyAllWindows()

# print success message to console
print("The video was successfully saved")
