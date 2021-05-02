Code tutorial for the rosbridge and for joystick was found here:
https://msadowski.github.io/ros-web-tutorial-pt1

github link for joystick gui code:
https://gist.github.com/msadowski/d5e276d9bc11a0846feda5ec0a5d8726#file-gui-html

The launch file, websocket.launch, needs to be configured to launch of the Raspberry Pi's public IP
Command for public IP: hostname -I

Then the html file that you want to display needs to be serviced. This can be done with a simple http server initiated in a python file. Make sure the port specified in the http server python file is different than the one specified in the websocket startup. The html file you want to service needs to be in the same directory or below this python file.
Insert code block here
Website that helped me figure this out: https://stackoverflow.com/questions/60967404/cant-establish-connection-to-web-server-using-rosbridge

finish commands


In terminal:
cd catkin_ws
source devel/setup.bash

roslaunch robot_gui_bridge websocket.launch

To launch for any computer on same network to access:
Add launching file Hanna made to connect from outside computer
10.214.152.67
To launch manual control page on the local display:
Go into file directory on Ubuntu 
click through files: home->catkin_ws->src->robot_gui_bridge->gui
click the gui.html file to launch the manual control page on the local displayhttps://msadowski.github.io/ros-web-tutorial-pt1/
