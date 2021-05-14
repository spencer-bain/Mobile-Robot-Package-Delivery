<!DOCTYPE html>
<html lang="en">
  <head>
    <?php
    // Connect to database
    try{
      require_once 'dbConn.php';
    } catch (Exception $ex) {
      $error = $ex->getMessage();
    }
    ?>
    <title>MRPD Homepage</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width">
    <script src="https://cdn.pubnub.com/sdk/javascript/pubnub.4.19.0.min.js"></script>
    <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/css/bootstrap.min.css" integrity="sha384-Gn5384xqQ1aoWXA+058RXPxPg6fy4IWvTNh0E263XmFcJlSAwiGgFAW/dAiS6JXm" crossorigin="anonymous">
    <link rel="stylesheet" href="style1.css">
    <script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/js/bootstrap.min.js" integrity="sha384-JZR6Spejh4U02d8jOt6vLEHfe/JQGiRRSQQxSfFWpi1MquVdAyjUar5+76PVCmYl" crossorigin="anonymous"></script>
    <script type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
    <script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.7.3/nipplejs.js"></script>
  </head>

  <body>
    <header>
      <IMG SRC="logo.png" ALT="some text" WIDTH=100 HEIGHT=100>
      <h1><a href="/">Mobile Robot Package Delivery</a></h1>
    </header>

    <!-- Joystick and ROS stuff -->
    <script type="text/javascript" type="text/javascript">
      var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
      });

      ros.on('connection', function () {
        document.getElementById("status").innerHTML = "Connected";
      });

      ros.on('error', function (error) {
        document.getElementById("status").innerHTML = "Error";
      });

      ros.on('close', function () {
        document.getElementById("status").innerHTML = "Closed";
      });

      var txt_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/txt_msg',
        messageType: 'std_msgs/String'
      });

      txt_listener.subscribe(function (m) {
        document.getElementById("msg").innerHTML = m.data;
        move(1, 0);
      });

      cmd_vel_listener = new ROSLIB.Topic({
        ros: ros,
        name: "/cmd_vel",
        messageType: 'geometry_msgs/Twist'
      });

      move = function (linear, angular) {
        var twist = new ROSLIB.Message({
          linear: {
            x: linear,
            y: 0,
            z: 0
          },
          angular: {
            x: 0,
            y: 0,
            z: angular
          }
        });
        cmd_vel_listener.publish(twist);
      }

      createJoystick = function () {
        var options = {
          zone: document.getElementById('zone_joystick'),
          threshold: 0.1,
          position: { left: 50 + '%' },
          mode: 'static',
          size: 150,
          color: '#000000',
        };
        manager = nipplejs.create(options);

        linear_speed = 0;
        angular_speed = 0;

        self.manager.on('start', function (event, nipple) {
          console.log("Movement start");
        timer = setInterval(function() {
          move(linear_speed, angular_speed);
        }, 25);
        });

        self.manager.on('move', function (event, nipple) {
          console.log("Moving");
        max_linear = 5.0;
        max_angular = 2.0;
        max_distance = 75.0;
        linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
        angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
        });

        self.manager.on('end', function () {
          console.log("Movement end");
        if (timer) {
          clearInterval(timer);
        }
        self.move(0, 0);
        });
      }

      window.onload = function () {
        createJoystick();
      }

    </script>

    <div class="bar">
      <h2 id="statusNotifcation">Status: Offline</h2>
      <button id="Login"><a href="/">Login</a></button>
      <button id="adminButton"><a href="/admin.php">Admin</a></button>
      <button id="updateStatus" onclick="updateStatus()">Update Status</button>
    </div>

    <!-- The Modal -->
    <div id="id01" class="modal">
      <span onclick="document.getElementById('id01').style.display='none'"
    class="close" title="Close Modal">&times;</span>

      <!-- Modal Content -->
      <form class="modal-content animate">
        <div class="login-container">

          <label for="email"><b>Email</b></label>
          <input type="text" placeholder="Enter email" name="email" required>

          <label for="psw"><b>Password</b></label>
          <input type="password" placeholder="Enter password" name="psw" required>

          <button type="submit">Login</button>
          <label>
            <input type="checkbox" checked="checked" name="remember"> Remember me
          </label>
        </div>

        <div class="login-container" style="background-color:#f1f1f1">
          <button type="button" onclick="document.getElementById('id01').style.display='none'" class="cancelbtn">Cancel</button>
          <span class="psw">Forgot <a href="#">password?</a></span>
        </div>
      </form>
    </div>

    <script>
    // Get the modal
    var modal = document.getElementById('id01');

    // When the user clicks anywhere outside of the modal, close it
    window.onclick = function(event) {
      if (event.target == modal) {
        modal.style.display = "none";
      }
    }
    </script>

    <script>
    function updateStatus() {
      // 0 = Ready for Delivery
      // 1 = On Delivery
      // 2 = Delivery Complete

      var status;
      // Testing
      var currentPosition = [1, 2];
      var endPosition = [1, 1];
      var lastPosition = [1, 2];

      // Get current and end position

      // If robot has not moved since last check
      if(currentPosition[0] == endPosition[0] && currentPosition[1] == endPosition[1]){
        status = 2;
        document.getElementById("statusNotifcation").innerHTML = "Status: Delivery Complete"
        document.getElementById("statusNotifcation").style.color = "Yellow";
        return status;
      }
      if(currentPosition[0] == lastPosition[0] && currentPosition[1] == lastPosition[1]){
        status = 0;
        document.getElementById("statusNotifcation").innerHTML = "Status: Ready for Delivery"
        document.getElementById("statusNotifcation").style.color = "Green";
        return status;
      }
      if(currentPosition[0] != lastPosition[0] || currentPosition[1] != lastPosition[1]){
        status = 1;
        document.getElementById("statusNotifcation").innerHTML = "Status: On Delivery"
        document.getElementById("statusNotifcation").style.color = "Red";
        return status;
      }
      // Update lastPosition to currentPosition
    }
    </script>

    <!-- Show database table of users -->
    <?php
    $i = 0;
    // Get email of users in database
    $sql = "SELECT * FROM users";
    $result = $db_conn->query($sql);

    // Display emails
    if ($result->num_rows > 0){
      // If table is not empty, create table
      echo "<table border = '1'>
      <tr>
      <th>First Name</th>
      <th>Last Name</th>
      <th>Email</th>
      <th>Latitude</th>
      <th>Longitude</th>
      </tr>";

      // Populate table
      while($row = $result->fetch_assoc()){
        echo "<tr>";
        echo "<td>" . $row['Fname'] . "</td>";
        echo "<td>" . $row['Lname'] . "</td>";
        echo "<td>" . $row['email'] . "</td>";
        echo "<td>" . $row['latitude'] . "</td>";
        echo "<td>" . $row['longitude'] . "</td>";
        echo "</tr>";
      }
      echo "</table>";
    } else {
      echo "No results";
    }
    mysqli_close($db_conn);
    ?>

    <!-- IMU table -->
    <div id="imu_msg">
    <br>
      <button id="imu_button">Get data</button><br>
      <table border = '1'>
        <tr>
          <th></th>
          <th>mag</th>
          <th>ang</th>
          <th>linear</th>
          <th>ori</th>
        </tr>
        <tr>
          <th>X</th>
          <td id="a1">mag_x</td>
          <td id="a2">ang_x</td>
          <td id="a3">linear_x</td>
          <td id="a4">ori_x</td>
        </tr>
        <tr>
          <th>Y</th>
          <td id="b1">mag_y</td>
          <td id="b2">ang_y</td>
          <td id="b3">linear_x</td>
          <td id="b4">linear_x</td>
        </tr>
        <tr>
          <th>Z</th>
          <td id="c1">mag_z</td>
          <td id="c2">ang_z</td>
          <td id="c3">linear_z</td>
          <td id="c4">ori_z</td>
        </tr>
        <tr>
          <th>W</th>
          <td></td>
          <td></td>
          <td></td>
          <td id="d4">ori_w</td>
        </tr>
      </table>
    </div>

    <script>
    // Connect to pubnub
    var pnChannel = "imu_data";
    var pubnub = new PubNub({
      publishKey: 'pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0',
      subscribeKey: 'sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96'
    });

    // Get button text
    document.querySelector('#imu_btn').addEventListener('click', function(){
      var text = document.getElementById("imu_button").textContent;
      if (text == "Get IMU Data"){
        pubnub.subscribe({channels: [pnChannel]});
        pubnub.addListener({message:update_imu});
        document.getElementById("imu_btn").classList.add('btn-danger');
        document.getElementById("imu_btn").classList.remove('btn-success');
        // Change "Start Tracking" to "Stop Tracking"
        document.getElementById("imu_btn").textContent = 'Stop IMU data';
      }else{
        // Disconnect from pubnub
        pubnub.unsubscribe({channels: [pnChannel]});
        document.getElementById("imu_btn").classList.remove('btn-danger');
        document.getElementById("imu_btn").classList.add('btn-success');
        document.getElementById("imu_btn").textContent = 'Get IMU Data';
      }
    });

    var update_imu = function update(payload){
    // Get data from pubnub
    mag_x = payload.message.mag_x;
    mag_y = payload.message.mag_y;
    mag_z = payload.message.mag_z;

    ang_x = payload.message.ang_x;
    ang_y = payload.message.ang_x;
    ang_z = payload.message.ang_x;

    linear_x = payload.message.linear_x;
    linear_y = payload.message.linear_y;
    linear_z = payload.message.linear_z;

    ori_x = payload.message.ori_x;
    ori_y = payload.message.ori_y;
    ori_z = payload.message.ori_z;
    ori_w = payload.message.ori_w;

    // Update table with new data
    document.getElementById("a1").innerHTML = mag_x;
    document.getElementById("a2").innerHTML = ang_x;
    document.getElementById("a3").innerHTML = linear_x;
    document.getElementById("a4").innerHTML = ori_x;

    document.getElementById("b1").innerHTML = mag_y;
    document.getElementById("b2").innerHTML = ang_y;
    document.getElementById("b3").innerHTML = linear_y;
    document.getElementById("b4").innerHTML = ori_y;

    document.getElementById("c1").innerHTML = mag_z;
    document.getElementById("c2").innerHTML = ang_z;
    document.getElementById("c3").innerHTML = linear_z;
    document.getElementById("c4").innerHTML = ori_z;

    document.getElementById("d4").innerHTML = ori_w;
    };
    </script>

    <!-- Table with GPS data -->
    <div id="gps_msg">
    <br>
      <button id="gps_btn">Get GPS data</button><br>
      <table border = '1'>
        <tr>
          <th>Latitude</th>
          <th>Longitude</th>
        </tr>
        <td id="lat1">lat</td>
        <td id="lng1">lng</td>
      </table>
    </div>

    <script>
    var update_gps = function(payload){
      if(payload.message.lat){
        lat = payload.message.lat;
        lng = payload.message.lng;

        document.getElementById("lat1").innerHTML = lat;
        document.getElementById("lng1").innerHTML = lng;
      }
    };

    // Connect to pubnub
    var pnChannel = "raspi-tracker";
    var pubnub = new PubNub({
      publishKey: 'pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0',
      subscribeKey: 'sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96'
    });

    // Listen for "Start Tracking" button click
    document.querySelector('#gps_btn').addEventListener('click', function(){
      var text = document.getElementById("gps_btn").textContent;
      if(text == "Get GPS data"){
        // Connect to pubnub and update coordinates
        pubnub.subscribe({channels: [pnChannel]});
        pubnub.addListener({message:update_gps});
        document.getElementById("gps_btn").classList.add('btn-danger');
        document.getElementById("gps_btn").classList.remove('btn-success');
        // Change "Start Tracking" to "Stop Tracking"
        document.getElementById("gps_btn").textContent = 'Stop Tracking';
      }
      else{
        // Disconnect from pubnub
        pubnub.unsubscribe({channels: [pnChannel]});
        document.getElementById("gps_btn").classList.remove('btn-danger');
        document.getElementById("gps_btn").classList.add('btn-success');
        document.getElementById("gps_btn").textContent = 'Get GPS data';
      }
    });
    </script>
  </body>
</html>
