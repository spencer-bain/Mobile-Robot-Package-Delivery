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
    <script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/js/bootstrap.min.js" integrity="sha384-JZR6Spejh4U02d8jOt6vLEHfe/JQGiRRSQQxSfFWpi1MquVdAyjUar5+76PVCmYl" crossorigin="anonymous"></script>
    <link rel="stylesheet" href="style1.css">
  </head>

  <body>
    <header>
      <IMG SRC="logo.png" ALT="OSU logo" WIDTH=100 HEIGHT=100>
      <h1><a href="/">Mobile Robot Package Delivery</a></h1>
    </header>

    <div class="bar">
      <h2 id="statusNotifcation">Status: Offline</h2>
      <button id="Login"onclick="document.getElementById('id01').style.display='block'">Login</button>
      <button id="adminButton"><a href="/admin.php">Admin</a></button>
    </div>

    <!-- The Modal -->
    <div id="id01" class="modal">
      <span onclick="document.getElementById('id01').style.display='none'"
    class="close" title="Close Modal">&times;</span>

      <!-- Modal Content -->
      <form class="modal-content animate">
        <div class="login-container">
          <label for="email"><b>Email</b></label>
          <input type="text" id="loginEmail" placeholder="Enter email" name="email" required>

          <label for="psw"><b>Password</b></label>
          <input type="password" id="loginPwd" placeholder="Enter password" name="psw" required>

          <button type="submit">Login</button>
          <label>
            <input type="checkbox" checked="checked" name="remember"> Remember me
          </label>
        </div>

        <div class="cancel-container" style="background-color:#f1f1f1">
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

    <?php
    $i = 0;
    // Get email of users in database
    $sql = "SELECT * FROM users";
    $result = $db_conn->query($sql);

    // Display emails
    if ($result->num_rows > 0){
      while($row = $result->fetch_assoc()){
        $data[] = $row[email];
      }
      // for($i = 0; $i <= $result->num_rows; $i++){
      //   print_r($data[$i]);
      //   echo "<br>";
    } else {
      echo "No results";
    }
    ?>

    <script>
    function getStatus() {
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

    function getCoordFromLocation(location){
      if (location === "NW"){
        location = "44.566213,-123.279874";
        return location;
      }else if (location === "NE"){
        location = "44.566215,-123.477897";
        return location;
      }else if (location === "SE"){
        location = "44.566252,-123.277948";
        return location;
      }else if (location === "SW"){
        location = "44.565269,-123.279874";
        return location;
      }else{
        return NULL;
      }
    }

    function auth(){
      // Get email from input box
      var receiverEmail = document.getElementById("email").value

      // Get authorized emails from database
      var passedArray = <?php echo json_encode($data); ?>;

      // Check inputted email against database
      var flag = 0;
      for(var x = 0; x < passedArray.length; x++){
        if(receiverEmail == passedArray[x]){
          flag = 1;
        }
      }
      if(flag == 1){
        alert("Selection aurthorized");
        // Get inputs
        var receiverFname = document.getElementById("fname").value
        var receiveLname = document.getElementById("lname").value
        // Get location inputs
        var senderLocation = document.getElementById("SenderLocations").value
        var receiverLocation = document.getElementById("ReceiverLocation").value
        // Convert locations to coordinates (lat, lng)
        senderLocation = getCoordFromLocation(senderLocation);
        receiverLocation = getCoordFromLocation(receiverLocation);
        // Get coordinates to global planner

      } else{
        alert("Not authorized");
      }
    }
    </script>

    <script type="text/text/javascript" type"text/javascript">
      // Connect to ROS for global planner
      var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
      });

      ros.on('connection', function() {
         console.log('Connected to websocket server.');
      });

       ros.on('error', function(error) {
         console.log('Error connecting to websocket server: ', error);
      });

       ros.on('close', function() {
         console.log('Connection to websocket server closed.');
       });

       // Publish topic
       var cmdVel = new ROSLIB.Topic({
         ros : ros,
         name : '/new_delivery'
         messageType : 'std_msgs/String'  // Work with Qusai
       });

       var String = new ROSLIB.Message({
         // Message should be current location (lineCoords[0])
         // Start location (senderLocation)
         // Receiver location (receiverLocation)
       })
    </script>

    <div class="sendAndReceive">
      <h1>Sender Selection</h1>
      <p>Please select your location</p>
      <label for="SenderLocations">Select a location:</label>
      <select name="SenderLocations" id="SenderLocations">
        <option value="NW">NW</option>
        <option value="SW">SW</option>
        <option value="SE">SE</option>
        <option value="NE">NE</option>
      </select>

      <h1>Receiver Selection</h1>
      <p>Please enter a receiver</p>
      <input type="text" id="fname" name="fname" placeholder="First Name">
      <input type="text" id="lname" name="lname" placeholder="Last Name">
      <input type="text" id="email" name="email" placeholder="Email">
      <label for="ReceiverLocation">Select a location:</label>
      <select name="SenderLocations" id="ReceiverLocation">
        <option value="NW">NW</option>
        <option value="SW">SW</option>
        <option value="SE">SE</option>
        <option value="NE">NE</option>
      </select>
      <input type="submit" id="submitBtn"  onclick="auth()" value="Submit">
    </div>



    <h1 id="cameraText">Live camera feed</h1>

    <!-- Add a placeholder for the Twitch embed -->
    <div id="twitch-embed"></div>

    <!-- Load the Twitch embed script -->
    <script src="https://embed.twitch.tv/embed/v1.js"></script>

    <div class="iframe-container">
      <iframe
        src="https://player.twitch.tv/?channel=osu_mobile_delivery_robot&parent=localhost"
        height="480"
        width="854"
        class="frame">
      </iframe>
    </div>

    <div class="gps">
      <div class="container">
        <h1>Realtime GPS Tracking</h1>
          <button class="btn btn-success" id="action">Start Tracking</button><br>
          <div id="map-canvas"></div>
      </div>
    </div>

    <script>
    // Map starting coordinates
    window.lat = 44.565702;
    window.lng = -123.278893;

    var map;
    var mark;
    var lineCoords = [];

    // Create the first map with a marker
    var initialize = function()
    {
      map = new google.maps.Map(document.getElementById('map-canvas'), {center:{lat:lat,lng:lng},zoom:12});
      mark = new google.maps.Marker({position:{lat:lat, lng:lng}, map:map});
    };

    window.initialize = initialize;

    // Resizing
    // google.maps.addDomListener(window, resize, initialize);

    // Function to update the map
    var redraw = function(payload)
    {
      if(payload.message.lat)
      {
        // Coordinates pulled from GPS module
        lat = payload.message.lat;
        lng = payload.message.lng;

        // Follow the new location on the map
        map.setCenter({lat:lat, lng:lng, alt:0});
        mark.setPosition({lat:lat, lng:lng, alt:0});

        // Push to Google Maps API
        lineCoords.push(new google.maps.LatLng(lat, lng));

        // Create the tracing line
        var lineCoordinatesPath = new google.maps.Polyline({
          path: lineCoords,
          geodesic: true,
          strokeColor: '#24fff8'
        });

        lineCoordinatesPath.setMap(map);
      }
    };

    // Connect to pubnub
    var pnChannel = "raspi-tracker";

    var pubnub = new PubNub({
      publishKey: 'pub-c-74e6b463-6ec6-4cea-9eb6-9f692a6506a0',
      subscribeKey: 'sub-c-efdc1f2e-2aa6-11eb-9713-12bae088af96'
    });

    // Listen for "Start Tracking" button click
    document.querySelector('#action').addEventListener('click', function(){
      var text = document.getElementById("action").textContent;
      if(text == "Start Tracking"){
        // Connect to pubnub and update coordinates
        pubnub.subscribe({channels: [pnChannel]});
        pubnub.addListener({message:redraw});
        document.getElementById("action").classList.add('btn-danger');
        document.getElementById("action").classList.remove('btn-success');
        // Change "Start Tracking" to "Stop Tracking"
        document.getElementById("action").textContent = 'Stop Tracking';
      }
      else{
        // Disconnect from pubnub
        pubnub.unsubscribe({channels: [pnChannel]});
        document.getElementById("action").classList.remove('btn-danger');
        document.getElementById("action").classList.add('btn-success');
        document.getElementById("action").textContent = 'Start Tracking';
      }
    });
    </script>
    <script src="https://maps.googleapis.com/maps/api/js?v=3.exp&key=AIzaSyA3ZuPyZ58gTWHJuVpyj9ZWGOiZHNfQa1w&callback=initialize">
    </script>
  </body>
</html>
