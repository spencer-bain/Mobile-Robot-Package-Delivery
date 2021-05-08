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
    <script type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
    <script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.7.3/nipplejs.js"></script>
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
    
    var path_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/new_path',
        messageType: 'gp_test/Path'
    });

    path_listener.subscribe(function (m) {
        // var pnchannel = "raspi-tracker";
        // pubnub.subscribe({channels: [pnchannel]});
        // pubnub.addListener({message: redraw});
        console.log("DRAW START");
        var new_path = [];
        for (var i = 0; i < m.coordinates.length; i++) {
            var np = {lat: m.coordinates[i].lat, lng: m.coordinates[i].long}
            // pubnub.publish({channel:pnchannel, np});
            new_path.push(new google.maps.LatLng(np.lat, np.lng));
        }
        var lineCoordinatesPath = new google.maps.Polyline({
            path: new_path,
            geodesic: true,
            strokeColor: '#24fff8'
        });
        lineCoordinatesPath.setMap(map);
        console.log("DRAW END");
    });
      
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
        location = "44.56620989854069,-123.27986681438706";
        return location;
      }else if (location === "NE"){
        location = "44.56621596508787,-123.27788935771436";
        return location;
      }else if (location === "SE"){
        location = "44.565257994343455,-123.27790160582192";
        return location;
      }else if (location === "SW"){
        location = "44.56527081121918,-123.27986799604192";
        return location;
      }else if (location === "Mid"){
        location = "44.56573665661425,-123.27898745497953";
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
        // Publish topic
        var cmdVel = new ROSLIB.Topic({
          ros : ros,
          name : '/new_delivery',
          messageType : 'gp_test/DeliveryInfo' 
        });
       
        var start = senderLocation.split(',');
        var end = receiverLocation.split(',');
        //current_lat : lineCoords[0].getPosition().lat(),
        //current_long : lineCoords[0].getPosition().lng(),
        var msg = new ROSLIB.Message({
          current_lat : parseFloat(start[0]),
          current_long : parseFloat(start[1]),
          start_lat : parseFloat(start[0]), 
          start_long : parseFloat(start[1]),
          goal_lat : parseFloat(end[0]), 
          goal_long : parseFloat(end[1])
        })
       
        cmdVel.publish(msg);
       
      } else{
        alert("Not authorized");
      }
    }
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
        <option value="Mid">Mid</option>
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
        <option value="Mid">Mid</option>
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
    {map
      map = new google.maps.Map(document.getElementById('map-canvas'), {center:{lat:lat,lng:lng},zoom:18, mapTypeId: "satellite"});
      //mark = new google.maps.Marker({position:{lat:lat, lng:lng}, map:map});
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
