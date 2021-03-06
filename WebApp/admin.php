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
  </head>

  <body>
    <header>
      <IMG SRC="logo.png" ALT="some text" WIDTH=100 HEIGHT=100>
      <h1><a href="/">Mobile Robot Package Delivery</a></h1>
    </header>

    <div class="bar">
      <h2 id="statusNotifcation">Status: Offline</h2>
      <button id="Login"><a href="/">Login</a></button>
      <button id="adminButton"><a href="/admin.html">Admin</a></button>
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

    <!-- Movement Buttons-->
    <div class="movement">
      <button id="up">Forward</button>
      </br>
      <button id="left">Left</button>
      <button id="right">Right</button>
      </br>
      <button id="down">Back</button>
      </br>
      <button id="stop">Stop</button>
    </div>

  </body>
</html>
