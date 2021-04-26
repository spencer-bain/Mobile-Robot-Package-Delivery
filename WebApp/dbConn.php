<?php
// To start webpage now:
//  php -S 127.0.0.1:8000
// Go to localhost:8000

$db_host = '127.0.0.1';
$db_user = "root";        // User of database ("Leon" on Hanna's Pi)
$db_password = "pwd";     // Password of database (Hanna's Pi password)
$db_name = "mydatabase";
$db_port = '3306';

// Create connection
$db_conn = new mysqli($db_host, $db_user, $db_password, $db_name, $db_port);

// Check connection
if ($db_conn->connect_error) {
  die("Connection failed: " . $db_conn->connect_error);
}
// echo "Connected successfully\n";
?>
