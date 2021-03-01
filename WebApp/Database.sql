-- Queries for database
-- Create table
DROP TABLE IF EXISTS class_receiver

CREATE TABLE class_receiver (
  latitude float NOT NULL,
  longitude float NOT NULL,
  Fname varchar(255) NOT NULL,
  Lname varchar(255) NOT NULL,
  email varchar(255) NOT NULL,
  PRIMARY KEY (email)
);

CREATE TABLE robot (
  latitude int NOT NULL,
  longitude int NOT NULL
);

-- Pull info about receiver
SELECT * FROM student WHERE student.email = ':senderEmail:';

-- Pull info about receiver
SELECT * FROM student WHERE student.email = ':receiverEmail:';

-- Update robot coordinates
INSERT INTO robot (longitude, latitude) VALUES (':robotLat:', ':robotLong:');

-- Get robot coordinates
SELECT * FROM robot;
