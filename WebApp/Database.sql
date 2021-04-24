-- Queries for database
-- Create table
DROP TABLE IF EXISTS users

CREATE TABLE users (
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

-- Populate table
-- NW
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.566213, -123.279874, 'Leon', 'Tran', 'tranleo@oregonstate.edu');
-- SW
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.565269, -123.279874, 'Wyatt', 'Deck', 'deckw@oregonstate.edu');
-- SE
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.566252, -123.277948, 'Ashley', 'Pettibone', 'pettiboa@oregonstate.edu');
-- NE
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.566215, -123.477897, 'Spencer', 'Bain', 'Bainspe@oregonstate.edu');
-- Middle
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.565728, -123.278919, 'Jacob', 'Gillette', '@oregonstate.edu');
-- East
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.565653, -123.277897, 'Qusai', 'Alawlaqi', 'alawlaqq@oregonstate.edu');
-- West
INSERT INTO myDatabase (latitude, longitude, Fname, Lname, email) VALUES (44.565772, -123.279890, 'Hanna', 'Anderson', 'gilletja@oregonstate.edu');
