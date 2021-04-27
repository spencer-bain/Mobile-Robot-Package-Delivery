DROP TABLE IF EXISTS Connections;
DROP TABLE IF EXISTS Nodes;

CREATE TABLE Nodes (
    nodeID int AUTO_INCREMENT UNIQUE,
    gpsLat FLOAT NOT NULL,
    gpsLong FLOAT NOT NULL,
    PRIMARY KEY(nodeID)
);

CREATE TABLE Connections (
    node1ID int NOT NULL,
    node2ID int NOT NULL,
    FOREIGN KEY (node1ID) REFERENCES Nodes(nodeID),
    FOREIGN KEY (node2ID) REFERENCES Nodes(nodeID)
);

INSERT INTO Nodes (gpsLat, gpsLong)
VALUES 
(44.56620989854069, -123.27986681438706),
(44.56620979615749, -123.2789765278698),
(44.56620636897372, -123.278864924565),
(44.56621596508787, -123.27788935771436),
(44.56565334777891, -123.27789884501831),
(44.565257994343455, -123.27790160582192),
(44.565259961282585, -123.2779913319387),
(44.56536322549297, -123.27819149019928),
(44.56525406046582, -123.27819563140777),
(44.56525602740414, -123.27887064790019),
(44.56525701087373, -123.27897831924034),
(44.56526195999426, -123.2796512729598),
(44.56527081121918, -123.27986799604192),
(44.56537112500739, -123.27965817496879),
(44.5661501824789, -123.27986279107361),
(44.56608998862439, -123.27982658125076),
(44.56577163910244, -123.2798173616191),
(44.565765907778925, -123.27939240705523),
(44.56540553872057, -123.27986209280752),
(44.56566054873755, -123.2789738005122),
(44.565654254096664, -123.27887179360934),
(44.56569545537003, -123.27898745497953),
(44.56569774432883, -123.27885813914203),
(44.56573665661425, -123.27898745497953),
(44.56572692854535, -123.27886376156974),
(44.56565949641154, -123.27818055471234),
(44.56576866068274, -123.27898532897126),
(44.56575980953358, -123.27886385361313),
(44.56581291640834, -123.27897980736405),
(44.56581685024898, -123.27887213602388);

INSERT INTO Connections (node1ID, node2ID)
VALUES 
(1, 2),
(2, 3),
(3, 4),
(4, 5),
(5, 6),
(6, 7),
(7, 8),
(7, 9),
(8, 9),
(9, 10),
(10, 11),
(11, 12),
(12, 13),
(12, 14),
(13, 14),
(1, 15),
(15, 16),
(16, 17),
(17, 18),
(13, 19),
(11, 20),
(10, 21),
(20, 21),
(14, 22),
(20, 22),
(21, 22),
(20, 23),
(21, 23),
(22, 23),
(18, 24),
(22, 24),
(23, 24),
(22, 25),
(23, 25),
(24, 25),
(5, 26),
(25, 26),
(1, 27),
(24, 27),
(25, 27),
(4, 28),
(27, 28),
(2, 29),
(27, 29),
(28, 29),
(3, 30),
(27, 30),
(28, 30),
(29, 30);