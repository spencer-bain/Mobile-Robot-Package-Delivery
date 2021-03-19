
-- Get Nodes
SELECT * FROM Nodes;

-- Get Connections
SELECT * FROM Connections;

-- Get Nodes connected to node
SELECT Nodes.nodeID, Nodes.gpsLat, Nodes.gpsLong FROM Nodes
JOIN Connections ON Nodes.nodeID = Connections.node2ID
WHERE 
Connections.node1ID = :nodeID;
SELECT Nodes.nodeID, Nodes.gpsLat, Nodes.gpsLong FROM Nodes
JOIN Connections ON Nodes.nodeID = Connections.node1ID
WHERE 
Connections.node2ID = :nodeID;