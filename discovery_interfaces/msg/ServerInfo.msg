# The name of the entity advertising this discovery server.
# This information is only for debugging purposes.
string name

# The discovery server id. Must be unique among all servers.
uint8 id 

# The IP address (or host name) and port of the discovery server. 
# Each discovery server must have a unique address:port pair.
string address
uint16 port
