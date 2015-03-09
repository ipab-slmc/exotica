#first of all, the name
string name

#indicate if this vertex is active or not
bool isActive

#the position
geometry_msgs/Point position

#Radius
float64 radius

#types to other vertices
int32 type

#if not a link, what link(s) it is interacting with
string[] toLinks

#Weighting factor
float64 w

#type definitions
uint8 LINK=0
uint8 GOAL=10
uint8 OBSTACLE=20
uint8 OBSTACLE_TO_ALL=30
uint8 IGNORE=40
uint8 NONE=255
