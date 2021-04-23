## GIPP
Generic Interval Path Planning  

## Introduction
This work is based on the SIPP (Safe Interval Path Planning) method, in which each
location on the map is split into a sequence of safe/collision intervals along the timeline.  
  
A safe interval is a continuous period of time for a location, during which there is no
collision and it is in collision one timestep prior and after the period; whereas a collision
interval is the opposite of a safe interval. Therefore, according to this definition, a safe
interval should be bounded by collision intervals or infinity, so it is impossible for a robot
to wait in place from one safe interval to another, because by doing so it will inevitably
cross at least one collision interval.  
  
The GIPP drops this limitation, by allowing a robot to cross collision intervals, with
additional costs. It also introduces different costs (both temporal and spatial) for a robot
to move between locations, which is set to 1 per grid for all time in the SIPP method.  
  
Therefore, the GIPP method can be considered as a ‘superset’ to SIPP. According to the
description, there is no need to know the location of obstacles for GIPP, the input will be
a set of ‘arrows’, which represents the direction and value of costs.  

**Please read GIPP.pdf for more details and run MAIN.py for examples**
