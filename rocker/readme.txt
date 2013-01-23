There are 3 functions includes in this folder:

evarocker_2d.m
evarocker_3d.m
tetra2.m

----------------------------------------------------------------------------------------------
evarocker_2d

The function evarocker_2d.m is straight forward. It is a 2D simulation of the installation ratio change during suspension motion for a push-rod system. 

You need to define the pickup points geometry (in 2D)and the hardpoints for rocker and damper mounts in the functions. The origin is taken as the intersection of the centreline and the front wheel axis on the ground.

Ideally you have finalised your pick-up points before you start working on the suspension packaging for the push/pull rod system.

The final output gives you a curve of the bump travel vs installation ratio

----------------------------------------------------------------------------------------------
evarocker_3d

The function evarocker_3d.m is similar to the 2D one except that it does the calculation for pull-rod geometry and performs 3D calculations! (of 2D motion is only a special case of 3D motion in the same plane and hence can be dealt with with no extra effort)

You need to define the 3D coordinate of the pickup points and rocker and damper mounts in the function. The origin is taken as the intersection of the centreline and the front wheel axis on the ground.

The final output gives you a curve of the bump travel vs installation ratio

This funcitoin uses the tetra2.m function, and therefore you need to put them in the same directory
