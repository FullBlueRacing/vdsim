There are four scripts and one function included in this folder, they are scripts for:

front_roll
rear_roll
front_bump
rear_bump

The roll scripts utilise the lineintersect. m function.

Basically they take the pickup points for the suspension geometry design and then calculate some key performance parameters. They are all 2-D kinematics for simplification as our suspension geometries are 2D (all motions are in the front plane)

You need to specify the pickup points yourself in all the scripts before you run them.

The rear ones are essentially the same as the front ones, but with different geometreis.