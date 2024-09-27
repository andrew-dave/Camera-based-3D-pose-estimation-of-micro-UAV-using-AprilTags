# Camera based 3D pose estimation of micro UAV using AprilTags
The goal is to implement a vision-based 3D pose estimator which estimates the position, orientation, linear and angular velocities of a Nano+ quadcopter. 
The drone is made to fly over a mat of April tags and the data relating to the image is provided. 
The project initially involves implementation of projective transformation-based position and orientation estimation and then involves calculation of velocities through optical flow, based on the given image data where the method of Randon Sample Consensus(RANSAC) is implemented for the rejection of outliers

#
