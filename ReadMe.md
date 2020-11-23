This repository contains cpp implementations of various linear and non-linear filters and their analysis.

This current version contains implementation of an Extended Kalman filter which filters bearing measurements and an intended motion to compute a 2D position estimate. The required libraries are STL and Eigen.

To build the Extended Kalman Filter using g++, run the following code from the repository folder:	
	
	g++ -I/usr/include/eigen3/ -I ./bayesian_filters/ -I ./filtering_utils/ -I ./Measurement_Model/ -I ./Motion_Model/ -I ./MVN/  bayesian_filters/*.cpp filtering_utils/*.cpp Measurement_Model/bearing_meas_model.cpp Motion_Model/ctmodel.cpp Motion_Model/cvmodel.cpp MVN/mvn.cpp -o test_ekf

It can be then run using:

	./test_ekf

The filter needs to be better tuned from looking at the current results.