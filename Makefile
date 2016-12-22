build:
	g++ show_point_cloud.cpp -I/usr/include/pcl-1.7/ -I/usr/include/eigen3/ -lpcl_io -lboost_system -o show_point_cloud 
