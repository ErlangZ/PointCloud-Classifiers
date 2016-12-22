build_code:
	mkdir -p build
	g++ --std=c++11 *.cpp -I/usr/include/pcl-1.7/ -I/usr/include/eigen3/ -I/usr/include/vtk-5.8/ -lpcl_io -lpcl_common -lpcl_visualization -lboost_system -o build/main 
