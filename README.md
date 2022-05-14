# camera-calibration

 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 
This program is a collaboration with Fengyan Zhang(zmocheng@gmail.com) and Leo Kan(leo.kan01@gmail.com), we discuss the implement method together and write the code independently.

This program is to implement camera calibration algorithm, It is implemented in C++ with Easy3D library.

The algorithm consists of the following step:
* use a virtual apparatus to collect a set of 3D-2D corresponding points;
* compute the SVD and inverse of a matrix;
* Extract the intrinsic camera parameters;
* Extract the extrinsic camera parameters;
* visualize and validate the calibration results with variance and RMSE.

Some imtermediate check:
* Check whether matrix P is constructed correctly
Print out it to the console after its construction to verify the structure(e.g. check whether the number of columns are equal to 12 or not). In our tests, the number of rows (of P matrix) equals 2n (n is the number of input 2D/3D points) and the number of columns equals 12, proving that the structure is correctly constructe

* Check whether SVD solution is correct

(a) Form the vector m from the last column of matrix V, and check the product of matrix P and m, since m is a 12-dimensional vector, the absolute value of each element should be within a small threshold from 0. In our tests, after testing all the input files, 0.01 is chosen to be the default value of this threshold, and meantime each element is printed to the console for direct observation. The threshold can be set differently according to different input files.

(b) Form the projection matrix M from the last column of matrix V, and multiply it with each input 3D points. In this manner each corresponding 2D pixel point can be obtained and the differences between input and calculated 2D points can be compared. This is actually how we estimate the accuracy of the results. In our tests, the difference (expressed as variance) is different due to the noisy level of different input files, generally speaking, the difference will be small if the input has a relatively low noisy level (more ‘exact’) and vice versa.


the implement result:

* we use the "exact" file (3D point and 2D point coordinates correspond exactly) for calibration and overlapped the results of the original photo with the calibration results, resulting in a perfect match.

<div align=center> <img src="https://user-images.githubusercontent.com/75926656/168442689-ea382420-8d05-4045-80cb-7886612eba7e.png" width="500"></div>

  
* Then we use "noisy" file (3D point and 2D point coordinates correspond not that well, with some random offset) for calibration and overlapped the results of the original photo with the calibration results, which shows a significant offset.
 
<div align=center> <img src="https://user-images.githubusercontent.com/75926656/168442953-5d329b97-ce3f-4685-9e45-9a6bff1a9dc7.png" width="500"></div>


Quality

The quality mainly depends on the quality of the input files(such as noisy level, the number of correspondences). Overall if the quality of the input data sets is high and the number of 3D/2D correspondences is big enough, the results will be most likely reliable.
