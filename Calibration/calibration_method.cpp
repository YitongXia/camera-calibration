 /**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "calibration.h"
#include "matrix_algo.h"


using namespace easy3d;

 // TODO: check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match)

 bool if_valid(const std::vector<Vector3D>& points_3d, const std::vector<Vector2D>& points_2d) {
     bool if_valid = points_3d.size() >= 6 && points_3d.size() == points_2d.size();
     return if_valid;
 }

 // TODO: construct the P matrix (so P * m = 0).
 Matrix construct_P_matrix(const std::vector<Vector3D>& points_3d, const std::vector<Vector2D>& points_2d){

     Matrix P(2 * points_3d.size(), 12,0.0);

     for(int i=0;i<points_3d.size();i++)
     {
         std::vector<double> temp_row = { points_3d[i].x(),points_3d[i].y(),points_3d[i].z(),1,0,0,0,0, -points_2d[i].x() * points_3d[i].x(), -points_2d[i].x() * points_3d[i].y(),-points_2d[i].x() * points_3d[i].z(), -points_2d[i].x()};
         std::vector<double> temp_row1 = { 0,0,0,0,points_3d[i].x(),points_3d[i].y(),points_3d[i].z(),1,-points_2d[i].y() * points_3d[i].x(), -points_2d[i].y() * points_3d[i].y(),-points_2d[i].y() * points_3d[i].z(), -points_2d[i].y()};
         P.set_row(2*i,temp_row);
         P.set_row(2*i+1, temp_row1);
     }
     return P;
 }


 // TODO: solve for M (the whole projection matrix, i.e., M = K * [R, t]) using SVD decomposition.
 //   Optional: you can check if your M is correct by applying M on the 3D points. If correct, the projected point
 //             should be very close to your input images points.

 Vector Construct_M (Matrix &p) {
     /// get the number of rows.
     int num_rows = p.rows();

     /// get the number of columns.
     int num_cols = p.cols();

     Matrix u(num_rows, num_rows, 0.0);   // initialized with 0s
     Matrix s(num_rows, num_cols, 0.0);   // initialized with 0s
     Matrix v(num_cols, num_cols, 0.0);   // initialized with 0s

     /// Compute the SVD decomposition of A.
     svd_decompose(p, u, s, v);

     Vector last_m = v.get_column(v.cols()-1);

     return last_m;
 }
 // TODO: extract intrinsic parameters from M.

 void extract_intrinsic_extrinsic(Vector& M, double& fx, double& fy, double& cx, double& cy, double& skew, Matrix33& R, Vector3D& t) {

     Matrix33 K;
     Matrix33 A;

     Vector3D a1(M[0],M[1],M[2]);
     Vector3D a2(M[4],M[5],M[6]);
     Vector3D a3(M[8],M[9],M[10]);
     Vector3D B(M[3],M[7],M[11]);

     A.set_row(0,a1);
     A.set_row(1,a2);
     A.set_row(2,a3);

     // 弧度还是角度
     auto cos_theta=-dot(cross(a1,a3), cross(a2,a3));

     auto cos_theta_numerator = dot((cross(a1, a3)), (cross(a2, a3)));
     auto cos_theta_denominator = norm(cross(a1, a3)) * norm(cross(a2, a3));
     auto theta = acos(-cos_theta_numerator / cos_theta_denominator);

     //auto theta=acos(cos_theta);
     auto sin_theta= sin(theta);
     auto cot_theta = 1/ tan(theta);

     auto p=1/ norm(a3);
     cx =p*p* dot(a1,a3);
     cy=p*p* dot(a2,a3);
     fx=p * p * norm(cross(a1,a3)) * sin_theta;
     fy=p * p * norm(cross(a2,a3)) * sin_theta;


     skew = -fx * cot_theta;

     Vector r1= cross(a2,a3)/ norm(cross(a2,a3));
     Vector r3=p*a3;
     Vector r2= cross(r3,r1);
     R.set_row(0,r1);
     R.set_row(1,r2);
     R.set_row(2,r3);

     // some value not sure

     K.set_row(0,{fx,skew,cx});
     K.set_row(1, {0,fx/sin_theta,cy});
     K.set_row(2,{0,0,1});

     t = p * inverse(K) * B;
 }

bool Calibration::calibration(
        const std::vector<Vector3D>& points_3d, /// input: An array of 3D points.
        const std::vector<Vector2D>& points_2d, /// input: An array of 2D image points.
        double& fx, double& fy,    /// output: the focal length (in our slides, we use 'alpha' and 'beta'),
        double& cx, double& cy,    /// output: the principal point (in our slides, we use 'u0' and 'v0'),
        double& skew,              /// output: the skew factor ('-alpha * cot_theta')
        Matrix33& R,               /// output: the 3x3 rotation matrix encoding camera orientation.
        Vector3D& t)               /// output：a 3D vector encoding camera translation.
{
    std::cout << "\nTODO: I am going to implement the calibration() function in the following file:\n"
                 "\t    - calibration_method.cpp\n\n";

    if_valid(points_3d,points_2d);
    if(if_valid(points_3d,points_2d))
    {
        Matrix P = construct_P_matrix(points_3d,points_2d);
        auto M = Construct_M(P);
        extract_intrinsic_extrinsic(M,fx,fy,cx,cy,skew,R,t);
        std::cout<<"camera calibration is finished!"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"input parameters are incorrect!"<<std::endl;
        return false;
    }

}



















