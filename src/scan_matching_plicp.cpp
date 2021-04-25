#include "scan_matching_plicp.h"

ScanMatchingPLICP::ScanMatchingPLICP()
{

}
ScanMatchingPLICP::~ScanMatchingPLICP()
{

}
void ScanMatchingPLICP::ScanMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr ref,pcl::PointCloud<pcl::PointXYZI>::Ptr per)
{
    points_map_.clear();
    lane_l_.clear();
    matched_map_points_.clear();
    nearest_points_2_.clear();
    nearest_points_.clear();
    normal_vector_.clear();
    ceres::Problem problem;
    ceres::Problem problem_pp;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(ref->makeShared());
    int K = 2;
    std::vector<int> index(K);
    std::vector<float> distance(K);
    double pose[3] = {0.0,-0.3,0};
    double pose_pp[3] = {0.0,-0.3,0};
    Eigen::Vector2d aver_points;
    
    for(int i = 0; i < per->points.size(); i++)
    {
        if (kdtree.nearestKSearch(per->points[i], K, index, distance) == K){
			//add constraints
			Eigen::Vector2d q = Eigen::Vector2d(per->points[i].x,per->points[i].y);
			Eigen::Vector2d p1 = PCL2Eigen(ref->points[index[0]]);
			Eigen::Vector2d p2 = PCL2Eigen(ref->points[index[1]]);

			matched_map_points_.emplace_back(q);
			points_map_.emplace_back(q);
			nearest_points_.emplace_back(p1);
			nearest_points_2_.emplace_back(p2);

			Eigen::Vector2d normal_vector;
			normal_vector(0) = -(p1(1) - p2(1));
			normal_vector(1) = p1(0) - p2(0);
			double norm;
			norm = sqrt(normal_vector[0]*normal_vector[0] + normal_vector[1]*normal_vector[1]);
			if(norm > 0.01)
			{
					normal_vector(0) = normal_vector(0)/norm;
					normal_vector(1) = normal_vector(1)/norm;
			}
			else
			{
					cout << "norm == 0" << endl;
					normal_vector(0) = normal_vector(0);
					normal_vector(1) = normal_vector(1);
					continue;
			}
			norm = sqrt(normal_vector[0]*normal_vector[0] + normal_vector[1]*normal_vector[1]);
			if(abs(norm - 1.0)  < 1e-4)
			{
					normal_vector_.emplace_back(normal_vector);
			}
			ceres::CostFunction *cost_function = PLICP::create(q, p1, p2);
			problem.AddResidualBlock(cost_function,
									nullptr,
									pose);
			ceres::CostFunction *cost_function_pp = PPICP::create(q, p1, p2);
			problem_pp.AddResidualBlock(cost_function_pp,
									nullptr,
									pose_pp);
		}
        
    }
    //check points 

    Eigen::MatrixXd points_distibution(2,2); 
    points_distibution = Eigen::MatrixXd::Zero(2,2);
    for(int i = 0; i < ref->points.size(); i++)
    {
        aver_points += Eigen::Vector2d(ref->points[i].x,ref->points[i].y);
    }
    aver_points = aver_points/ ref->points.size();
    std::cout << aver_points.transpose() << std::endl;
    for(int i = 0; i < ref->points.size(); i++)
    {

        points_distibution += (Eigen::Vector2d(ref->points[i].x,ref->points[i].y)-aver_points)*
                              (Eigen::Vector2d(ref->points[i].x,ref->points[i].y)-aver_points).transpose();
    }
    Eigen::EigenSolver<Eigen::Matrix2d> pointses(points_distibution);

    std::cout << "Eigenvalves for points distribution:\n" << pointses.eigenvalues().real() << std::endl;
    std::cout << "Eigenvectors for points distribution:\n" << pointses.eigenvectors().real() << std::endl;
	// plicp
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    rot_plicp_ = Euler2Rotation(pose[2]);
    Eigen::Vector3d transform;
    transform << pose[0],pose[1],pose[2];
    pose[2] = pose[2]*180/3.14;
    trans_plicp_ << pose[0],pose[1];
    
    printf("pl icp: %lf, %lf, %lf\n", pose[0], pose[1],pose[2] );
    Eigen::MatrixXd COV(3,3);
    ComputeCovariance(points_map_,nearest_points_,transform,COV);

	// ppicp
    ceres::Solver::Options options_pp;
    options_pp.linear_solver_type = ceres::DENSE_SCHUR;
    options_pp.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary_pp;
    ceres::Solve(options_pp, &problem_pp, &summary_pp);
    std::cout << summary_pp.BriefReport() << "\n";
    Eigen::Vector3d transform_pp;
    transform_pp << pose_pp[0],pose_pp[1],pose_pp[2];
    pose_pp[2] = pose_pp[2]*180/3.14;
    
    printf("pp icp: %lf, %lf, %lf\n", pose_pp[0], pose_pp[1],pose_pp[2] );
    Eigen::MatrixXd COV_pp(3,3);
    ComputeCovarianceWithPP(points_map_,nearest_points_,transform_pp,COV_pp);

}

Eigen::Vector2d ScanMatchingPLICP::PCL2Eigen(pcl::PointXYZI& p)
{
    Eigen::Vector2d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    return pp;
}

Eigen::Matrix2d ScanMatchingPLICP::Euler2Rotation(double yaw)
{
    Eigen::Matrix2d rot;
    rot << cos(yaw), -sin(yaw),sin(yaw),cos(yaw);
    return rot;
}

void ScanMatchingPLICP::ComputeCovariance(vector<Eigen::Vector2d>& data_pi, vector<Eigen::Vector2d>& model_qi, Eigen::Vector3d& transform, Eigen::MatrixXd& ICP_COV)
{
    
    double Tx = transform(0);
    double Ty = transform(1);
    double yaw = transform(2);

    double x, y, a;
    x = Tx; y = Ty;
    a = yaw; 

    //Matrix initialization
    Eigen::MatrixXd d2J_dX2(3,3);
    d2J_dX2 = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd Apn(3,3);
    Apn = Eigen::MatrixXd::Zero(3,3);
    /****  Calculating d2J_dX2  ****/
    int count = normal_vector_.size();
    for (size_t s = 0; s < count; ++s )
    {
        int index = normal_vector_[s](2);
        double pix = data_pi[index](0);
        double piy = data_pi[index](1);
        double qix = model_qi[index](0);
        double qiy = model_qi[index](1);

        double nix = normal_vector_[s](0);
        double niy = normal_vector_[s](1);
        Eigen::Vector2d ai_ver;
        ai_ver(0) = -piy;
        ai_ver(1) = pix;
        Eigen::Vector2d ni;
        ni << nix,niy;
        
        Eigen::Matrix3d Apn_temp;
        
        Apn_temp << (ni.transpose()*ai_ver)*(ni.transpose()*ai_ver), (ni.transpose()*ai_ver)*ni.transpose(),
                    ni*(ni.transpose()*ai_ver), ni*ni.transpose();
        Apn = Apn + Apn_temp;
        if (niy!=niy)// for nan removal in the input point cloud data:)
            continue;
        
        double norm = sqrt(nix*nix+niy*niy);
        if(norm - 1.0  > 1e-4 )
        {
            continue;
        }
        double 	d2J_dx2,     d2J_dydx,	  d2J_dadx,  
                d2J_dxdy,    d2J_dy2,	  d2J_dady,
                d2J_dxda,    d2J_dyda,    d2J_da2;

        d2J_dx2 =

                2*pow(nix,2);


        d2J_dy2 =

                2*pow(niy,2);

        d2J_dydx =

                2*nix*niy;

        d2J_dxdy =

                2*nix*niy;

        d2J_da2 =
        
                (2*nix*(pix*cos(a) - piy*sin(a)) + 2*niy*(piy*cos(a) + pix*sin(a)))*(nix*(qix - x - pix*cos(a) + piy*sin(a)) - niy*(y - qiy + piy*cos(a) + pix*sin(a))) + (nix*(piy*cos(a) + pix*sin(a)) - niy*(pix*cos(a) - piy*sin(a)))*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));
        
                
        d2J_dxda =
        
                -nix*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));
        
        
        d2J_dadx =
        
                -2*nix*(nix*(piy*cos(a) + pix*sin(a)) - niy*(pix*cos(a) - piy*sin(a)));
        
        
        d2J_dyda =
        
                -niy*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));
        
        
        d2J_dady =
        
                -2*niy*(nix*(piy*cos(a) + pix*sin(a)) - niy*(pix*cos(a) - piy*sin(a)));
        Eigen::MatrixXd d2J_dX2_temp(3,3);

        d2J_dX2_temp << d2J_dx2,     d2J_dydx,	  d2J_dadx,
                        d2J_dxdy,    d2J_dy2,	  d2J_dady,   
                        d2J_dxda,    d2J_dyda,    d2J_da2;	



        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }// End of the FOR loop!!!
    int n = normal_vector_.size();
    if (n > 200) n = 200;////////////****************************IMPORTANT CHANGE***** but may not affect********************/////////////////////////////////////////
   
//     std::cout << "\nNumber of Correspondences used for ICP's covariance estimation = " << n << std::endl;
//     std::cout << "Apn:\n" << Apn << std::endl;
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(d2J_dX2); 
    Eigen::Vector3d eigen_values = eigen_solver.eigenvalues().real();
     
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver_apn(Apn);
    Eigen::Vector3d eigen_values_apn = eigen_solver_apn.eigenvalues().real();
    std::cout << "eigen_value:" <<eigen_values.transpose() << std::endl;
//     std::cout << "eigen_value_apn:" <<eigen_values_apn.transpose() << std::endl;
    std::cout << "eien_vectors:\n" << eigen_solver.eigenvectors().real() << std::endl;
//     std::cout << "eien_vectors:\n" << eigen_solver_apn.eigenvectors().real() << std::endl;

    double x_value = eigen_values(0);
    Eigen::MatrixXd  d2J_dX2_removex(2,2);
    Eigen::MatrixXd d2J_dZdX_removex(2,2*n);
    if( x_value < 1e-4 )
    {
        d2J_dX2_removex << d2J_dX2(1,1), d2J_dX2(1,2),
                           d2J_dX2(2,1),  d2J_dX2(2,2);	
        
    }
    Eigen::MatrixXd d2J_dZdX(3,4*n);

    for (int k = 0; k < n ; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        int index = normal_vector_[k](2);

        double pix = data_pi[index](0);
        double piy = data_pi[index](1);
        double qix = model_qi[index](0);
        double qiy = model_qi[index](1);

        double nix = normal_vector_[k](0);;
        double niy = normal_vector_[k](1);;
        if (niy!=niy)// for nan removal in the input point cloud data:)
            continue;
        int norm = sqrt(nix*nix+niy*niy);
        if(norm != 1 )
        {
            continue;
        }
        Eigen::MatrixXd d2J_dZdX_temp(3,4);


        double 	d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dqix_dx,    d2J_dqiy_dx,	  
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dqix_dy,    d2J_dqiy_dy,	   
                d2J_dpix_da,    d2J_dpiy_da,    d2J_dqix_da,    d2J_dqiy_da;

        
        d2J_dpix_dx = 2*nix*(nix*cos(a) + niy*sin(a));
        
        d2J_dpix_dy = 2*niy*(nix*cos(a) + niy*sin(a));
        
        d2J_dpix_da =
        
                - (2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)))*(nix*cos(a) + niy*sin(a)) - (2*niy*cos(a) - 2*nix*sin(a))*(nix*(qix - x - pix*cos(a) + piy*sin(a)) - niy*(y - qiy + piy*cos(a) + pix*sin(a)));
        
        
        d2J_dpiy_dx = 2*nix*(niy*cos(a) - nix*sin(a));
        
        d2J_dpiy_dy = 2*niy*(niy*cos(a) - nix*sin(a));
        
        d2J_dpiy_da =
        
                (2*nix*cos(a) + 2*niy*sin(a))*(nix*(qix - x - pix*cos(a) + piy*sin(a)) - niy*(y - qiy + piy*cos(a) + pix*sin(a))) - (2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)))*(niy*cos(a) - nix*sin(a));
        
        
        d2J_dqix_dx = -2*nix*nix;
        
        d2J_dqix_dy = -2*nix*niy;
        
        d2J_dqix_da =
        
                nix*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));
        
        
        d2J_dqiy_dx = -2*nix*niy;
        
        d2J_dqiy_dy = -2*niy*niy;
        
        d2J_dqiy_da = niy*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));

        d2J_dZdX_temp <<    d2J_dpix_dx,    d2J_dpiy_dx,      d2J_dqix_dx,    d2J_dqiy_dx,	   
                            d2J_dpix_dy,    d2J_dpiy_dy,      d2J_dqix_dy,    d2J_dqiy_dy,
                            d2J_dpix_da,    d2J_dpiy_da,      d2J_dqix_da,    d2J_dqiy_da;
        


        d2J_dZdX.block<3,4>(0,3*k) = d2J_dZdX_temp;
        // if(x_value < 1e-4)
        {
            Eigen::MatrixXd d2J_dZdX_temp(2,2);
            d2J_dZdX_temp << d2J_dpiy_dy,   d2J_dqiy_dy,
                             d2J_dpiy_da,   d2J_dqiy_da;
        
            d2J_dZdX_removex.block<2,2>(0,2*k) = d2J_dZdX_temp;
        }

    }
    
    Eigen::MatrixXd cov_z(3*n,3*n);
    cov_z = 0.01 * Eigen::MatrixXd::Identity(3*n,3*n); 
    ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();
    std::cout << "ICP_COV = \n" << ICP_COV << std::endl;
    
    Eigen::MatrixXd cov_z_removex(2*n,2*n);
    cov_z_removex = 0.01 * Eigen::MatrixXd::Identity(2*n,2*n);
    ICP_COV =  d2J_dX2_removex.inverse() * d2J_dZdX_removex * cov_z_removex * d2J_dZdX_removex.transpose() * d2J_dX2_removex.inverse();
    std::cout <<  "ICP_COV1 = \n" << ICP_COV << std::endl;
    
}

void ScanMatchingPLICP::ComputeCovarianceWithPP(vector<Eigen::Vector2d>& data_pi, vector<Eigen::Vector2d>& model_qi, Eigen::Vector3d& transform, Eigen::MatrixXd& ICP_COV)
{
    
    double Tx = transform(0);
    double Ty = transform(1);
    double yaw = transform(2);

    double x, y, a;
    x = Tx; y = Ty;
    a = yaw; 
    //Matrix initialization
    Eigen::MatrixXd d2J_dX2(3,3);
    d2J_dX2 = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd App(3,3);
    App = Eigen::MatrixXd::Zero(3,3);
    /****  Calculating d2J_dX2  ****/
    int count = normal_vector_.size();
    for (size_t s = 0; s < count; ++s )
    {
        int index = normal_vector_[s](2);
        double pix = data_pi[index](0);
        double piy = data_pi[index](1);
        double qix = model_qi[index](0);
        double qiy = model_qi[index](1);

        double nix = normal_vector_[s](0);
        double niy = normal_vector_[s](1);
        Eigen::Vector2d ai_ver;
        ai_ver(0) = -piy;
        ai_ver(1) = pix;
        Eigen::Vector2d ni;
        ni << nix,niy;
        
        Eigen::Matrix3d App_temp;
        
        App_temp << ai_ver.transpose()*ai_ver, ai_ver.transpose(),
                    ai_ver, Eigen::Matrix2d::Identity();
        App = App + App_temp;
        if (niy!=niy)// for nan removal in the input point cloud data:)
            continue;
        
        double norm = sqrt(nix*nix+niy*niy);
        if(norm - 1.0  > 1e-4 )
        {
            continue;
        }
        double 	d2J_dx2,     d2J_dydx,	  d2J_dadx,  
                d2J_dxdy,    d2J_dy2,	  d2J_dady,
                d2J_dxda,    d2J_dyda,    d2J_da2;

        d2J_dx2 = 2;
        
        d2J_dy2 = 2;
        
        d2J_dydx = 0;
        
        d2J_dxdy = 0;
        
        d2J_da2 =
        
        (piy*cos(a) + pix*sin(a))*(2*piy*cos(a) + 2*pix*sin(a)) + (pix*cos(a) - piy*sin(a))*(2*pix*cos(a) - 2*piy*sin(a)) + (2*pix*cos(a) - 2*piy*sin(a))*(qix - x - pix*cos(a) + piy*sin(a)) - (2*piy*cos(a) + 2*pix*sin(a))*(y - qiy + piy*cos(a) + pix*sin(a));
        
        d2J_dxda = - 2*piy*cos(a) - 2*pix*sin(a);
        
        d2J_dadx = - 2*piy*cos(a) - 2*pix*sin(a);
        
        d2J_dyda = 2*pix*cos(a) - 2*piy*sin(a);
        
        d2J_dady = 2*pix*cos(a) - 2*piy*sin(a);

        Eigen::MatrixXd d2J_dX2_temp(3,3);

        d2J_dX2_temp << d2J_dx2,     d2J_dydx,	  d2J_dadx,
                        d2J_dxdy,    d2J_dy2,	  d2J_dady,   
                        d2J_dxda,    d2J_dyda,    d2J_da2;	



        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }// End of the FOR loop!!!
    int n = normal_vector_.size();
    if (n > 200) n = 200;////////////****************************IMPORTANT CHANGE***** but may not affect********************/////////////////////////////////////////
   
    // std::cout << "\nNumber of Correspondences used for ICP's covariance estimation = " << n << std::endl;
    // std::cout << "d2J_dX2:\n" << d2J_dX2 << std::endl;
//     std::cout << "App:\n" << App << std::endl;
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(d2J_dX2);
    Eigen::Vector3d eigen_values = eigen_solver.eigenvalues().real();
    Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver_apn(App);
    Eigen::Vector3d eigen_values_apn = eigen_solver_apn.eigenvalues().real();
    std::cout << "eigen_value_PP:" <<eigen_values.transpose() << std::endl;
//     std::cout << "eigen_value_app:" <<eigen_values_apn.transpose() << std::endl;
    std::cout << "eigen_vectors:\n" << eigen_solver.eigenvectors().real() << std::endl;
//     std::cout << "eigen_vectors_app:\n" << eigen_solver_apn.eigenvectors().real() << std::endl;

    double x_value = eigen_values(0);
    Eigen::MatrixXd  d2J_dX2_removex(2,2);
    Eigen::MatrixXd d2J_dZdX_removex(2,2*n);
    // if( x_value < 1e-4 )
    {
        // std::cout << d2J_dX2<< std::endl;
        d2J_dX2_removex << d2J_dX2(1,1), d2J_dX2(1,2),
                           d2J_dX2(2,1),  d2J_dX2(2,2);	
        // std::cout << d2J_dX2_removex<< std::endl;
        
    }
    Eigen::MatrixXd d2J_dZdX(3,4*n);

    for (int k = 0; k < n ; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        int index = normal_vector_[k](2);

        double pix = data_pi[index](0);
        double piy = data_pi[index](1);
        double qix = model_qi[index](0);
        double qiy = model_qi[index](1);

        double nix = normal_vector_[k](0);;
        double niy = normal_vector_[k](1);;
        if (niy!=niy)// for nan removal in the input point cloud data:)
            continue;
        int norm = sqrt(nix*nix+niy*niy);
        if(norm != 1 )
        {
            continue;
        }
        Eigen::MatrixXd d2J_dZdX_temp(3,4);


        double 	d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dqix_dx,    d2J_dqiy_dx,	  
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dqix_dy,    d2J_dqiy_dy,	   
                d2J_dpix_da,    d2J_dpiy_da,    d2J_dqix_da,    d2J_dqiy_da;

        
        d2J_dpix_dx = 2*cos(a);
        d2J_dpix_dy = 2*sin(a);
        d2J_dpix_da =
        
        2*cos(a)*(y - qiy + piy*cos(a) + pix*sin(a)) + 2*sin(a)*(qix - x - pix*cos(a) + piy*sin(a)) - cos(a)*(2*piy*cos(a) + 2*pix*sin(a)) + sin(a)*(2*pix*cos(a) - 2*piy*sin(a));
        
        d2J_dpiy_dx = -2*sin(a);
        
        d2J_dpiy_dy = 2*cos(a);
        
        d2J_dpiy_da =
        
        2*cos(a)*(qix - x - pix*cos(a) + piy*sin(a)) - 2*sin(a)*(y - qiy + piy*cos(a) + pix*sin(a)) + cos(a)*(2*pix*cos(a) - 2*piy*sin(a)) + sin(a)*(2*piy*cos(a) + 2*pix*sin(a));
        
        d2J_dqix_dx = -2;
        d2J_dqix_dy = 0;
        d2J_dqix_da = 2*piy*cos(a) + 2*pix*sin(a);
        d2J_dqiy_dx = 0;
        d2J_dqiy_dy = -2;
        d2J_dqiy_da = 2*piy*sin(a) - 2*pix*cos(a);

        d2J_dZdX_temp <<    d2J_dpix_dx,    d2J_dpiy_dx,      d2J_dqix_dx,    d2J_dqiy_dx,	   
                            d2J_dpix_dy,    d2J_dpiy_dy,      d2J_dqix_dy,    d2J_dqiy_dy,
                            d2J_dpix_da,    d2J_dpiy_da,      d2J_dqix_da,    d2J_dqiy_da;
        


        d2J_dZdX.block<3,4>(0,3*k) = d2J_dZdX_temp;
        // if(x_value < 1e-4)
        {
            Eigen::MatrixXd d2J_dZdX_temp(2,2);
            d2J_dZdX_temp << d2J_dpiy_dy,   d2J_dqiy_dy,
                             d2J_dpiy_da,   d2J_dqiy_da;
        
            d2J_dZdX_removex.block<2,2>(0,2*k) = d2J_dZdX_temp;
        }

    }
    
//     std::cout <<"d2J_dZdX:\n" << d2J_dZdX << std::endl;
    Eigen::MatrixXd cov_z(3*n,3*n);
    cov_z = 0.01 * Eigen::MatrixXd::Identity(3*n,3*n); 
    ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();
    std::cout << "ICP_COV_PP: = \n" << ICP_COV << std::endl;
    
    Eigen::MatrixXd cov_z_removex(2*n,2*n);
    cov_z_removex = 0.01 * Eigen::MatrixXd::Identity(2*n,2*n);
    ICP_COV =  d2J_dX2_removex.inverse() * d2J_dZdX_removex * cov_z_removex * d2J_dZdX_removex.transpose() * d2J_dX2_removex.inverse();
    std::cout <<  "ICP_COV_PP = \n" << ICP_COV << std::endl;
    
}