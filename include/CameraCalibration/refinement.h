#ifndef __REFINEMENT_H__
#define __REFINEMENT_H__
#include <ceres/ceres.h>

namespace xb
{

struct Fitting_Cost
{
    Fitting_Cost(double a, double b, double c) : 
            a_(a),
            b_(b),
            c_(c)
    {}

    template<typename T>
    bool operator()(const T* const xy, T* residual) const
    {
        residual[0] = pow(T(a_) * xy[0] + T(b_) * xy[1] + T(c_), 2) / (T(a_) * T(a_) + T(b_) * T(b_));
        return true;
    }

    const double a_, b_, c_;
};

std::vector<double> LM_refinement(std::vector<Eigen::Vector3f> lines)
{
    double xy[2] = {0, 0};
    ceres::Problem problem;
    for(size_t i = 0; i < lines.size(); i++)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Fitting_Cost, 1, 2>(new Fitting_Cost(lines[i][0], lines[i][1], lines[i][2])), 
                                nullptr, xy);
    }

    ceres::Solver::Options options; 
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    // options.max_num_iterations = 10000;
    // options.gradient_tolerance = 1e-3; //梯度阈值
    // options.function_tolerance = 1e-4;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary; 
    ceres::Solve(options, &problem, &summary); 
    
    // std::cout << summary.BriefReport() << std::endl; 
    std::cout << "x = " << xy[0] << " y = " << xy[1] << std::endl; 
    return {xy[0], xy[1]};
}




} // namespace xb


#endif