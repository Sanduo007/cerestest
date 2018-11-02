#include <stdio.h>
#include <random>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Core>

#include "CostFunction.h"

using namespace std;
using namespace Eigen;

int main()
{
    //-------------------------------------------------first: POSE optimization problem
    double R[4] = {0.0, 0.0, 0.0, 0.0}; //use EPNP as init
    double T[3] = {0.0, 0.0, 0.0};
    vector<Eigen::Vector3d> allworldcorners3d;
    vector<Eigen::Vector2d> allcorners;

    ceres::Problem poseproblem;
    ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();

    poseproblem.AddParameterBlock(R, 4, local_parameterization);
    poseproblem.AddParameterBlock(T, 3);

    for (int i = 0; i < allworldcorners3d.size(); i++)
    {
        ceres::CostFunction *cost_function = REPROJECT_COST::Create(allcorners[i](0), allcorners[i](1), allworldcorners3d[i]);
        poseproblem.AddResidualBlock(cost_function, NULL, R, T);
    }
    ceres::Solver::Options poseoptions;
    poseoptions.linear_solver_type = ceres::DENSE_SCHUR;
    poseoptions.minimizer_progress_to_stdout = true; // 输出到cout

    ceres::Solver::Summary posesummary;
    ceres::Solve(poseoptions, &poseproblem, &posesummary);

    cout << posesummary.BriefReport() << endl;
    cout << "R " << R << " T " << T << endl;
    //----------------------------------------------second:a leastsquare carve fitting problem
    // cout << std::fixed;
    // double x = 3.0, y = 2.5;
    // int N = 100;
    // double sigma = 1.0;
    // double xy[2] = {5, 5}; //the initial values are of tremendous importance.If [0,0] was given here, the problem will be caught in local optimum

    // random_device rd{};
    // normal_distribution<> d{0, sigma};

    // vector<double> k1, k2, z;
    // for (int i = 0; i < N; i++)
    // {
    //     double mk1 = i / 10.0;
    //     double mk2 = (i + 5.0) / 10.0;
    //     k1.push_back(mk1);
    //     k2.push_back(mk2);
    //     z.push_back(x * x * mk1 + y * y * y * mk2 + d(rd));
    // }

    // ceres::Problem problem;
    // for (int i = 0; i < N; i++)
    // {
    //     cout << k1[i] << " " << k2[i] << " " << z[i] << endl;
    //     ceres::CostFunction *cost_function = CURVE_FITTING_COST::Create(k1[i], k2[i], z[i]);

    //     problem.AddResidualBlock(cost_function, nullptr, xy);
    // }

    // ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.minimizer_progress_to_stdout = true; // 输出到cout
    // ceres::Solver::Summary summary;              // 优化信息

    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // ceres::Solve(options, &problem, &summary);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // cout << summary.BriefReport() << endl;
    // cout << "xy: " << xy[0] << " " << xy[1] << endl;
    return 0;
}