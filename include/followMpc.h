/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#ifndef MPC_H
#define MPC_H

#include <vector>
#include <map>
#include <Eigen/Core>

using namespace std;

class MPC
{
    public:
        MPC();
    
        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;

        vector<vector<vector<double>>> all_obs;     // 场景中所有障碍物的稀疏轨迹
        vector<vector<vector<double>>> all_dmbe;    // 场景中所有障碍物的稀疏轨迹的dmbe
        vector<vector<double>> ob;          // 稠密轨迹，用于mpc。[x,y,v,theta]
        vector<vector<double>> min_dmbe;    // 稠密轨迹的dmbe。[h,k,a,b,phi]
        vector<double> init_states; // (x, y, theta, v, w, a, dw)

        void LoadParams(const std::map<string, double> &params);
    
    private:
        // Parameters for mpc solver
        double _max_angvel, _max_throttle, _bound_value, _max_vel, _max_a, _max_dw;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _ex_start, _ey_start, _etheta_start, _angvel_start, _a_start, _dw_start, _cbf_start;
        std::map<string, double> _params;

        unsigned int dis_cnt;
};

#endif /* MPC_H */
