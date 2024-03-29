
#include "navMpc.h"
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <bits/stdc++.h>

// The program use fragments of code from
// https://github.com/udacity/CarND-MPC-Quizzes

using CppAD::AD;

// =========================================
// FG_eval class definition implementation.
// =========================================
class FG_eval 
{
    public:
        double _dt, _ref_ex, _ref_ey, _ref_vel, _l; 
        double  _w_ex, _w_ey, _w_etheta, _w_vel, _w_ev, _w_ew, _w_angvel, _w_angvel_d, _w_vel_d;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _ex_start, _ey_start, _etheta_start, _angvel_start, _a_start;
        Eigen::VectorXd _target_params;
        AD<double> cost_ex, cost_ey, cost_vel, cost_etheta, cost_angvel,cost_angvel_d, cost_vel_d;
        // Constructor
        FG_eval()
        { 
            // Set default value
            _dt         = 0.1;  // in sec
            _l          = 0.5;
            _ref_ex     = 0;
            _ref_ey     = 0;
            _ref_vel    = 0.5; // m/s
            _w_ex       = 1000;
            _w_ey       = 1000;
            _w_vel      = 1000;
            _w_angvel   = 100;
            _w_angvel_d = 0;
            _w_vel_d  = 0;

            _w_ev = 100.0;
            _w_ew = 100.0;

            _mpc_steps    = 20;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _ex_start   = _theta_start + _mpc_steps;
            _ey_start  = _ex_start + _mpc_steps;
            _etheta_start  = _ey_start + _mpc_steps;
            _v_start = _etheta_start + _mpc_steps - 1;
            _angvel_start    = _v_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params, Eigen::VectorXd target_params)
        {
            _target_params = target_params;
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _l  = params.find("L") != params.end() ? params.at("L") : _l;
            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
            _ref_ex   = params.find("REF_EX") != params.end()  ? params.at("REF_EX") : _ref_ex;
            _ref_ey  = params.find("REF_EY") != params.end() ? params.at("REF_EY") : _ref_ey;
            // _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            _w_ex   = params.find("W_EX") != params.end()   ? params.at("W_EX") : _w_ex;
            _w_etheta  = params.find("W_ETHETA") != params.end()  ? params.at("W_ETHETA") : _w_etheta;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_vel_d = params.find("W_DVEL") != params.end()     ? params.at("W_DVEL") : _w_vel_d;

            _ref_vel = _target_params[3];   // 动点速度

            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _ex_start   = _theta_start + _mpc_steps;
            _ey_start  = _ex_start + _mpc_steps;
            _etheta_start  = _ey_start + _mpc_steps;
            _v_start = _etheta_start + _mpc_steps - 1;
            _angvel_start    = _v_start + _mpc_steps - 1;
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        void operator()(ADvector& fg, const ADvector& vars)
        {
            // fg[0] 代价函数
            fg[0] = 0;
            cost_ex =  0;
            cost_ey = 0;
            cost_vel = 0;
            cost_etheta = 0;
            cost_angvel = 0;
            cost_angvel_d = 0;
            cost_vel_d = 0;

            for (int i = 0; i < _mpc_steps; i++) 
            {
                fg[0] += _w_ex * CppAD::pow(vars[_ex_start + i], 2);
                // fg[0] += _w_ex * CppAD::pow(vars[_ey_start + i], 2);
                fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start + i], 2);
                fg[0] += _w_vel * CppAD::pow(vars[_etheta_start + i] - _ref_vel, 2);
                fg[0] += _w_angvel * CppAD::pow(vars[_angvel_start + i], 2);
                cost_ex +=  _w_ex * CppAD::pow(vars[_ex_start + i] - _ref_ex, 2);
                // cost_ey +=  _w_ex * CppAD::pow(vars[_ey_start + i] - _ref_ey, 2); 
                cost_vel += _w_vel * CppAD::pow(vars[_etheta_start + i] - _ref_vel, 2);
                cost_etheta += _w_etheta * CppAD::pow(vars[_etheta_start + i], 2);
                cost_angvel += _w_angvel * CppAD::pow(vars[_angvel_start + i], 2);
            }
            cout << "--- costs   ---" <<endl;
            cout << "cost_ex: " << cost_ex << ", \t cost_etheta:" << cost_etheta << endl;
            cout << "cost_vel:" << cost_vel << ", \t cost_angvel" << cost_angvel << endl;

            // 角速度、加速度变化惩罚
            for (int i = 0; i < _mpc_steps - 2; i++) {
              fg[0] += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
              fg[0] += _w_vel_d * CppAD::pow(vars[_v_start + i + 1] - vars[_v_start + i], 2);

              cost_angvel_d += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
              cost_vel_d += _w_vel_d * CppAD::pow(vars[_v_start + i + 1] - vars[_v_start + i], 2);
            }
            cout << "cost of angvel_d, vel_d: " << cost_angvel_d << ", " << cost_vel_d << endl; 
            
            // fg[x] 约束
            // Initial constraints
            // 加载与目标相关参数 t时刻
            AD<double> xR0 = _target_params[0];
            AD<double> yR0 = _target_params[1];
            AD<double> thetaR0 = _target_params[2];
            AD<double> vR0 = _target_params[3];
            AD<double> wR0 = _target_params[4];
            AD<double> xd = _target_params[5];
            AD<double> yd = _target_params[6];

            // 系统运动学模型约束 单射
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t.     t时刻
                AD<double> x0     = vars[_x_start +     i];
                AD<double> y0     = vars[_y_start +     i];
                AD<double> theta0 = vars[_theta_start + i];
                AD<double> ex0    = vars[_ex_start +    i];
                AD<double> ey0    = vars[_ey_start +    i];
                AD<double> thetaE0 = vars[_etheta_start +    i];

                AD<double> v0     = vars[_v_start + i];
                AD<double> w0     = vars[_angvel_start + i];
                

                // t + 1 时刻
                fg[1 + _x_start + i]     = x0 + v0 * CppAD::cos(theta0) * _dt;
                fg[1 + _y_start + i]     = y0 + v0 * CppAD::sin(theta0) * _dt;
                fg[1 + _theta_start + i] = theta0 +  w0 * _dt;
                fg[1 + _ex_start + i]    = ex0 + ((vR0 - yd*wR0)*CppAD::cos(thetaE0) - xd*wR0*CppAD::sin(thetaE0) + ey0*w0 - v0) * _dt;
                fg[1 + _ey_start + i]    = ey0 + ((vR0 - yd*wR0)*CppAD::sin(thetaE0) + xd*wR0*CppAD::cos(thetaE0) - ex0*w0 - w0*_l) * _dt;
                fg[1 + _etheta_start + i]= thetaE0 + w0 * _dt;

            }
        }
};

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC() 
{
    // 设置默认值
    _mpc_steps = 20;
    _max_angvel = 3.0; // Maximal angvel radian (~30 deg)
    _max_throttle = 1.0; // Maximal throttle accel
    _bound_value  = 1.0e3; // Bound value for other variables
    _max_vel = 1.0;

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _ex_start   = _theta_start + _mpc_steps;
    _ey_start  = _ex_start + _mpc_steps;
    _etheta_start  = _ey_start + _mpc_steps;
    _v_start = _etheta_start + _mpc_steps - 1;     // TODO 考虑ev ew
    _angvel_start    = _v_start + _mpc_steps - 1;  // TODO 考虑ev ew   // 状态量 x y theta, 控制量v w
}

void MPC::LoadParams(const std::map<string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    _max_vel = _params.find("MAXVEL") != _params.end()  ? _params.at("MAXVEL") : _max_vel;
    
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _ex_start   = _theta_start + _mpc_steps;
    _ey_start  = _ex_start + _mpc_steps;
    _etheta_start  = _ey_start + _mpc_steps;
    _v_start = _etheta_start + _mpc_steps - 1;     // TODO 考虑ev ew
    _angvel_start    = _v_start + _mpc_steps - 1;  // TODO 考虑ev ew   // 状态量 x y theta, 控制量v w
    cout << "\n!! MPC Obj parameters updated !! " << endl; 
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd target_params) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;   // 可以将Dvector视为std::vector<double>的别名。CPPAD_TESTVECTOR是一个宏，用于定义基于double类型的向量
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double ex = state[3];
    const double ey = state[4];
    const double etheta = state[5];

    // 设置模型变量
    size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;// TODO 考虑ev ew
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 6;          // TODO 考虑ev ew

    // 初始化变量
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start]      = x;
    vars[_y_start]      = y;
    vars[_theta_start]  = theta;
    vars[_ex_start]     = ex;
    vars[_ey_start]     = ey;
    vars[_etheta_start] = etheta;

    // 设置变量的上下限
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // 将所有非执行机构的上限和下限设置为最大负值和正值。
    for (int i = 0; i < _v_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // 速度上下限
    for (int i = _v_start; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_max_vel;
        vars_upperbound[i] = _max_vel;
    }
    //  角度的上限和下限分别设置为-25度和25度（以弧度为单位的值）。
    for (int i = _angvel_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }


    // 约束的下限和上限,第一步预测的约束为初始值,后面的约束都为0
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_ex_start] = ex;
    constraints_lowerbound[_ey_start] = ey;
    constraints_lowerbound[_etheta_start] = etheta;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_ex_start] = ex;
    constraints_upperbound[_ey_start] = ey;
    constraints_upperbound[_etheta_start] = etheta;

    // 计算目标和约束的对象
    FG_eval fg_eval;
    fg_eval.LoadParams(_params, target_params);

    // options for IPOPT solver
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_v_start]);
    result.push_back(solution.x[_angvel_start]);
    return result;
}
