#pragma once

#include <memory>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <esv_planner/env/map_manager/PCSmap_manager.h>
#include <esv_planner/env/swept_volume/sw_manager.hpp>

#include "esv_planner/env/grid_map.h"

#include <esv_planner/optimization/svsdf_backend/utils/minco.hpp>
#include <esv_planner/optimization/svsdf_backend/utils/config.hpp>
#include <esv_planner/optimization/svsdf_backend/utils/flatness.hpp>
#include <esv_planner/optimization/svsdf_backend/utils/lbfgs.hpp>
#include <esv_planner/optimization/svsdf_backend/utils/debug_publisher.hpp>
#define TRAJ_ORDER 5

#define GNOW ros::Time::now()
#define DEBUGOUT(x) std::cout << x << std::endl

class OriTraj
{
public:
    // ros
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher debug_pub;
    minco::MINCO_S3NU minco;
    flatness::FlatnessMap flatmap;
    Trajectory<TRAJ_ORDER> step_traj;

    double rho;
    double vmax;
    double omgmax;
    double weight_v;
    double weight_omg;
    double weight_a;

    int integralRes;
    Eigen::Matrix3d initState;
    Eigen::Matrix3d finalState;

    Eigen::Matrix3Xd  accelerations; 
    Eigen::Matrix3Xd  ref_points;        
    std::vector<Eigen::Matrix3d> att_constraints;

    Eigen::Matrix3Xd  points;        
    Eigen::VectorXd times;
    Eigen::Matrix3Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX3d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;
    
    int pieceN;
    int spatialDim;
    int temporalDim;


    double smooth_fac;

    // ESDF collision penalty for midend
    const esv_planner::GridMap* grid_map_ = nullptr;
    double weight_esdf_ = 0.0;
    double esdf_margin_ = 0.30;

    Config conf;
    double weightPR;
    double weightAR;

    int iter = 0;
    void drawDebugTraj();

    static inline bool smoothedL1(const double &x,
                                  const double &mu,
                                  double &f,
                                  double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }

    static inline void forwardP(const Eigen::VectorXd &xi,
                                Eigen::Matrix3Xd &P)
    {
        const int sizeP = xi.size() / 3;
        P.resize(3, sizeP);
        for (int i = 0; i < sizeP; i++)
        {
            P.col(i) = xi.segment(3 * i, 3);
            // Replace NaN with zero to prevent MINCO from diverging
            for (int d = 0; d < 3; d++) {
                if (!std::isfinite(P(d, i))) P(d, i) = 0.0;
            }
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardP(const Eigen::Matrix3Xd &P,
                                 EIGENVEC &xi)
    {
        const int sizeP = P.cols();
        for (int i = 0; i < sizeP; ++i)
        {
            xi.segment(3 * i, 3) = P.col(i);
        }
        return;
    }

    // tao--->T
    static inline void forwardT(const Eigen::VectorXd &tau,
                                Eigen::VectorXd &T)
    {
        const int sizeTau = tau.size();
        T.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++)
        {
            if (!std::isfinite(tau(i))) {
                T(i) = 1.0;  // safe default
                continue;
            }
            T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            if (T(i) < 0.01) T(i) = 0.01;
            if (T(i) > 100.0) T(i) = 100.0;
        }
        return;
    }
    // T--->tao
    template <typename EIGENVEC>
    static inline void backwardT(const Eigen::VectorXd &T,
                                 EIGENVEC &tau)
    {
        const int sizeT = T.size();
        tau.resize(sizeT);
        for (int i = 0; i < sizeT; i++)
        {
            tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                     const Eigen::VectorXd &gradT,
                                     EIGENVEC &gradTau)
    {
        const int sizeTau = tau.size();
        gradTau.resize(sizeTau);
        double denSqrt;

        for (int i = 0; i < sizeTau; i++)
        {
            if (tau(i) > 0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradP(const Eigen::VectorXd &xi,
                                     const Eigen::Matrix3Xd &gradP,
                                     EIGENVEC &gradXi)
    {
        const int sizeP = gradP.cols();
        for (int i = 0; i < sizeP; ++i)
        {
            gradXi.segment(3 * i, 3) = gradP.col(i);
        }
        return;
    }


    bool inline grad_cost_dir(  const Eigen::Vector3d& pos,
                                const Eigen::Vector3d& acc,
                                Eigen::Vector3d &gradp,
                                Eigen::Vector3d &grada,
                                double &cost_p,
                                double &cost_a,
                                const int indexi)
    {
        cost_p = 0.0;
        cost_a = 0.0;
        gradp.setZero();
        grada.setZero();

        Eigen::Vector3d ref_pos = ref_points.col(indexi);
        Eigen::Vector3d ref_acc = accelerations.col(indexi);

        Eigen::Vector3d pos_diff       =       pos - ref_pos;
        const double d = pos_diff.norm();
        cost_p = d * d;
        if (d > 1e-12) {
            gradp = 2.0 * pos_diff;
        } else {
            gradp.setZero();
        }

        Eigen::Vector3d normed_ref_acc = ref_acc.normalized();
        Eigen::Vector3d zero_pose      = Eigen::Vector3d(0,0,1);

        double cost = 0.0;
        cost += cost_p;
        cost += cost_a;
        return (cost > 0);
    }
    

    static inline void addPosePenalty (  void *ptr,
                                         const Eigen::VectorXd &T,
                                         const Eigen::MatrixX3d &coeffs,
                                         double &cost,
                                         Eigen::VectorXd  &gradT,
                                         Eigen::MatrixX3d &gradC        )
    {
        OriTraj &obj = *(OriTraj *)ptr;
        Eigen::Vector3d pos, vel, acc, jer;
        Eigen::Vector3d grad_tmp_p, grad_tmp_a;

        double cost_tmp_p, cost_tmp_a;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
        Eigen::Matrix<double, 6, 3> gradViolaPc;
        double s1, s2, s3, s4, s5;
        double gradViolaPt;

        int constraints_count = obj.pieceN - 1;
        assert(constraints_count==obj.points.cols());
        int segment;
        const double alpha = 0.0;
        for (int i = 0; i < constraints_count; ++i) {

            segment = i + 1;
            const auto& c = coeffs.block<6, 3>(segment * 6, 0);

            s1 = alpha * T(segment);
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            beta0 << 1.0, s1, s2, s3, s4, s5;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;

            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc.setZero(); // not needed for grad_cost_dir position penalty

            if ( obj.grad_cost_dir(pos, acc, grad_tmp_p, grad_tmp_a, cost_tmp_p, cost_tmp_a, i) )  {

                gradViolaPc = beta0 * grad_tmp_p.transpose();
                gradViolaPt = alpha * grad_tmp_p.dot(vel);

                gradC.block<6, 3>(segment * 6, 0) +=  obj.weightPR * gradViolaPc;
                gradT(segment) += obj.weightPR * (cost_tmp_p  * gradViolaPt);

                cost += obj.weightPR * cost_tmp_p + obj.weightAR * cost_tmp_a;
              }
        }
    }


    static inline double costFunction(void *ptr,
                                      const Eigen::VectorXd &x,
                                      Eigen::VectorXd &g,
                                      double &p_cost)
    {
        OriTraj &obj = *(OriTraj *)ptr;
        const int dimTau = obj.temporalDim;
        const int dimXi  = obj.spatialDim;
        const double weightT = obj.rho;
        Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
        Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
        Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
        Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
        forwardT(tau, obj.times); // tao--->T
        forwardP(xi, obj.points);

        double cost = 0;
        obj.minco.setParameters(obj.points, obj.times);

        obj.minco.getEnergy(cost);
        double energy_cost = cost;
        if (!std::isfinite(cost)) {
            std::cout << "[MidEnd] NaN after getEnergy! T=" << obj.times.transpose() << std::endl;
            if (obj.points.cols() > 0) {
                std::cout << "[MidEnd] points_col0=" << obj.points.col(0).transpose() << std::endl;
            }
            g.setZero();
            return 1e18;
        }
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs); // ∂E/∂c
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);   // ∂E/∂T

        obj.minco.getTrajectory(obj.step_traj);

        addPosePenalty (   ptr,
                           obj.times,
                           obj.minco.getCoeffs(),
                           cost,
                           obj.partialGradByTimes,
                           obj.partialGradByCoeffs  );
        
        addTimeIntPenalty( ptr,
                           obj.times,
                           obj.minco.getCoeffs(),
                           cost,
                           obj.partialGradByTimes,
                           obj.partialGradByCoeffs);

        obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes, // ∂Cost/∂c,T -> ∂Cost/∂q,T
                                obj.gradByPoints, obj.gradByTimes);

        cost += weightT * obj.times.sum();

        obj.gradByTimes.array() += weightT;
        backwardGradT(tau, obj.gradByTimes, gradTau);
        backwardGradP(xi, obj.gradByPoints, gradXi);
        if (!std::isfinite(cost)) {
            double energy = 0;
            obj.minco.getEnergy(energy);
            std::cout << "[MidEnd] Non-finite cost! cost=" << cost
                      << " energy=" << energy
                      << " times_sum=" << obj.times.sum()
                      << " tau_range=[" << tau.minCoeff() << "," << tau.maxCoeff() << "]"
                      << " xi_range=[" << xi.minCoeff() << "," << xi.maxCoeff() << "]"
                      << " T_range=[" << obj.times.minCoeff() << "," << obj.times.maxCoeff() << "]"
                      << " points_finite=" << obj.points.allFinite()
                      << " gradT_finite=" << obj.gradByTimes.allFinite()
                      << std::endl;
            g.setZero();
            return 1e18;
        }
        // Sanitize gradient: if any component is non-finite, zero the whole gradient
        // to prevent L-BFGS from producing NaN in x on the next iteration
        if (!g.allFinite()) {
            g.setZero();
        }
        return cost;
    }



public:
    OriTraj() {}
    ~OriTraj() {}

    void setParam(ros::NodeHandle &nh, Config &config)
    {
        this->conf = config;
        this->nh.reset(new ros::NodeHandle(nh));

        weightPR   = config.weight_pr;
        weightAR   = config.weight_ar;
        rho            = config.rho_mid_end;
        vmax           = config.vmax;
        omgmax         = config.omgmax;
        weight_v       = config.weight_v;
        weight_omg     = config.weight_omg;
        weight_a       = config.weight_a;
        smooth_fac     = config.smoothingEps;
        integralRes    = config.integralIntervs;
        ROS_WARN_STREAM("==smooth_fac==" << smooth_fac);
        ROS_WARN_STREAM("==rho==" << rho);

        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
        // flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        debug_pub           = nh.advertise<visualization_msgs::Marker>("/ori_traj/debug_traj", 10);
    }

    bool getOriTraj(    const Eigen::Matrix3d initS,
                        const Eigen::Matrix3d finalS,
                        const std::vector<Eigen::Vector3d> &Q,
                        Eigen::VectorXd T,
                        std::vector<Eigen::Vector3d> acc_list,
                        std::vector<Eigen::Matrix3d> rot_list,
                        const int N,
                        Trajectory<TRAJ_ORDER> &traj,
                        Eigen::VectorXd& opt_x );
    
    static inline double costaltitude(const Eigen::Vector4d &wxyz_rotate,const Eigen::Matrix3d &rotate_ref)
    {
    const double w=wxyz_rotate(0);
    const double x=wxyz_rotate(1);
    const double y=wxyz_rotate(2);
    const double z=wxyz_rotate(3);
    const double a0=rotate_ref(0,0);
    const double a1=rotate_ref(0,1);
    const double a2=rotate_ref(0,2);
    const double b0=rotate_ref(1,0);
    const double b1=rotate_ref(1,1);
    const double b2=rotate_ref(1,2);
    const double c0=rotate_ref(2,0);
    const double c1=rotate_ref(2,1);
    const double c2=rotate_ref(2,2);
    double cost=2*a0*(2*y*y+2*z*z-1)+2*b1*(2*x*x+2*z*z-1)+2*c2*(2*x*x+2*y*y-1)+2*a1*(2*w*z-2*x*y)-2*a2*(2*w*y+2*x*z)-2*b0*(2*w*z+2*x*y)
    +2*b2*(2*w*x-2*y*z)+2*c0*(2*w*y-2*x*z)-2*c1*(2*w*x+y*z)+6;
    return cost;
    }
    static inline void gradaltitude(const Eigen::Vector4d &wxyz_rotate,const Eigen::Matrix3d &rotate_ref,double&dfdw,double&dfdx,double&dfdy,double&dfdz)
    {
    dfdw=0;
    dfdx=0;
    dfdy=0;
    dfdz=0;
    const double w=wxyz_rotate(0);
    const double x=wxyz_rotate(1);
    const double y=wxyz_rotate(2);
    const double z=wxyz_rotate(3);
    const double a0=rotate_ref(0,0);
    const double a1=rotate_ref(0,1);
    const double a2=rotate_ref(0,2);
    const double b0=rotate_ref(1,0);
    const double b1=rotate_ref(1,1);
    const double b2=rotate_ref(1,2);
    const double c0=rotate_ref(2,0);
    const double c1=rotate_ref(2,1);
    const double c2=rotate_ref(2,2);
    dfdw=4*(b2*x-a2*y+a1*z-c1*x-b0*z+c0*y);
    dfdx=4*(b2*w-a1*y+2*b1*x-c1*w-b0*y-a2*z+2*c2*x-c0*z);
    dfdy=4*(2*a0*y-a1*x-a2*w-b0*x+c0*w-b2*z+2*c2*y-c1*z);
    dfdz=4*(a1*w-b0*w-a2*x+2*a0*z-c0*x-b2*y+2*b1*z-c1*y);
    }

    static inline double WC2(const double x, double &dx)
    {
        if(x < -1){ dx = 0; return 0;}
        else if(x < -0.5) { 
            dx = 4*(x+1);
            return 2*(x+1)*(x+1);
        }
        else if(x < 0.5) { 
            dx = -4*x;
            return 1 - 2*x*x;
        }
        else if(x < 1) { 
            dx = 4*(x-1);
            return 2*(x-1)*(x-1);
        }
        else { dx = 0; return 0;}
    }

    static inline void addTimeIntPenalty( void *ptr,
                                          const Eigen::VectorXd &T,
                                          const Eigen::MatrixX3d &coeffs,
                                          double &cost,
                                          Eigen::VectorXd &gradT,
                                          Eigen::MatrixX3d &gradC)
    {
        // SE(2) ground-robot version: penalize xy-velocity and yaw-rate
        // directly from the MINCO polynomial coefficients.
        // Trajectory dimensions: row 0 = x, row 1 = y, row 2 = yaw.
        // No quadrotor flatness map needed.
        OriTraj &obj = *(OriTraj *)ptr;
        const double velSqrMax = obj.vmax * obj.vmax;
        const double omgSqrMax = obj.omgmax * obj.omgmax;
        const double weightVel = obj.weight_v;
        const double weightOmg = obj.weight_omg;
        const double weightAcc = obj.weight_a;

        Eigen::Vector3d pos, vel, acc;
        double step, alpha;
        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
        double violaVel, violaOmg;
        double violaVelPena, violaVelPenaD, violaOmgPena, violaOmgPenaD;
        double node, pena;

        const int pieceNum = T.size();
        const int integralResolution = obj.integralRes;
        const double smoothFactor = obj.smooth_fac;
        const double integralFrac = 1.0 / integralResolution;

        for (int i = 0; i < pieceNum; i++)
        {
            const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);
            step = T(i) * integralFrac;

            for (int j = 0; j <= integralResolution; j++)
            {
                s1 = j * step;
                s2 = s1 * s1;
                s3 = s2 * s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0(0) = 1.0; beta0(1) = s1; beta0(2) = s2; beta0(3) = s3; beta0(4) = s4; beta0(5) = s5;
                beta1(0) = 0.0; beta1(1) = 1.0; beta1(2) = 2.0*s1; beta1(3) = 3.0*s2; beta1(4) = 4.0*s3; beta1(5) = 5.0*s4;
                beta2(0) = 0.0; beta2(1) = 0.0; beta2(2) = 2.0; beta2(3) = 6.0*s1; beta2(4) = 12.0*s2; beta2(5) = 20.0*s3;
                beta3(0) = 0.0; beta3(1) = 0.0; beta3(2) = 0.0; beta3(3) = 6.0; beta3(4) = 24.0*s1; beta3(5) = 60.0*s2;

                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                pos = c.transpose() * beta0;
                const Eigen::Vector3d jer = c.transpose() * beta3;

                pena = 0.0;
                Eigen::Vector3d gradVelTotal = Eigen::Vector3d::Zero();

                // xy-velocity penalty: ||vel_xy||^2 - vmax^2
                const double vel_xy_sq = vel(0)*vel(0) + vel(1)*vel(1);
                violaVel = vel_xy_sq - velSqrMax;
                if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                {
                    gradVelTotal(0) += weightVel * violaVelPenaD * 2.0 * vel(0);
                    gradVelTotal(1) += weightVel * violaVelPenaD * 2.0 * vel(1);
                    pena += weightVel * violaVelPena;
                }

                // yaw-rate penalty: vel(2)^2 - omgmax^2
                // vel(2) = d(yaw)/dt = yaw rate for ground robot
                const double yaw_rate = vel(2);
                violaOmg = yaw_rate * yaw_rate - omgSqrMax;
                if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                {
                    gradVelTotal(2) += weightOmg * violaOmgPenaD * 2.0 * yaw_rate;
                    pena += weightOmg * violaOmgPena;
                }

                // xy-acceleration smoothness penalty: ||acc_xy||^2
                // Directly penalizes lateral oscillation
                Eigen::Vector3d gradAccTotal = Eigen::Vector3d::Zero();
                const double acc_xy_sq = acc(0)*acc(0) + acc(1)*acc(1);
                gradAccTotal(0) += weightAcc * 2.0 * acc(0);
                gradAccTotal(1) += weightAcc * 2.0 * acc(1);
                pena += weightAcc * acc_xy_sq;

                // ESDF collision penalty for xy position
                Eigen::Vector3d gradPosTotal = Eigen::Vector3d::Zero();
                if (obj.grid_map_ != nullptr && obj.weight_esdf_ > 0.0) {
                    const double esdf_val = obj.grid_map_->getEsdf(pos(0), pos(1));
                    const double violation = obj.esdf_margin_ - esdf_val;
                    double violaEsdfPena, violaEsdfPenaD;
                    if (smoothedL1(violation, smoothFactor, violaEsdfPena, violaEsdfPenaD)) {
                        const double h = obj.grid_map_->resolution();
                        const double dEdx = (obj.grid_map_->getEsdf(pos(0)+h, pos(1))
                                           - obj.grid_map_->getEsdf(pos(0)-h, pos(1))) / (2.0*h);
                        const double dEdy = (obj.grid_map_->getEsdf(pos(0), pos(1)+h)
                                           - obj.grid_map_->getEsdf(pos(0), pos(1)-h)) / (2.0*h);
                        gradPosTotal(0) = -obj.weight_esdf_ * violaEsdfPenaD * dEdx;
                        gradPosTotal(1) = -obj.weight_esdf_ * violaEsdfPenaD * dEdy;
                        pena += obj.weight_esdf_ * violaEsdfPena;
                    }
                }

                node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                alpha = j * integralFrac;

                // Write gradient directly (no flatmap.backward needed)
                gradC.block<6, 3>(i * 6, 0) += (beta0 * gradPosTotal.transpose()) *
                                                node * step;
                gradC.block<6, 3>(i * 6, 0) += (beta1 * gradVelTotal.transpose()) *
                                                node * step;
                // Acceleration gradient w.r.t. coefficients
                gradC.block<6, 3>(i * 6, 0) += (beta2 * gradAccTotal.transpose()) *
                                                node * step;

                gradT(i) += (gradPosTotal.dot(vel) + gradVelTotal.dot(acc) + gradAccTotal.dot(jer)) *
                                alpha * node * step +
                            node * integralFrac * pena;

                cost += node * step * pena;
            }
        }
    }

    static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls)
    {
        OriTraj &obj = *(OriTraj *)instance;
        obj.iter++;
        if (obj.iter <= 3 || obj.iter % 10 == 0) {
            std::cout << "[mid] iter=" << obj.iter << " cost=" << fx
                      << " T=" << obj.times.transpose() << std::endl;
        }
        if (obj.conf.enableearlyExit)
        {
            obj.drawDebugTraj();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(obj.conf.debugpause));
        }
        return k > 30;
    }

public:
    typedef shared_ptr<OriTraj> Ptr;
};




