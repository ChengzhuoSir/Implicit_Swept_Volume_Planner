#ifndef __LBFGS_HPP__
#define __LBFGS_HPP__

#include <Eigen/Core>
#include <iostream>
#include <vector>

namespace lbfgs
{
    /**
     * L-BFGS optimization parameters.
     *  Call lbfgs_load_default_parameters() to initialize this with default values.
     */
    typedef struct
    {
        /**
         * The number of corrections to approximate the inverse hessian matrix.
         *  The L-BFGS routine stores the computation results of previous \ref m
         *  iterations to approximate the inverse hessian matrix of the current
         *  iteration. This parameter controls the size of the limited memories
         *  (corrections). The default value is \c 6. Values less than \c 3 are
         *  not recommended. Large values will result in excessive computing time.
         */
        int mem_size;

        /**
         * Epsilon for convergence test.
         *  This parameter determines the accuracy with which the solution is to
         *  be found. A minimization terminates when
         *  ||g|| < \ref g_epsilon * max(1, ||x||),
         *  where ||.|| denotes the Euclidean (L2) norm. The default value is
         *  \c 1e-5.
         */
        double g_epsilon;

        /**
         * Distance for delta-based convergence test.
         *  This parameter determines the distance, in iterations, to compute
         *  the rate of decrease of the cost function. If the value of this
         *  parameter is zero, the delta-based convergence test will not
         *  be performed. The default value is \c 0.
         */
        int past;

        /**
         * Delta for convergence test.
         *  This parameter determines the minimum rate of decrease of the
         *  cost function. The optimization terminates when the following condition is
         *  satisfied:
         *  (f(x_{k-past}) - f(x_k)) / f(x_k) < \ref delta.
         *  The default value is \c 0.
         */
        double delta;

        /**
         * The maximum number of iterations.
         *  The lbfgs_optimize() function terminates when the iteration count
         *  exceeds this parameter. Setting this parameter to zero continues an
         *  optimization process until a convergence or error. The default value
         *  is \c 0.
         */
        int max_iterations;

        /**
         * The maximum number of trials in line search for each iteration.
         */
        int max_linesearch;

        /**
         * The minimum step of the line search routine.
         *  The default value is \c 1e-20. This value need not be modified unless
         *  the exponents are extremely large or small.
         */
        double min_step;

        /**
         * The maximum step of the line search routine.
         *  The default value is \c 1e+20. This value need not be modified unless
         *  the exponents are extremely large or small.
         */
        double max_step;

        /**
         * A parameter used in Armijo condition.
         *  The default value is \c 1e-4. This parameter should be greater
         *  than zero and less than \c 0.5.
         */
        double ftol;

        /**
         * A parameter used in Wolfe condition.
         *  The default value is \c 0.9. This parameter should be greater
         *  than the \ref ftol parameter and less than \c 1.0.
         */
        double wolfe;

    } lbfgs_parameter_t;

    /**
     * Return values of lbfgs_optimize().
     *  Roughly speaking, a negative value indicates an error.
     */
    enum
    {
        /** L-BFGS reaches convergence. */
        LBFGS_CONVERGENCE = 0,
        /** L-BFGS satisfies stopping criteria. */
        LBFGS_STOP,
        /** The iteration has been canceled by the monitor callback. */
        LBFGS_CANCELED,

        /** Unknown error. */
        LBFGSERR_UNKNOWNERROR = -1024,
        /** Invalid number of variables specified. */
        LBFGSERR_INVALID_N,
        /** Invalid parameter lbfgs_parameter_t::mem_size specified. */
        LBFGSERR_INVALID_MEMSIZE,
        /** Invalid parameter lbfgs_parameter_t::g_epsilon specified. */
        LBFGSERR_INVALID_GEPSILON,
        /** Invalid parameter lbfgs_parameter_t::past specified. */
        LBFGSERR_INVALID_PAST,
        /** Invalid parameter lbfgs_parameter_t::delta specified. */
        LBFGSERR_INVALID_DELTA,
        /** Invalid parameter lbfgs_parameter_t::min_step specified. */
        LBFGSERR_INVALID_MINSTEP,
        /** Invalid parameter lbfgs_parameter_t::max_step specified. */
        LBFGSERR_INVALID_MAXSTEP,
        /** Invalid parameter lbfgs_parameter_t::ftol specified. */
        LBFGSERR_INVALID_FTOL,
        /** Invalid parameter lbfgs_parameter_t::wolfe specified. */
        LBFGSERR_INVALID_WOLFE,
        /** The line search step became too small. */
        LBFGSERR_OUTOFINTERVAL,
        /** The line search step became too large. */
        LBFGSERR_OUTOFMAXSTEP,
        /** The line search routine failed (maximum number of trials reached). */
        LBFGSERR_MAXIMUMLINESEARCH,
        /** The line search routine failed (maximum number of iterations reached). */
        LBFGSERR_MAXIMUMITERATION,
        /** The line search routine failed (Wolfe condition satisfied only partially). */
        LBFGSERR_WOLFE_PARTIAL,
        /** The algorithm routine failed (NaN or Inf encountered). */
        LBFGSERR_INVALID_PARAMETERS,
    };

    /**
     * Callback interface to provide objective function and gradient availabilities.
     *
     *  The lbfgs_optimize() function call this function to obtain the values of objective
     *  function and its gradients when needed. A client program must implement
     *  this function to provide these fields.
     *
     *  @param  instance    The user data sent for lbfgs_optimize().
     *  @param  x           The current values of variables.
     *  @param  g           The gradient vector. The callback function must fill the gradient
     *                      values calculated at the current variables.
     *  @param  p_cost      Optional pointer to the current objective function value.
     *  @return             The value of the objective function for the current variables.
     */
    typedef double (*lbfgs_evaluate_t)(void *instance,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g,
                                       double &p_cost);

    /**
     * Callback interface to provide objective function and gradient availabilities.
     *
     *  The lbfgs_optimize() function call this function to obtain the values of objective
     *  function and its gradients when needed. A client program must implement
     *  this function to provide these fields.
     *
     *  @param  instance    The user data sent for lbfgs_optimize().
     *  @param  x           The current values of variables.
     *  @param  step        The current values of step.
     *  @return             The boundary value of the step.
     */
    typedef double (*lbfgs_stepbound_t)(void *instance,
                                        const Eigen::VectorXd &x,
                                        const Eigen::VectorXd &step);

    /**
     * Callback interface to receive the progress of the optimization process.
     *
     *  The lbfgs_optimize() function call this function for each iteration.
     *  A client program can implement this function to receive information on the
     *  progress of the optimization process.
     *
     *  @param  instance    The user data sent for lbfgs_optimize().
     *  @param  x           The current values of variables.
     *  @param  g           The current values of gradients.
     *  @param  fx          The current values of cost.
     *  @param  step        The current values of step.
     *  @param  k           The iteration count.
     *  @param  ls          The line search trials.
     *  @return             zero to continue the optimization process. Returning a
     *                      non-zero value will terminate the optimization process.
     */
    typedef int (*lbfgs_progress_t)(void *instance,
                                    const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &g,
                                    const double fx,
                                    const double step,
                                    const int k,
                                    const int ls);

    /**
     * Initialize L-BFGS parameters to the default values.
     *
     *  @param  param       The struct to be initialized.
     */
    inline void lbfgs_load_default_parameters(lbfgs_parameter_t &param)
    {
        param.mem_size = 6;
        param.g_epsilon = 1e-5;
        param.past = 0;
        param.delta = 0.0;
        param.max_iterations = 0;
        param.max_linesearch = 40;
        param.min_step = 1e-20;
        param.max_step = 1e+20;
        param.ftol = 1e-4;
        param.wolfe = 0.9;
    }

    /**
     * Start a L-BFGS optimization.
     */
    inline int lbfgs_optimize(Eigen::VectorXd &x,
                              double &final_cost,
                              lbfgs_evaluate_t proc_evaluate,
                              lbfgs_stepbound_t proc_stepbound,
                              lbfgs_progress_t proc_progress,
                              void *instance,
                              const lbfgs_parameter_t &param)
    {
        int ret = 0;
        int n = x.size();
        int m = param.mem_size;

        /* Check the parameters for errors. */
        if (n <= 0)
            return LBFGSERR_INVALID_N;
        if (m <= 0)
            return LBFGSERR_INVALID_MEMSIZE;
        if (param.g_epsilon < 0.0)
            return LBFGSERR_INVALID_GEPSILON;
        if (param.past < 0)
            return LBFGSERR_INVALID_PAST;
        if (param.delta < 0.0)
            return LBFGSERR_INVALID_DELTA;
        if (param.min_step < 0.0)
            return LBFGSERR_INVALID_MINSTEP;
        if (param.max_step < param.min_step)
            return LBFGSERR_INVALID_MAXSTEP;
        if (param.ftol < 0.0 || 0.5 < param.ftol)
            return LBFGSERR_INVALID_FTOL;
        if (param.wolfe < param.ftol || 1.0 < param.wolfe)
            return LBFGSERR_INVALID_WOLFE;

        /* Initialize the limited memory L-BFGS buffer. */
        Eigen::MatrixXd lm_s = Eigen::MatrixXd::Zero(n, m);
        Eigen::MatrixXd lm_y = Eigen::MatrixXd::Zero(n, m);
        Eigen::VectorXd lm_ys = Eigen::VectorXd::Zero(m);
        Eigen::VectorXd lm_alpha = Eigen::VectorXd::Zero(m);

        /* Evaluate the function value and its gradient. */
        Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
        double p_cost = 0.0;
        double fx = proc_evaluate(instance, x, g, p_cost);

        /* Store the current cost function value. */
        std::vector<double> fx_past;
        if (0 < param.past)
            fx_past.push_back(fx);

        /* Compute the search direction. */
        /* d = -g, since the initial hessian is an identity matrix. */
        Eigen::VectorXd d = -g;

        /* Compute the L2 norm of the variable and the gradient. */
        double xnorm = x.norm();
        double gnorm = g.norm();

        /* If the initial variable is already a stationary point, then terminate. */
        if (gnorm <= param.g_epsilon * std::max(1.0, xnorm))
        {
            final_cost = fx;
            return LBFGS_CONVERGENCE;
        }

        /* The step size for the first iteration. */
        double step = 1.0 / d.norm();

        int k = 1;
        int end = 0;
        double olddnorm = d.norm();
        for (;;)
        {
            /* Store the current values of x and g. */
            Eigen::VectorXd xp = x;
            Eigen::VectorXd gp = g;

            /* If a step bound callback is provided, then compute the boundary. */
            double step_bound = param.max_step;
            if (proc_stepbound)
                step_bound = proc_stepbound(instance, x, d);

            /* Perform the line search. */
            double step_max = std::min(param.max_step, step_bound);
            double step_min = param.min_step;

            int count = 0;
            double dginit = g.dot(d);
            double finit = fx;

            /* If the initial search direction is not a descent direction, then terminate. */
            if (dginit > 0.0)
            {
                // return LBFGSERR_UNKNOWNERROR;
                d = -g;
                dginit = g.dot(d);
            }

            for (;;)
            {
                x = xp + step * d;
                fx = proc_evaluate(instance, x, g, p_cost);
                ++count;

                if (fx > finit + param.ftol * step * dginit)
                {
                    step_max = step;
                }
                else
                {
                    /* The Armijo condition is satisfied. */
                    if (g.dot(d) < param.wolfe * dginit)
                    {
                        step_min = step;
                    }
                    else
                    {
                        /* The Wolfe condition is satisfied. */
                        break;
                    }
                }

                if (count >= param.max_linesearch)
                {
                    // ret = LBFGSERR_MAXIMUMLINESEARCH;
                    break;
                }

                if (step_max <= step_min)
                {
                    // ret = LBFGSERR_OUTOFINTERVAL;
                    break;
                }

                if (step_max < param.max_step)
                {
                    step = 0.5 * (step_min + step_max);
                }
                else
                {
                    step *= 2.0;
                }

                if (step < step_min || step > step_max)
                {
                    // ret = LBFGSERR_OUTOFINTERVAL;
                    break;
                }
            }

            if (ret < 0)
            {
                x = xp;
                break;
            }

            /* Compute the L2 norm of the variable and the gradient. */
            xnorm = x.norm();
            gnorm = g.norm();

            /* Report the progress. */
            if (proc_progress)
            {
                if ((ret = proc_progress(instance, x, g, fx, step, k, count)))
                    break;
            }

            /* Convergence test. */
            if (gnorm <= param.g_epsilon * std::max(1.0, xnorm))
            {
                ret = LBFGS_CONVERGENCE;
                break;
            }

            /* Delta-based convergence test. */
            if (0 < param.past)
            {
                if (param.past <= k)
                {
                    double rate = (fx_past[k % param.past] - fx) / fx;
                    if (rate < param.delta)
                    {
                        ret = LBFGS_STOP;
                        break;
                    }
                    fx_past[k % param.past] = fx;
                }
                else
                {
                    fx_past.push_back(fx);
                }
            }

            /* If the number of iterations exceeds the limit, then terminate. */
            if (0 < param.max_iterations && param.max_iterations <= k)
            {
                ret = LBFGSERR_MAXIMUMITERATION;
                break;
            }

            /* Update the L-BFGS limited memory buffer. */
            Eigen::VectorXd s = x - xp;
            Eigen::VectorXd y = g - gp;
            double ys = y.dot(s);
            double yy = y.dot(y);

            if (ys > 1e-10 * s.dot(s))
            {
                lm_s.col(end) = s;
                lm_y.col(end) = y;
                lm_ys(end) = ys;

                end = (end + 1) % m;
            }

            /* Compute the search direction d. */
            d = -g;
            int j = end;
            for (int i = 0; i < std::min(k, m); ++i)
            {
                j = (j + m - 1) % m;
                lm_alpha(j) = lm_s.col(j).dot(d) / lm_ys(j);
                d -= lm_alpha(j) * lm_y.col(j);
            }

            d *= ys / yy;

            for (int i = 0; i < std::min(k, m); ++i)
            {
                double beta = lm_y.col(j).dot(d) / lm_ys(j);
                d += (lm_alpha(j) - beta) * lm_s.col(j);
                j = (j + 1) % m;
            }

            /* The step size for the next iteration. */
            step = 1.0;
            ++k;

            /* Check if the search direction is a descent direction. */
            if (g.dot(d) > 0.0)
            {
                Eigen::VectorXd g_recover;
                double p_cost_new;
                if( !std::isfinite(fx) )//防止d出现nan，fx由于计算出现nan，则重新利用一阶信息
                {
                    double fx_ = proc_evaluate(instance, x, g_recover, p_cost_new);
                    d=-g_recover;
                    g=g_recover;//重新利用一阶信息
                    double d_norm = d.norm();
                    if (std::isfinite(d_norm) && d_norm > 1e-9) {
                        d.normalize();
                        d *= olddnorm; // 重新求取方向
                    } else {
                        return LBFGS_CANCELED;
                    }
                    std::cout << "\033[35m there " << "\033[0m" << std::endl;
                    end = (end + 1) % m; // 防止除0操作
                }
                std::cout << "\033[36m with norm:" << d.norm() << "\033[0m" << std::endl;
                olddnorm=d.norm();//打补丁
                /* The search direction d is ready. We try step = 1 first. */
                step = 1.0;
            }
        }

        /* Return the final value of the cost function. */
        final_cost = fx;

        return ret;
    }

    /**
     * Get string description of an lbfgs_optimize() return code.
     *
     *  @param err          A value returned by lbfgs_optimize().
     */
    inline const char *lbfgs_strerror(const int err)
    {
        switch (err)
        {
        case LBFGS_CONVERGENCE:
            return "Success: reached convergence (g_epsilon).";

        case LBFGS_STOP:
            return "Success: met stopping criteria (past f decrease less than delta).";

        case LBFGS_CANCELED:
            return "The iteration has been canceled by the monitor callback.";

        case LBFGSERR_UNKNOWNERROR:
            return "Unknown error.";

        case LBFGSERR_INVALID_N:
            return "Invalid number of variables specified.";

        case LBFGSERR_INVALID_MEMSIZE:
            return "Invalid parameter lbfgs_parameter_t::mem_size specified.";

        case LBFGSERR_INVALID_GEPSILON:
            return "Invalid parameter lbfgs_parameter_t::g_epsilon specified.";

        case LBFGSERR_INVALID_PAST:
            return "Invalid parameter lbfgs_parameter_t::past specified.";

        case LBFGSERR_INVALID_DELTA:
            return "Invalid parameter lbfgs_parameter_t::delta specified.";

        case LBFGSERR_INVALID_MINSTEP:
            return "Invalid parameter lbfgs_parameter_t::min_step specified.";

        case LBFGSERR_INVALID_MAXSTEP:
            return "Invalid parameter lbfgs_parameter_t::max_step specified.";

        case LBFGSERR_INVALID_FTOL:
            return "Invalid parameter lbfgs_parameter_t::ftol specified.";

        case LBFGSERR_INVALID_WOLFE:
            return "Invalid parameter lbfgs_parameter_t::wolfe specified.";

        case LBFGSERR_OUTOFINTERVAL:
            return "The line search step became too small.";

        case LBFGSERR_OUTOFMAXSTEP:
            return "The line search step became too large.";

        case LBFGSERR_MAXIMUMLINESEARCH:
            return "The line search routine failed (maximum number of trials reached).";

        case LBFGSERR_MAXIMUMITERATION:
            return "The line search routine failed (maximum number of iterations reached).";

        case LBFGSERR_WOLFE_PARTIAL:
            return "The line search routine failed (Wolfe condition satisfied only partially).";

        case LBFGSERR_INVALID_PARAMETERS:
            return "The algorithm routine failed (NaN or Inf encountered).";

        default:
            return "Unknown error.";
        }
    }
}

#endif
