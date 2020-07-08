#include <map>
#include <cmath>
#include "matplotlibcpp.h"
#include "ekf_loc.h"


// Dead Reckoning
namespace robotloc {

    DeadReckon::DeadReckon(double init_x, double init_y, double init_theta) {
        m_mu << init_x, init_y, init_theta;
    }

    const Vector3d& DeadReckon::get_pose() const {
        return m_mu;
    }            

    const Vector3d& DeadReckon::pose_estimate(double dt, const Vector2d &u) {
        Simulator::motion_update(dt, u, m_mu);
        return m_mu;
    }

}


// Extended Kalman Filter
namespace robotloc {

    ExtKalman::ExtKalman(double init_x, double init_y, double init_theta,
                         double init_sigma) {
        m_mu << init_x, init_y, init_theta;
        m_Sigma = init_sigma * Matrix3d::Identity();
    }

    const Vector3d& ExtKalman::get_pose() const {
        return m_mu;
    }

    const Matrix3d& ExtKalman::get_Sigma() const {
        return m_Sigma;
    }

    const Vector3d& ExtKalman::pose_estimate(
        double dt, const Vector2d& u, const Matrix2d& M,
        const Vector2d& z, const Vector2d& q) {
        ekf_control_update(dt, u, M);
        ekf_measure_update(dt, z, q);
        return m_mu;
    }    

    void ExtKalman::ekf_control_update(double dt, const Vector2d& u,
                                       const Matrix2d& M) {
        /*
         * Prediction step in Kalman filter estimation
         * Input:
         *   u: current control vector with uncertainty
         *   M: covariance matrix of control vector u
         *   mu_t-1: m_mu, states estimates from last run cycle
         *   Sigma_t-1: m_Sigma, states uncertainty from last run cycle
         */
        const double& v = u(0), w = u(1);
        auto& theta = m_mu(2);

        // Linearized Jacobin matrix of the motion model
        Matrix3d G;
        G << 1., 0., -dt * v * std::sin(theta),
             0., 1., dt * v * std::cos(theta),
             0., 0., 1.;
        
        // Linearized transformation matrix mapping uncertainty
        // from control space to state space
        Matrix32d V;
        V << dt * std::cos(theta), 0,
             dt * std::sin(theta), 0,
             0, dt;
        auto R = V * M * V.transpose();

        // Prediction
        Simulator::motion_update(dt, u, m_mu);  // predict \bar{mu}
        m_Sigma = G * m_Sigma * G.transpose() + R;  // predict \bar{Sigma}
    }

    void ExtKalman::ekf_measure_update(double dt, const Vector2d& z,
                                       const Vector2d& q) {
        /*
         * Measure update step in Kalman filter estimation
         * Input:
         *   z: current measure vector with uncertainty
         *   q: variance vector of measure vector z
         */

        // Get predicted measurement from predicted states
        Vector2d z_hat {m_mu(0), m_mu(1)};

        // Linearized Jacobian matrix of the measurement model
        Matrix23d H;
        H << 1, 0, 0,
             0, 1, 0;

        // Uncertainty corresponding to the predicted measurement z_hat
        Matrix2d Q = q.asDiagonal();
        Matrix2d S = H * m_Sigma * H.transpose() + Q;

        // Kalman gain
        Matrix32d K = m_Sigma * H.transpose() * S.inverse();

        // Update prediction with measurement
        m_mu += K * (z - z_hat);
        m_Sigma -= K * H * m_Sigma;
    }

}    


// Simulator
namespace robotloc {

    Simulator::Simulator(double dt, double a1, double a2,
                         double var_x, double var_y,
                         double init_v, double init_w,
                         double init_x, double init_y, double init_theta)
        : m_dt{dt}, m_a1{a1}, m_a2{a2}, m_var_x{var_x}, m_var_y{var_y},
          m_init_v{init_v}, m_init_w{init_w},
          m_init_x{init_x}, m_init_y(init_y), m_init_theta{init_theta} {
        // reset dynamics to initial values
        reset();
    }
    

    std::default_random_engine Simulator::m_rand{};
    std::normal_distribution<double> Simulator::m_norm{0, 1};

    void Simulator::reset() {
        m_u0 << m_init_v, m_init_w;
        m_mu << m_init_x, m_init_y, m_init_theta;
        m_u << m_init_v, m_init_w;
        m_z << m_init_x, m_init_y;

        // Initials of covariances M and q do not matter,
        // since M is overwritten in every cycle,
        // and Q does not change.
        m_M << 0., 0., 0., 0.;
        m_q << m_var_x, m_var_y;
    }

    void Simulator::motion_update(double dt, const Vector2d& u, Vector3d& mu) {
        const double& theta = mu(2);
        mu += Vector3d{u(0) * std::cos(theta), u(0) * std::sin(theta), u(1)} * dt;
    }

    void Simulator::exec_control_commands() {
        /* Change control vector u = (v, w) according to specific commands
        ** Here is only a simplified command set:
        **    1. keep translatonal velocity unchanged
        **    2. flip the sign of w to reverse moving direction
         */
        auto& theta = m_mu(2);
        if (theta > M_PI_2 || theta < -1.5 * M_PI)
            m_u0(1) *= -1;
    }

    void Simulator::update_true_pose() {
        Simulator::motion_update(m_dt, m_u0, m_mu);
    }

    void Simulator::read_noisy_control() {
        /* Add Gaussian noise to u = (v, w)
         */
        auto& v = m_u0(0), w = m_u0(1);
        double v_sqr = v * v, w_sqr = w * w;
        double epsilon_v = m_a1 * v_sqr + m_a2 * w_sqr;  // variance of v
        double epsilon_w = m_a2 * v_sqr + m_a1 * w_sqr;  // variance of w
        m_M << epsilon_v, 0.0, 0.0, epsilon_w;
        m_u(0) = v + std::sqrt(epsilon_v) * m_norm(m_rand);
        m_u(1) = w + std::sqrt(epsilon_w) * m_norm(m_rand);
    }

    void Simulator::read_noisy_measurement() {
        /* Add Gaussian noise to measurement z = (x, y)
         */
        m_z << m_mu(0) + std::sqrt(m_q[0]) * m_norm(m_rand),
               m_mu(1) + std::sqrt(m_q[1]) * m_norm(m_rand);
    }

    void Simulator::cov_eig_ellipse (
        const Matrix2d& CovXY,std::vector<double>& X, std::vector<double>& Y) {
        Eigen::EigenSolver<Matrix2d> es(CovXY);
        auto lambdas = es.eigenvalues();
        double a = std::sqrt(std::abs(lambdas(0))), b = std::sqrt(std::abs(lambdas(1)));
        auto v1 = es.eigenvectors().col(0);
        double vx = std::abs(v1(0)), vy = std::abs(v1(1));
        double theta = std::atan2(vy, vx);
        double r0 = std::cos(theta), r1 = std::sin(theta);  // rotation: [[r0, r1], [-r1, r0]]
        
        // Generate ellipse points
        double x, y;
        for (double r = 0.0; r < 2 * M_PI + 0.1; r += 0.1) {
            // parametric ellipse
            x = a * std::cos(r);
            y = b * std::sin(r);
            // rotation
            X.push_back(r0 * x + r1 * y);
            Y.push_back(-r1 * x + r0 * y);
        }
    }

    void Simulator::plot_init(int width, int height, bool do_plot) {
        sim_t.clear();
        z_x.clear();
        z_y.clear();
        sim_x.clear();
        sim_y.clear();
        sim_theta.clear();
        dr_x.clear();
        dr_y.clear();
        dr_theta.clear();
        ekf_x.clear();
        ekf_y.clear();
        ekf_theta.clear();

        if (!do_plot) return;

        namespace plt = matplotlibcpp;
        plt::figure_size(width, height);
        plt::subplots_adjust(std::map<std::string, double>{{"wspace", 0.3}, {"hspace", 0.3}});
    }

    void Simulator::plot_step(double t, const Vector2d& z,
                              const Vector3d& sim_mu, const Vector3d& dr_mu,
                              const Vector3d& ekf_mu, const Matrix3d& ekf_Sigma,
                              bool do_plot) {
        /*
        ** plot one-step of measurements, motion trajecture and estimates
        */
        if (!do_plot) return;

        // Update plot vectors
        sim_t.push_back(t);
        z_x.push_back(z(0));
        z_y.push_back(z(1));
        sim_x.push_back(sim_mu(0));
        sim_y.push_back(sim_mu(1));        
        sim_theta.push_back(sim_mu(2) * 180. / M_PI);
        dr_x.push_back(dr_mu(0));
        dr_y.push_back(dr_mu(1));
        dr_theta.push_back(dr_mu(2) * 180. / M_PI);
        ekf_x.push_back(ekf_mu(0));
        ekf_y.push_back(ekf_mu(1));
        ekf_theta.push_back(ekf_mu(2) * 180. / M_PI);

        // Plots
        namespace plt = matplotlibcpp;
        plt::cla();
        plt::subplot2grid(3, 3, 0, 0, 2, 3);
        plt::plot(sim_x, sim_y, std::map<std::string, std::string> {
                {"label", "True Motion"}, {"color", "blue"},
                {"linestyle", "-"}});
        plt::plot(dr_x, dr_y, std::map<std::string, std::string> {
                {"label", "Dead Reckoning Estimates"},
                {"color", "black"}, {"linestyle", "-"}});
        plt::plot(ekf_x, ekf_y, std::map<std::string, std::string> {
                {"label", "Ext. Kalman Filter Estimates"},
                {"color", "red"}, {"linestyle", "-"}});
        plt::plot(z_x, z_y, std::map<std::string, std::string> {
                {"label", "Measurements"}, {"color", "green"},
                {"linestyle", ""}, {"marker", "."}, {"markersize", "5"}});
        plt::grid(true);
        plt::axis("equal");
        plt::xlabel("X (m)"); plt::ylabel("Y (m)");
        plt::legend();
        plt::title("Bayes Filter Localization and Tracking");

        plt::subplot2grid(3, 3, 2, 0, 1, 1);
        auto sdx = std::sqrt(m_var_x), sdy = std::sqrt(m_var_y);
        auto sd_max = std::max(sdx, sdy), sd_min = std::min(sdx, sdy);
        plt::xlim(-sd_max*1.2, sd_max*1.4);
        plt::ylim(-sd_max*1.2, sd_max*1.4);
        plt::axis("square");
        plt::title("Uncertainty Ellipses & Heading");
        std::vector<double> X, Y;
        Matrix2d CovXY;
        CovXY << m_var_x, 0., 0., m_var_y;
        cov_eig_ellipse(CovXY, X, Y);
        plt::named_plot("Measure", X, Y, "slategray");
        X.clear(); Y.clear();
        CovXY = ekf_Sigma.topLeftCorner<2, 2>();
        cov_eig_ellipse(CovXY, X, Y);
        plt::named_plot("EKF", X, Y, "indigo");
        static_cast<void>( plt::arrow(0.0, 0.0,
                                      sd_min*std::cos(ekf_mu(2)),
                                      sd_min*std::sin(ekf_mu(2)),
                                      "b", "k", 0.12, 0.05));
        plt::legend();

        plt::subplot2grid(3, 3, 2, 1, 1, 2);
        plt::plot(sim_t, ekf_theta, std::map<std::string, std::string> {
                {"label", "Measurements"}, {"color", "tab:orange"},
                {"linestyle", "-"}, {"marker", ""}});
        plt::grid(true);
        plt::title("Estimated Yaw Angle");
        plt::xlabel("Time (s)"); plt::ylabel("Angle (Degree)");

        plt::pause(.000001);
    }

    void Simulator::run(double sim_time_length, bool do_plot,
                        int plot_width, int plot_height) {
        /*
        ** Run simulation
        */
        // Reset internals and instantialize estimators
        reset();
        DeadReckon dr {m_init_x, m_init_y, m_init_theta};
        ExtKalman ekf {m_init_x, m_init_y, m_init_theta, m_q.sum()/m_q.size()};

        // Set up plots
        plot_init(plot_width, plot_height, do_plot);
        // Plot initials
        plot_step(0, m_z, m_mu, dr.get_pose(), ekf.get_pose(), ekf.get_Sigma());

        // Loop over time steps
        for (double t = 0.; t < sim_time_length; t += m_dt) {
            // Simulator generates motion, measurement and uncertainties
            exec_control_commands();
            update_true_pose();
            read_noisy_control();
            read_noisy_measurement();
            // Dead reckoning estimates
            const Vector3d& dr_mu = dr.pose_estimate(m_dt, m_u);
            // Extended Kalman filter estimates
            const Vector3d& ekf_mu = ekf.pose_estimate(m_dt, m_u, m_M, m_z, m_q);
            const Matrix3d& ekf_Sigma = ekf.get_Sigma();
            // Plot
            plot_step(t+m_dt, m_z, m_mu, dr_mu, ekf_mu, ekf_Sigma);
        }
    }

}
