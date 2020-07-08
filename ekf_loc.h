#ifndef __EKF_LOC_H_
#define __EKF_LOC_H_

#include <string>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


namespace robotloc {

    using std::vector;
    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Matrix2d;
    using Eigen::Matrix3d;
    using Matrix32d = Eigen::Matrix<double, 3, 2>;
    using Matrix23d = Eigen::Matrix<double, 2, 3>;


    class DeadReckon {
        /* Dead reckoning navigaton on 2D plane, with noisy control u. */
        public:
            DeadReckon(double init_x, double init_y, double init_theta);
            const Vector3d& get_pose() const;            
            const Vector3d& pose_estimate(double dt, const Vector2d& u);

        private:
            Vector3d m_mu {Vector3d::Zero()};  // estimates of states mu = [x, y, theta]
    };


    class ExtKalman {
        /* Extended Kalman filter for localization on 2D plane */
        public:
            ExtKalman(double init_x, double init_y, double init_theta,
                      double init_sigma);
            const Vector3d& get_pose() const;
            const Matrix3d& get_Sigma() const;
            const Vector3d& pose_estimate(double dt, const Vector2d& u, const Matrix2d& M,
                               const Vector2d& z, const Vector2d& q);
            
        private:
            Vector3d m_mu;  // states [x, y, theta] with uncertainty
            Matrix3d m_Sigma { Matrix3d::Zero() };  // Uncertainty matrix of states

            // EKF first step: Control update of states and its uncertainty.
            void ekf_control_update(double dt, const Vector2d& u,
                                    const Matrix2d& M);

            // EKF second step: Measure update of states and its uncertainty.
            void ekf_measure_update(double dt, const Vector2d& z,
                                    const Vector2d& q);
    };


    class Simulator {
        public:
            Simulator(double dt=0.1, double a1=0.2, double a2=0.1,
                      double var_x=0.5, double var_y=0.5,
                      double init_v=1.0, double init_w=0.1,
                      double init_x=0., double init_y=0., double init_theta=0.);

            // One-step motion update based on the linearized motion model.
            static void motion_update(double dt, const Vector2d& u, Vector3d& mu);

            // Execute control commands by change vector u values.
            void exec_control_commands();

            // Update true pose without uncertainty.
            void update_true_pose();

            // Read noisy control u with added Gaussian noise.
            void read_noisy_control();

            // Read noisy measurement z with added Gaussian noise.
            void read_noisy_measurement();

            // Run simulation
            void run(double sim_time_length, bool do_plot=true,
                     int plot_width=1100, int plot_height=950);

        private:
            // Gaussian noise generators
            static std::default_random_engine m_rand;
            static std::normal_distribution<double> m_norm;

            // Hyper parameters
            const double m_dt;  // time step (seconds)
            const double m_a1;  // hyperparameters a1, a2, a3, a4 for generating control uncertainty
            const double m_a2;  // a1 = a4, a2 = a3
            const double m_var_x;  // variance of measurement x
            const double m_var_y;  // variance of measurement y
            const double m_init_v, m_init_w;  // initial values of controls
            const double m_init_x, m_init_y, m_init_theta;  // initial values of states

            // True control vector and states without uncertainty
            Vector2d m_u0;  // control vector u = [v, w] without noise
            Vector3d m_mu;  // true states [x, y, theta] without noise

            // Noisy control and measure vectors with uncertainty
            Vector2d m_u;  // control vector u = [v, w] with uncertainty
            Matrix2d m_M;  // covaraince matrix of control vector u.
            Vector2d m_z;  // measurement vector z = [x, y] with uncertainty
            Vector2d m_q;  // variance of measurement vector q = [var_x, var_y]
    
            // Reset dynamics to initial values and clear up results.
            void reset();

            // Plot measurements, motion trajecture and estimates
            std::vector<double> sim_t, sim_x, sim_y, sim_theta;  // simulated states
            std::vector<double> z_x, z_y;  // noisy measurements
            std::vector<double> dr_x, dr_y, dr_theta;  // dead reckon estimates
            std::vector<double> ekf_x, ekf_y, ekf_theta;  // Kalman filter estimates            
            static void cov_eig_ellipse(const Matrix2d& CovXY,
                std::vector<double>& X, std::vector<double>& Y);
            void plot_init(int width, int height, bool do_plot=true);
            void plot_step(double t, const Vector2d& z, const Vector3d& sim_mu,
                           const Vector3d& dr_mu, const Vector3d& ekf_mu,
                           const Matrix3d& ekf_Sigma, bool do_plot=true);
    };

}

#endif // __EKF_LOC_H_
