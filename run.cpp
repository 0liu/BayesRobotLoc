#include "ekf_loc.h"


int main() {

    // Simulation parameters
    double dt=0.1; // simulation time step in seconds
    double init_x=3., init_y=3., init_theta=0;  // initial states
    double init_v=1.0, init_w=0.1;  // initial control variables
    double a1=0.2, a2=0.1;  // control variance coefficients
    double var_x=.8, var_y=.5;  // measurement variance
    
    robotloc::Simulator sim {dt, a1, a2, var_x, var_y,
                             init_v, init_w, init_x, init_y, init_theta};
    sim.run(120);
    return 0;
}


