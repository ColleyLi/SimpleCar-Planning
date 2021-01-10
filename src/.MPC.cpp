/**
 * Model predictive control with ACADO by manually propagation
 * to track the path generated manually by ompl
 * author: Jianfeng Cui
*/

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

USING_NAMESPACE_ACADO

int main()
{
    // Input reference path file and output trajectory file
    std::string states_file = "../data/simple_car_path_geometric.txt";
    std::ifstream xref(states_file);
    std::string output_states_dir = "../data/output_states.txt";
    std::ofstream output_states_file(output_states_dir);
    std::string output_controls_dir = "../data/output_controls.txt";
    std::ofstream output_controls_file(output_controls_dir);

    const double tTotal = 30;
    const int totalSteps = 60;
    const double dt = tTotal / totalSteps;
    const double wheelbase = 1.0;
    int num_waypoints = 0;

    // Store all reference path in a 2d vector
    std::vector<std::vector<double>> ref_states;
    if(!xref){
        std::cerr << "cannot open reference state file!" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::string line;
    while(std::getline(xref, line))
    {
        std::vector<double> s(3, 0);
        std::istringstream iss(line);
        for(std::vector<double>::iterator vit = s.begin(), vend = s.end(); vit != vend; vit++)
        {
            iss >> *vit;
        }
        ref_states.push_back(s);
        num_waypoints++;
    }
    if(ref_states.empty()){
        std::cerr << "failed to store the reference in ref_states!" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::vector<double> pose2D = ref_states[0];
    output_states_file << pose2D[0] << ' ' << pose2D[1] << ' ' << pose2D[2] << std::endl;
    
    // Manually start simulation
    for(int i = 0; i < totalSteps; i++)
    {
        std::cout << "Time at " << i << std::endl;
        // Parameters setting
        const double tStart = 0.0;
        const int numSteps = 20; // N = 20
        const double tEnd = tStart + numSteps * dt;

        // Variables
        DifferentialState x;
        DifferentialState y;
        DifferentialState theta;

        Control u0; // velocity
        Control u1; // steering angle

        // Differential equation
        DifferentialEquation f;
        f << dot(x) == u0 * cos(theta);
        f << dot(y) == u0 * sin(theta);
        f << dot(theta) == u0 * tan(u1) / wheelbase;  // the wheelbase length

        // OutputFcn identity;
        // DynamicSystem dynamicSystem(f, identity);

        // Process process(dynamicSystem, INT_RK45);
        // process.set(INTEGRATOR_TYPE, INT_EX_EULER);
        // process.set(NUM_INTEGRATOR_STEPS, numSteps);

        // Objective function
        Function h;
        h << x;
        h << y;
        h << theta;
        h << u0;
        h << u1;

        // Coefficient matrix
        DMatrix Q(5, 5);
        Q(0, 0) = 1.0;
        Q(1, 1) = 1.0;
        Q(2, 2) = 0.1;
        Q(3, 3) = 1e-3;
        Q(4, 4) = 1e-3;

        // Reference
        // DVector r(5);

        // Time grid, reference grid, and output states grid
        Grid timeGrid(tStart, tEnd, numSteps);

        VariablesGrid reference_grid(3, timeGrid);

        VariablesGrid output_states(3, timeGrid);
        std::cout << "here 1" << std::endl;
        for(int ii = 0; ii < numSteps; ii++)
        {
            int i_ref = i+ii < totalSteps ? i+ii : totalSteps;
            for(int s = 0; s < 3; s++){
                reference_grid(ii, s) = ref_states[i_ref][s];
            }
        }
        std::cout << reference_grid << std::endl;

        // Define an optimal control problem
        OCP ocp(timeGrid);

        ocp.minimizeLSQ(Q, h, reference_grid);

        ocp.subjectTo(f);
        ocp.subjectTo(AT_START, x == pose2D[0]);
        ocp.subjectTo(AT_START, y == pose2D[1]);
        ocp.subjectTo(AT_START, theta == pose2D[2]);

        ocp.subjectTo(0.0 <= x <= 20);
        ocp.subjectTo(0.0 <= y <= 20);
        ocp.subjectTo(-M_PI <= theta <= M_PI); // TODO: how to represent S1?
        ocp.subjectTo(-0.5 <= u0 <= 0.5);
        ocp.subjectTo(M_PI / 4 <= u1 <= M_PI / 4);
        std::cout << "here 3" << std::endl;
        // Define the solver
        OptimizationAlgorithm alg(ocp);
        std::cout << "here 3" << std::endl;
        // alg.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        // alg.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
        // alg.set(PRINTLEVEL, NONE);
        // alg.set(INTEGRATOR_TYPE, INT_EX_EULER);
        // alg.set(CG_USE_OPENMP, YES);
        // alg.set(NUM_INTEGRATOR_STEPS, 2 * numSteps);
        // alg.set(LEVENBERG_MARQUARDT, 1e-4);
        // alg.set(QP_SOLVER, QP_QPOASES);
        // alg.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);
        // alg.set(HOTSTART_QP, YES);

        alg.set(MAX_NUM_QP_ITERATIONS, 50);
        alg.set(MAX_NUM_ITERATIONS, 50);
        alg.set(INFEASIBLE_QP_HANDLING, IQH_STOP);
        alg.set( INTEGRATOR_TYPE, INT_RK45);
        alg.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
        alg.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        alg.set(KKT_TOLERANCE, 1e-8);
        std::cout << "here 3" << std::endl;
        // alg.initializeDifferentialStates(init_states);
        // alg.initializeControls(init_controls);

        // Controller controller(alg);

        // DVector x0(3);
        // x0(0) = 0.0;
        // x0(1) = 0.0;
        // x0(2) = M_PI / 2;
        // // controller.init(0.0, x_);
        // // controller.step(0.0, x_);
        // SimulationEnvironment sim(0.0, 20.0, process, controller);
        // sim.init(x0);
        // sim.run();
        if(!alg.solve()){
            return 0;
        }
        std::cout << "here 3" << std::endl;

        VariablesGrid diffStates;
        alg.getDifferentialStates(diffStates);

        VariablesGrid Controls;
        alg.getControls(Controls);

        // VariablesGrid diffStates;
        // sim.getProcessDifferentialStates(diffStates);

        // VariablesGrid Control;
        // sim.getFeedbackControl(Control);

        // GnuplotWindow window;
        // window.addSubplot(diffStates(0), "x");
        // window.addSubplot(diffStates(1), "y");
        // window.addSubplot(diffStates(2), "theta");
        // window.addSubplot(Controls(0), "u0");
        // window.addSubplot(Controls(1), "u1");
        // window.plot();

        // output_states_file << diffStates(0, 0) << ' ' <<
        //                         diffStates(0, 1) << ' ' << 
        //                             diffStates(0, 2) << std::endl;
        std::cout << "here 4" << std::endl;
        output_controls_file << Controls(0, 0) << ' ' <<
                                Controls(0, 1) << ' ' << std::endl;

        // Apply the calculated control
        pose2D[0] += Controls(0, 0) * dt * cos(pose2D[2]);
        pose2D[1] += Controls(0, 0) * dt * sin(pose2D[2]);
        pose2D[2] += Controls(0, 0) * dt * tan(Controls(0, 1)) / wheelbase;

        output_states_file << pose2D[0] << ' ' << pose2D[1] << ' ' << pose2D[2] << std::endl;
    }

    return 0;
}