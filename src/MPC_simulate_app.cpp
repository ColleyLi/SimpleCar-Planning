/**
 * Model predictive control with ACADO using simulation environment
 * to track the path generated using ompl_app
 * author: Jianfeng Cui, Yulei Qiu
*/

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

USING_NAMESPACE_ACADO

int main()
{
    // Input reference path file and output trajectory file
    std::string states_file = "../data/simple_car_path_geometric_app.txt";
    std::ifstream xref(states_file);

    const double wheelbase = 10.0; // 1 m
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
    const double tTotal = 120.0;
    const int totalSteps = num_waypoints - 1;
    // const double dt = tTotal / totalSteps;

    // Parameters setting
    const double tStart = 0.0;
    const int numSteps = 25; // N = 30
    const double dt = 0.1;
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

    OutputFcn identity;
    DynamicSystem dynamicSystem(f, identity);

    Process process(dynamicSystem, INT_RK45);
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
    Q(2, 2) = 0.7;
    Q(3, 3) = 1e-6;
    Q(4, 4) = 1e-6;

    // Reference
    DVector r(5);

    // Time grid, reference grid, and output states grid
    Grid timeGrid(tStart, tEnd, numSteps);

    // Generate reference grid as the referece trajectory input later
    VariablesGrid reference_grid(3, tStart, tTotal, totalSteps);
    for(int i = 0; i < totalSteps; i++)
    {
        for(int s = 0; s < 3; s++){
            reference_grid(i, s) = ref_states[i][s];
        }
    }
    std::cout << reference_grid << std::endl;

    // Define an optimal control problem
    OCP ocp(timeGrid);

    ocp.minimizeLSQ(Q, h, r);
    
    ocp.subjectTo(f);
    // ocp.subjectTo(AT_START, x == pose2D[0]);
    // ocp.subjectTo(AT_START, y == pose2D[1]);
    // ocp.subjectTo(AT_START, theta == pose2D[2]);

    ocp.subjectTo(0.0 <= x <= 650.0);
    ocp.subjectTo(-440.0 <= y <= 0.0);
    ocp.subjectTo(-M_PI <= theta <= M_PI); // TODO: how to represent S1?
    ocp.subjectTo(-10.0 <= u0 <= 10.0);
    ocp.subjectTo(- M_PI / 3 <= u1 <= M_PI / 3);

    // Define the solver
    // OptimizationAlgorithm alg(ocp);
	RealTimeAlgorithm alg(ocp, dt);

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

    // alg.set(MAX_NUM_QP_ITERATIONS, 50);
    // alg.set(MAX_NUM_ITERATIONS, 50);
    alg.set(INFEASIBLE_QP_HANDLING, IQH_STOP);
    alg.set(INTEGRATOR_TYPE, INT_RK45);
    alg.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    alg.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    alg.set(KKT_TOLERANCE, 1e-8);

    // alg.initializeDifferentialStates(init_states);
    // alg.initializeControls(init_controls);

    StaticReferenceTrajectory reference(reference_grid);
    std::cout << reference_grid << std::endl;
    Controller controller(alg, reference);

    DVector x0(3);
    x0(0) = ref_states[0][0];
    x0(1) = ref_states[0][1];
    x0(2) = ref_states[0][2];

    // // controller.init(0.0, x_);
    // // controller.step(0.0, x_);

    SimulationEnvironment sim(0.0, tTotal, process, controller);

	if (sim.init(x0) != SUCCESSFUL_RETURN)
		exit(EXIT_FAILURE);
	if (sim.run() != SUCCESSFUL_RETURN)
		exit(EXIT_FAILURE);
    // if(!alg.solve()){
    //     return 0;
    // }


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

	VariablesGrid outputStatesApp;
	VariablesGrid outputControlsApp;

	sim.getProcessDifferentialStates( outputStatesApp );
    sim.getFeedbackControl( outputControlsApp );

    std::ofstream output_states_app = std::ofstream("../data/output_states_app.txt");
    std::ofstream output_controls_app = std::ofstream("../data/output_controls_app.txt");

    for (auto i=0; i<outputStatesApp.getDim()/outputStatesApp.getNumRows()/outputStatesApp.getNumCols(); i++)
    {
        output_states_app << outputStatesApp(i,0) <<" "<<
                        outputStatesApp(i,1) <<" "<<
                        outputStatesApp(i,2) <<"\n";
    }

    for (auto i=0; i<outputControlsApp.getDim()/outputControlsApp.getNumRows()/outputControlsApp.getNumCols(); i++)
    {
        output_controls_app << outputControlsApp(i,0) <<" "<<
                            outputControlsApp(i,1) <<"\n";
    }

    return 0;
}