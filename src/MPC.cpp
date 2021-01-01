#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

using namespace std;

USING_NAMESPACE_ACADO

int main()
{
    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState x;
	DifferentialState y;
	DifferentialState theta;

	Control U0; // velocity
        Control U1; // steering angle

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

	f << dot(x) == U0*cos(theta);
	f << dot(y) == U0*sin(theta);
	f << dot(theta) == U0*tan(U1) / 0.3;  // 0.3 is the wheelbase length



    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK78 );


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h1;

    h1 << x;
    h1 << y;
    h1 << theta;


    DMatrix Q(3,3);
    Q.setIdentity();
	Q(0,0) = 1.0;
	Q(1,1) = 1.0;
	Q(2,2) = 1.0;


    DVector r(3);
    // r.setAll( 0.0 );
        r(0) = 26.01760; 
        r(1) = -0.81004;
        r(2) = 3.14145;


    Function h2;

    h2 << x;
    h2 << y;
    h2 << theta;


    DMatrix Q_end(3,3);
    Q_end.setIdentity();
	Q_end(0,0) = 1.0;
	Q_end(1,1) = 1.0;
	Q_end(2,2) = 5.0;


    DVector r_end(3);
        // r.setAll( 0.0 );
        r_end(0) = 26.01760; 
        r_end(1) = -0.81004;
        r_end(2) = 3.14145;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 15.0;

    OCP ocp( t_start, t_end, 20 );

    ocp.minimizeLSQ( Q, h1, r );
    ocp.minimizeLSQEndTerm( Q_end, h2, r_end );

	ocp.subjectTo( f );

	ocp.subjectTo( -0.5 <= U0 <= 0.5 );
        ocp.subjectTo( -0.6 <= U1 <= 0.6 );
        ocp.subjectTo( -3.1416 <= theta <= 3.1416 );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp,0.1 );
	alg.set( KKT_TOLERANCE, 1e-04 );
        alg.set( INTEGRATOR_TYPE, INT_RK78);
        alg.set( PLOT_RESOLUTION, MEDIUM ) ;
	
        VariablesGrid myReference; myReference.read( "path.txt" );// read the reference
        StaticReferenceTrajectory reference( myReference );

	Controller controller( alg,reference );



    // SETTING UP THE SIMULATION ENVIRONMENT:
    // --------------------------------------
	SimulationEnvironment sim( 0.0,250.0,process,controller );

	DVector x0(3);
	x0(0) = 0.0;
	x0(1) = 0.0;
	x0(2) = 0.0;


	if (sim.init( x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );


    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
	VariablesGrid diffStates;
	sim.getProcessDifferentialStates( diffStates );

	VariablesGrid feedbackControl;
	sim.getFeedbackControl( feedbackControl );

	//GnuplotWindow window;
	//window.addSubplot( diffStates(0), "x [m]" );
	//window.addSubplot( diffStates(1), "y [m]" );
	//window.addSubplot( diffStates(2), "theta [rad]" );
	//window.addSubplot( feedbackControl(0),      "velocity [m/s]" );
	//window.addSubplot( feedbackControl(1),      "steering angle [rad]" );
	//window.plot();
        diffStates.print("MPC_path.txt");
        //diffStates.print();


    return 0;
}



