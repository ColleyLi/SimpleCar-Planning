#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
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
	f << dot(theta) == U0*tan(U1) / 1.0;  // the wheelbase length



    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK45 );


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h1;

    h1 << x << y << theta << U0 << U1;

    DMatrix Q(5,5);
    Q.setIdentity();
	Q(0,0) = 1.0;
	Q(1,1) = 1.0;
	Q(2,2) = 1.0;
        Q(3,3) = 1.0;
        Q(4,4) = 1.0;

    DVector r(5);
        r(0) = 9;
        r(1) = 9;
        r(2) = 0.75;
        r(3) = 0.2;
        r(4) = 0.2;


    Function h2;
    h2 << x << y << theta;

    DMatrix Q_end(3,3);
    Q_end.setIdentity();
	Q_end(0,0) = 0.1;
	Q_end(1,1) = 0.1;
	Q_end(2,2) = 0.1;

    //DVector r_end(3);
        r_end(0) = 18;
        r_end(1) = 18;
        r_end(2) = 1.5;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const int N = 10;
    const double dt = 0.1;
    const double t_end = t_start + N*dt;

    OCP ocp( t_start, t_end, N );

    ocp.minimizeLSQ( Q, h1, r);
    //ocp.minimizeLSQEndTerm( Q_end, h2, r_end);

	ocp.subjectTo( f );
	ocp.subjectTo( -10.0 <= U0 <= 10.0 );
        ocp.subjectTo( -M_PI/4 <= U1 <= M_PI/4 );
        ocp.subjectTo( -M_PI <= theta <= M_PI );


    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
        VariablesGrid myReference; myReference.read( "path.txt" );// read the reference
        StaticReferenceTrajectory reference( myReference );


	RealTimeAlgorithm alg( ocp, dt );
	Controller controller( alg, reference );


    // SETTING UP THE SIMULATION ENVIRONMENT:
    // --------------------------------------
	SimulationEnvironment sim( 0.0, 120.0, process, controller );

	DVector x0(3);
	x0(0) = 2.0;
	x0(1) = 2.0;
	x0(2) = 0.0;


	if (sim.init( x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );


    // ...AND PRINT THE RESULTS:
    // ----------------------------------------------------------
	VariablesGrid outputStates;
	VariablesGrid outputControls;

	sim.getProcessDifferentialStates( outputStates );
        sim.getFeedbackControl( outputControls );

        ofstream output_states = std::ofstream("output_states.txt");
        ofstream output_controls = std::ofstream("output_controls.txt");

        //outputStates.print( "output_states.txt" );

        for (auto i=0; i<outputStates.getDim()/outputStates.getNumRows()/outputStates.getNumCols(); i++)
        {
          output_states << outputStates(i,0) <<" "<<
                           outputStates(i,1) <<" "<<
                           outputStates(i,2) <<"\n";
        }

        for (auto i=0; i<outputControls.getDim()/outputControls.getNumRows()/outputControls.getNumCols(); i++)
        {
          output_controls << outputControls(i,0) <<" "<<
                             outputControls(i,1) <<"\n";
        }


    return 0;
}

