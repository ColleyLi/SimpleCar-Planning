#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

using namespace std;

USING_NAMESPACE_ACADO

int main()
{
  const double ts = 0.0; ///< Start time
  const double te = 1.0; ///< End time
  const int numSteps = 10; ///< number of steps for output trajectory
  const int totalSteps = 40; ///< number of steps in reference trajectory
  const int tfinal = 10; ///< Final time of reference trajectory

  //int prop_steps = int(double(numSteps)/te)/(totalSteps/tfinal);

  ofstream output_file = std::ofstream("output_states.txt");

  DifferentialState x;
  DifferentialState y;
  DifferentialState theta;

  Control U0; // velocity
  Control U1; // steering angle

  DifferentialEquation f;
  f << dot(x) == U0*cos(theta);
  f << dot(y) == U0*sin(theta);
  f << dot(theta) == U0*tan(U1) / 1.0;  // the wheelbase length

  Function h1;

  h1 << x;
  h1 << y;
  h1 << theta;

  DMatrix Q(3,3);
  Q.setIdentity();
  Q(0,0) = 2.0;
  Q(1,1) = 2.0;
  Q(2,2) = 1.0;

  //for (auto i=0; i < ?; i++)
//{

  Grid timeGrid(ts, te, numSteps);
  VariablesGrid output_states(3, timeGrid); ///< 3 stats variables
  VariablesGrid output_controls(2, timeGrid); ///< 2 control

  //VariablesGrid reference_grid(3, timeGrid);

  OCP ocp(timeGrid);

  ocp.subjectTo(f);
  ocp.subjectTo( -0.8 <= U0 <= 0.8 );
  ocp.subjectTo( -M_PI/4 <= U1 <= M_PI/4 );
  ocp.subjectTo( -M_PI <= theta <= M_PI );

  //ocp.minimizeLSQ(Q, h1, "path.txt");
  ocp.minimizeLSQ(Q, h1);

  RealTimeAlgorithm alg( ocp, (te-ts)/numSteps );

  DVector x0(3);
  x0.setZero();

  //alg.init( ts, x0 );
  //alg.solve( x0 );
  //alg.getDifferentialStates(output_states);
  //alg.getControls(output_controls);


  //output_states.print("output_states_test.txt");

  output_file << output_states(0,0) <<" "<<
                 output_states(0,1) <<" "<<
                 output_states(0,2) <<"\n";


//}


  return 0;
}
