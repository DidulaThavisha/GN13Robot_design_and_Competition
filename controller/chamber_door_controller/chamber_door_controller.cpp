#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 32

using namespace webots;
using namespace std;

int main()
{
  Supervisor *supervisor = new Supervisor();
  Motor *motor = supervisor->getMotor("motor_chamber_door");

  // do this once only

  Node *piece_node = supervisor->getFromDef("piece_box");
  if (piece_node == NULL)
  {
    cerr << "No DEF MY_ROBOT node found in the current world file" << endl;
    exit(1);
  }
  Field *trans_field = piece_node->getField("translation");

  while (supervisor->step(TIME_STEP) != -1)
  {
    // this is done repeatedly
    const double *values = trans_field->getSFVec3f();
    // cout << "Chess piese is at position: " << values[0] << ' '
         // << values[1] << ' ' << values[2] << endl;

    if (-1.45 < values[0] && values[0] < -1.17 && -1.45 < values[2] && values[2] < -1.18)
    {
      motor->setPosition(0.4);
      // cout << "check mate" << endl;
    }
    else
    {
      motor->setPosition(0.0);
    }
  }

  delete supervisor;
  return 0;
}