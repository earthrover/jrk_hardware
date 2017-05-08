
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "jrk_hardware/jrk_hardware.h"

using std::cout;
using std::cerr;
using std::endl;
using std::exception;

int run(int argc, char **argv)
{
  std::vector<std::string> joints({ "A", "B" });
  std::vector<std::string> ports({ "/dev/ttyACM0", "/dev/ttyACM2" });

  cout << joints.size() << endl;

  cout << "Jrk Devices: [" << joints.size() << "]{ ";

  if (joints.size() > 0)
    cout << joints[0];
  for (int i = 1; i < joints.size(); i++)
  {
    cout << ", " << joints[i];
  }
  cout << " }" << std::endl;

  jrk_control::JrkHardware jrk(joints, ports);

  std::string debug = jrk.debug();

  cout << debug;
}

int main(int argc, char **argv)
{
  try
  {
    return run(argc, argv);
  }
  catch (exception &e)
  {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
