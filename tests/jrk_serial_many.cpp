#include <iostream>
#include <vector>

#include "jrk_hardware/jrk_serial.h"
#include "serial/serial.h"

using std::vector;
using std::string;
using std::exception;
using std::cerr;
using std::cout;
using std::endl;
using serial::Serial;
using pololu::Jrk;

void print_usage()
{
  cerr << "Usage: jrk_serial_many port [port ...]" << endl;
}

int run(int argc, char **argv)
{
  if (argc < 2)
  {
    print_usage();
    return 0;
  }

  cout << "Serial devices" << endl;
  cout << "==============" << endl;

  vector<Serial> serial_devices(argc - 1);
  auto timeout = serial::Timeout::simpleTimeout(500);

  for (int i = 0; i < argc - 1; i++)
  {
    string port(argv[i + 1]);
    serial_devices[i].setPort(port);
    serial_devices[i].setBaudrate(9600);
    serial_devices[i].setTimeout(timeout);
    serial_devices[i].open();
    cout << serial_devices[i].getPort() << " is";
    if (!serial_devices[i].isOpen())
      cout << " not";
    cout << " open" << endl;
  }

  cout << "==============" << endl;

  vector<Jrk> jrk_devices;
  for (auto &ser : serial_devices)
  {
    Jrk jrk(ser);
    jrk_devices.push_back(jrk);
  }

  cout << "Jrk objects initialised." << endl;

  cout << "Sending 0xAA" << endl;
  for (auto &jrk : jrk_devices)
  {
    jrk.sendBaudRateIndication();
  }

  uint16_t feedback = 0;

  cout << "Reading Errors...";
  for (auto &jrk : jrk_devices)
  {
    feedback = jrk.getErrors();
    cout << " got " << feedback;
  }
  cout << endl;

  cout << "Reading feedback... ";
  for (auto &jrk : jrk_devices)
  {
    feedback = jrk.getFeedback();
    cout << " got " << feedback;
  }
  cout << endl;

  cout << "Stopping motor(s)" << endl;
  for (auto &jrk : jrk_devices)
  {
    jrk.motorOff();
  }
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
