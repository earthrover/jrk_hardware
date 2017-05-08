
#include <cstdio>
#include <iostream>
#include <string>

#include "jrk_control/jrk_serial.h"
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cerr;
using std::cout;
using std::endl;

void print_usage()
{
  cerr << "Usage: jrk_serial_single <serial port address> [timeout]" << endl;
}

int run(int argc, char **argv)
{
  if (argc < 2)
  {
    print_usage();
    return 0;
  }

  string port(argv[1]);

  unsigned int timeout = 500;

  if (argc == 3)
  {
#if defined(WIN32) && !defined(__MINGW32__)
    sscanf_s(argv[2], "%u", &timeout);
#else
    sscanf(argv[2], "%u", &timeout);
#endif
  }

  // auto ser(std::make_shared<serial::Serial>(port, 9600, serial::Timeout::simpleTimeout(timeout)));
  serial::Serial ser(port, 9600, serial::Timeout::simpleTimeout(timeout));

  cout << "Serial port at " << port << " open @9600 with " << timeout << "s timeout." << endl;

  cout << "Is the serial port open?";
  if (ser.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  pololu::Jrk jrk(ser);

  cout << "Sending 0xAA" << endl;
  jrk.sendBaudRateIndication();

  uint16_t feedback = 0;

  cout << "Reading Errors... ";
  feedback = jrk.getErrors();
  cout << "got " << feedback << endl;

  cout << "Reading feedback... ";
  feedback = jrk.getFeedback();
  cout << "got " << feedback << endl;

  cout << "Sending target 2047" << endl;
  jrk.setTarget(2047);

  cout << "Reading feedback... ";
  feedback = jrk.getFeedback();
  cout << "got " << feedback << endl;

  cout << "Sending target 2247" << endl;
  jrk.setTarget(2247);

  cout << "Reading feedback... ";
  feedback = jrk.getFeedback();
  cout << "got " << feedback << endl;

  cout << "Stopping motor" << endl;
  jrk.motorOff();
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
