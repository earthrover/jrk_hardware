POLOLU CONFIGURATION
===============

Here is the configuration for the Pololus.

We have them chained. The first pololu is normally the steering.
We connect the TX and RX from the top pololu into the bottom and reprogram them with the following

## Steering module

```
INITIALIZED	0
INPUT_MODE	SERIAL
INPUT_MINIMUM	0
INPUT_MAXIMUM	4095
OUTPUT_MINIMUM	0
OUTPUT_NEUTRAL	2048
OUTPUT_MAXIMUM	4095
INPUT_INVERT	0
INPUT_SCALING_DEGREE	0
INPUT_POWER_WITH_AUX	0
INPUT_ANALOG_SAMPLES_EXPONENT	5
INPUT_DISCONNECT_MINIMUM	0
INPUT_DISCONNECT_MAXIMUM	4095
INPUT_NEUTRAL_MAXIMUM	2048
INPUT_NEUTRAL_MINIMUM	2048
SERIAL_MODE	USB_DUAL_PORT
SERIAL_FIXED_BAUD_RATE	9600
SERIAL_TIMEOUT	0
SERIAL_ENABLE_CRC	0
SERIAL_NEVER_SUSPEND	0
SERIAL_DEVICE_NUMBER	11
FEEDBACK_MODE	ANALOG
FEEDBACK_MINIMUM	1396
FEEDBACK_MAXIMUM	2700
FEEDBACK_INVERT	1
FEEDBACK_POWER_WITH_AUX	0
FEEDBACK_DEAD_ZONE	2
FEEDBACK_ANALOG_SAMPLES_EXPONENT	5
FEEDBACK_DISCONNECT_MINIMUM	0
FEEDBACK_DISCONNECT_MAXIMUM	4095
PROPORTIONAL_MULTIPLIER	1
PROPORTIONAL_EXPONENT	1
INTEGRAL_MULTIPLIER	307
INTEGRAL_EXPONENT	8
DERIVATIVE_MULTIPLIER	1
DERIVATIVE_EXPONENT	2
PID_PERIOD	10
PID_INTEGRAL_LIMIT	1000
PID_RESET_INTEGRAL	0
MOTOR_PWM_FREQUENCY	0
MOTOR_INVERT	0
MOTOR_MAX_DUTY_CYCLE_WHILE_FEEDBACK_OUT_OF_RANGE	600
MOTOR_MAX_ACCELERATION_FORWARD	600
MOTOR_MAX_ACCELERATION_REVERSE	600
MOTOR_MAX_DUTY_CYCLE_FORWARD	600
MOTOR_MAX_DUTY_CYCLE_REVERSE	600
MOTOR_MAX_CURRENT_FORWARD	81
MOTOR_MAX_CURRENT_REVERSE	81
MOTOR_CURRENT_CALIBRATION_FORWARD	37
MOTOR_CURRENT_CALIBRATION_REVERSE	37
MOTOR_BRAKE_DURATION_FORWARD	0
MOTOR_BRAKE_DURATION_REVERSE	0
MOTOR_COAST_WHEN_OFF	1
ERROR_ENABLE	0
ERROR_LATCH	0
```

## Power module

```
INITIALIZED	0
INPUT_MODE	SERIAL
INPUT_MINIMUM	0
INPUT_MAXIMUM	4095
OUTPUT_MINIMUM	0
OUTPUT_NEUTRAL	2048
OUTPUT_MAXIMUM	4095
INPUT_INVERT	0
INPUT_SCALING_DEGREE	0
INPUT_POWER_WITH_AUX	0
INPUT_ANALOG_SAMPLES_EXPONENT	5
INPUT_DISCONNECT_MINIMUM	0
INPUT_DISCONNECT_MAXIMUM	4095
INPUT_NEUTRAL_MAXIMUM	2048
INPUT_NEUTRAL_MINIMUM	2048
SERIAL_MODE	UART_FIXED_BAUD_RATE
SERIAL_FIXED_BAUD_RATE	9600
SERIAL_TIMEOUT	0
SERIAL_ENABLE_CRC	0
SERIAL_NEVER_SUSPEND	0
SERIAL_DEVICE_NUMBER	11
FEEDBACK_MODE	TACHOMETER
FEEDBACK_MINIMUM	1596
FEEDBACK_MAXIMUM	2500
FEEDBACK_INVERT	0
FEEDBACK_POWER_WITH_AUX	0
FEEDBACK_DEAD_ZONE	0
FEEDBACK_ANALOG_SAMPLES_EXPONENT	8
FEEDBACK_DISCONNECT_MINIMUM	1596
FEEDBACK_DISCONNECT_MAXIMUM	2500
PROPORTIONAL_MULTIPLIER	819
PROPORTIONAL_EXPONENT	12
INTEGRAL_MULTIPLIER	2
INTEGRAL_EXPONENT	0
DERIVATIVE_MULTIPLIER	0
DERIVATIVE_EXPONENT	0
PID_PERIOD	90
PID_INTEGRAL_LIMIT	1000
PID_RESET_INTEGRAL	0
MOTOR_PWM_FREQUENCY	0
MOTOR_INVERT	0
MOTOR_MAX_DUTY_CYCLE_WHILE_FEEDBACK_OUT_OF_RANGE	600
MOTOR_MAX_ACCELERATION_FORWARD	150
MOTOR_MAX_ACCELERATION_REVERSE	150
MOTOR_MAX_DUTY_CYCLE_FORWARD	600
MOTOR_MAX_DUTY_CYCLE_REVERSE	600
MOTOR_MAX_CURRENT_FORWARD	81
MOTOR_MAX_CURRENT_REVERSE	81
MOTOR_CURRENT_CALIBRATION_FORWARD	37
MOTOR_CURRENT_CALIBRATION_REVERSE	37
MOTOR_BRAKE_DURATION_FORWARD	0
MOTOR_BRAKE_DURATION_REVERSE	0
MOTOR_COAST_WHEN_OFF	1
ERROR_ENABLE	0
ERROR_LATCH	0

```

## Have fun!


```
----------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------
----------------------+ossssso+:----------/////::---------------------------------------------------
----------------------smdmdmNmyy+/::///ymdmNmmmmmmdh/:+/--------------------------------------------
-----------------------s:ymmmmo::/:----dNmdNmmmmmmNmm+++:://////osyyysoo+/--------ohhysssso++-------
----------------------:+--dmmmdyo::/::+mmddddmNNNNNNNh/:::----:sdddhhhhhdy-----syodmmmNNNmmmdd:-----
----------------------:+-:dddodmdo/:/yydmmNNNNNmmddhyssssssssyyyoydddms:s/---:/NmdmNmmmmdmNmmNh+----
--------:--yyyysso++/://-/ddy-:dmdo-/hmhmmmdhyysssssssssssyhhyyhmmmmmd//yyyyyydmmmdddmmNmmNNNNmd+---
-------hhsyNNNNNmmmmddh/-odmo--:dmdo+hhyysssssssssssysyyhdddddmNNmmmdddddyddmmmmmmNNNmmNNNNNNNNNy---
....--/mmmmmddmmmmNNNmNo:sdm//+osyssssssssssssssyyyhmmdmmmmmmmmmmmmmdddmmymmNmmmNNmmhdmNNmddmNNNs---
...-/+ddhddmmmmNmNNNNNmhyyhdsddddhhhhhhhhhhyhhddddmmmmNNmmmmmmmmmmmmdddmNydNNNNNNNmmmmNNdhyyymmNmo--
.../hhmmmmmmdmmNNNNNNhyssss+/ydmdddddddddmdhddmNNddmmmmmmdmmmmmNmdmmmddmdshmNNNNmmNmmNNmhyyyyhNNNy--
...:mmdmNmdddmNNNNmNNhhyyyysoydmmdddddy:/yyssyhmmmdddddmmmmmNNNNmddmmddddyhmNNNNNNNmmNNmyyyyyhNNm:--
...ymmNNmmmmNNNNmdmNmdmddddddddmmmmddmdhyhyyddmmddmmNmmmmNNNNNNNNNNmmmddmyhNNNNNNNNNmNNdyyyyyhmNh---
.-ymmmmmmmmdmNNNdmmmNmNNNNNhyyddmmmmmdhhddddmmmmNNNmdhdmNNNmmmmNNNNmmmmmNNmNNNNNNNNNNNNmhyyyymmNm---
./mmmNNNNmddmNNdmmdhhhddmNNyyyddmmmmmdyymNNNNNmmNNmmdmmNNmhyyyhmmNNmdhmmmNNNddmNNNNmmNNNhyyhdmNmh---
.-smmmNNNNmmNNNdNmddddddmNNhyyddmmNmNmyydmNNNNNNNmmdmmmNNhyyyyyhNNNNs/dmmNNdo+yNNNNNmmNNNmdmmNNo-...
..oNNmNNNNmNNNNdmNmmmmmmmmdoyyhdmmNNmmyydmNmNNmmmNNmdmNNmyyyyyyyNNNd/.-:+hmdysmNNNNNNNNNNNNNNNd-....
.-dNNNNmmmmmmNNmdmNNmmNNNNhhyydhmmmmmdyydmNmmNNNNNNmmmNNdyyyyyyyNNNo....-::+oo+mNNNNNNmNNNNNNmh--...
.-dNNNmNNNNmmNNNmdmdmNNNds/ossyssooosyyyhNNNmmmNNNNNmNmNdyyyyyyhmNNs--:::::::oymNNNNNNNNNNNNNmdhyo:-
../ymNNNNNNNNNNNNNNNNNNm:------.....-+ooshhmNNNNNNmmNmNNmyyyyyymmNNmosssooosyyddmNmNNNNNNNNmmdy+:-..
...-hNNNNNNmNNNNNNNNNNNhooo++oooo+///:::--/mNNNNmmmmmmNNNdhyyhmmNNdyo/:-...--/ohmNNNNNNmdhs+:-......
...-sdmNNNNNNNNNNNNNNNNy++/:...---:/+ossyyhmNNNmNNNNmmmNNNmmmmmNNy-........:+osyyhhhhyo/-...........
....-oydmmNNNNNNNNNNmmmdhs+:.....--/oshdmmmmmmmNNNNNNNNNNNNmmNNNNo+++//::------.....................
......-/hmmNNNNmmNmmdddyssoo+++syhdmmmmmmmmmmhmNNNNNmmNmNNmmmNNdm/////+osyhhhyyo:...................
...:+shdmmmmmddhs+:---::::::/+oo+/::+yhdddmmmdhdmNNNNNNNNNNNNNmmmmdho:.....---......................
...-::///+++/:-........................-:::/+ssyhdmmmNNNNNNNNmdho/-.................................
............................................-:+shmmmmmmmmmhs+:-.....................................
...........................................-/osyyhdddhs+:-..........................................
....................................................................................................
....................................................................................................
```
