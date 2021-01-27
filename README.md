# Porsche-997-and-987-Gauge-Cluster-Operator
This code is designed to be used with the Macchina M2 (Arduino Due based).  It uses OBD PID requests to run the gauge cluster in the Porsche through the CANbus.  Tested with GM 6.2L LT1 swapped Porche 997.

Additional functions exist within the code.  The car it was used with was a Tiptronic, converted to manual.  A section of code is added to use an input on the M2 as a switch that sends CAN to the dash to light up "R" and to turn on the reverse lights

As engine speed goes above 450, an output is activated that can be used to turn on a TRW power steering pump.

A replicated TOS signal is ouput using PWM signals.  This will need to be scaled if your tire size is different.  It's currently scalled for a 40pulse per revolution wheel speed with 997 Carrera S wheel size

Partial AC functions are added.  This code reads the bit that the 997 sends to turn on the AC.  Nothing is done with that at this point.  This portion is untested

A simple Radiator fan algo is added.  It sends a CANbus request as coolant exceeds 90C (194F).  From there it ramps up fan speed until coolant reaches 110C where it puts the fans on max.  Be careful if you mess with this, the same byte has a bit for "Engine running."
