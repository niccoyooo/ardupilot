So far...

I commented out unwanted code (full deletes later) in Arducopter/Copter.cpp
I made notes in Tools/scripts/build_options.py of what to disable; the file uses a binary system where...
    0 Enable
    1 Disable

Lindsey has added the voltage of the pedal to the logging message process in ArduCopter, along with pitch, roll, yaw, acceleration, velocity and position. Made additional comment outs on Copter.cpp

Need to have:
    I2C connection with the Raspberry pi for data transmit
    Possible: connection to a separate app to enable editing to data and analyzing data