# ar3_microcontrollers
This module contains the microcontroller firmware for the AR3. 

## Teensy
There are two Arduino sketches provided for the Teensy 3.5.  

* `baseline_with_ARCS` is fully compatible with the default configurations of the AR3 and also contains the original code for ARCS. You will be able to use both software without needing to reflash the Teensy. You cannot use both simultaneously, however. There will only be minor changes to this for fixing bugs and such.

* `baseline_dev` has been cleaned up for development and does not support ARCS. There will be documented changes that you can make which are adapted for my hardware, which are currently switching J2 and J5 to joint position encoders and switching the J2 motor to 4000 steps/rev. I will continue developing on this version to add more features.
