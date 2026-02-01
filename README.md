This is my try at building my own dcc decoder. The code is funktional with a small PC817 Opto just hooked up to the track and an arduino, but Ive tried making a pcb for it (order is ongoing as of writing this).

There is currently no code for the PCB, as I dont yet have one of them here and cant test on it.

The PCB has support for 3 individual inputs (e.g. left rail, right rail and ac contact or overhead wire), with all of them providing power.

It does not support anything other than standard dcc, this includes no support for any analog system.

There are jumpers for connecting an AC motor: desolder both of then, then hook the 2 wires of the coil to the left and right ports and the neutral wire to the middle one. For DC motors just use the outer ones.


Feel free to contribute to this project!