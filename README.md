# coupled_learning_spring
Arduino code for controlling the measurement and updating of a tunable elastic spring unit cell. For use in a coupled learning network.


## How it works:

The unit cell has four learning modes:
0: Null state. Doesn't do anything. Purely for visualization purposes.
1: Free state. Records the steady-state free flex sensor value.
2: Clamped state. Records the steady-state clamped flex sensor value.
3: Update state: Computes the update rule based on recorded free and clamped flex sensor values, set a new goal layer value.

There are also three plotting modes:
0: Null plot. Plots nothing.
1: Sensor plot. Plots spring displacement (from flex sensor values)
2: Layer plot. Plots current and goal encoder position.

At all times, the main loop is running a low-pass filter on the sensor values and checking if the encoder position needs to be changed.


## Keyboard commands

There are a few keyboard commands that can be executed during runtime:

"M" -> change learning mode. No options, just cycles through each.
"P" -> change plotting mode. For example: "P 1" sets plotting mode to sensor plot.
"I" -> set initial layers in half-layer increments. For example: "I 1" sets encoder position to 0.
"A" -> set goal layers in half-layer increments. For example: "A 1" sets goal encoder position to 0.
