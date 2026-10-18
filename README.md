# Arduino Accelerometer Tilt Leveler

Arduino Uno R3 + GY-521 (MPU-6050) sketch. Press the button to save the current tilt angle;
the LEDs then show how far the board has moved from it:

| Difference from saved angle | LED            |
|-----------------------------|----------------|
| ≤ 0.25°                     | Blue (pin 9)   |
| 0.25° – 0.50°               | Green (pin 8)  |
| > 0.50°                     | Red (pin 10)   |

Sketch: [`TiltLeveler/TiltLeveler.ino`](TiltLeveler/TiltLeveler.ino) — uses only the built-in `Wire` library.
Wiring notes are in the comment block at the top of the sketch. Serial Monitor: 115200 baud.

If tilting the stick the way you care about doesn't change the reading, set `USE_ROLL` to `false`.
