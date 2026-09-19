# Autonomous Table-Cleaning Robot

An autonomous vacuum/mop cleaning robot built on the ESP32. It drives
itself around a surface, avoids obstacles using an ultrasonic sensor,
and sweeps with Roomba-style side brushes.

## What it does
- Moves autonomously with differential-drive (two independently driven wheels)
- Detects and avoids obstacles using an HC-SR04 ultrasonic sensor
- Sweeps debris inward with rotating side brushes

## Hardware
- ESP32 microcontroller
- DC gear motors + motor driver
- HC-SR04 ultrasonic distance sensor
- Rechargeable Li-ion battery pack
- Custom acrylic + cardboard chassis

## How the navigation works
I tested several strategies to get reliable coverage:
- Timed moves and turns
- Gyro-assisted turns
- A Roomba-style random walk as a fallback

I settled on stopping at a fixed distance from obstacles using the
ultrasonic sensor rather than timed forward motion, which made the
behaviour much more consistent. Pivot turns were calibrated to correct
for the two motors not spinning at exactly the same speed.

## The interesting problem
The robot kept dropping its wireless connection under load. I traced it
to electrical noise from the motors, and fixed it with power-supply and
motor-driver changes. A good reminder that real hardware behaves nothing
like the clean version in theory.

## Status
Working prototype. Built as a college project and competition entry.
