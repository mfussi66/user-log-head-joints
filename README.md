# Log neck joints data

The snippet code allows moving one of the neck DoFs between two set points both in position and PWM control mode and logs the the following data for the roll and pitch joints:

- PID reference
- PID output
- Joint encoder 
- Motor current
- Motor encoder 
- PWM readouts
- Torque

The matlab script plots the logged data.

## Compilation

```bash
cd user-log-head-joints
mkdir build
cd build 
ccmake .. # use it to define your installation folder, then hit "generate"
make install
```

## Usage

```bash
cd user-log-head-joints
cd app
userloghead --from config.ini
```
