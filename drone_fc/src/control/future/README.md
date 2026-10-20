# Future controller work

## NMPC real-time iteration scaffold

`nmpc_rti.hpp` defines the fixed-horizon state/reference/model contract and
separates the RTI preparation and feedback phases behind a caller-supplied
backend. The frame convention is NED position/velocity, body-to-NED attitude,
FRD body rates, and SI units.

No QP solver is included, and the scaffold is not connected to the flight-mode
or motor-output path. `feedback()` returns `kBackendUnavailable` with a zero
input unless a backend is supplied. Filling the model from bench-calibrated
mass, inertia, rotor thrust, and yaw-moment data, then measuring bounded
execution time on the target MCU, are prerequisites for any future integration.
