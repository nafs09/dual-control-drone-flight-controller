# Future controller work

## NMPC real-time-iteration interface scaffold

nmpc_rti.hpp defines a fixed-size interface for a 20-interval horizon. It separates the real-time iteration preparation phase from the feedback phase through caller-supplied function pointers. State, reference, model, and input values use SI units; positions and velocities are NED; attitude is body-to-NED; body rates and torques use FRD axes.

The interface validates basic finite/range constraints and initializes feedback output to zero before calling a backend. With no backend, preparation or feedback reports kBackendUnavailable. A successful status means only that the supplied callback returned success; it does not establish solver feasibility, stability, deadline compliance, or safe motor commands.

## Not included

- No nonlinear dynamics implementation.
- No linearization, QP solver, optimizer workspace, or constraint setup.
- No rotor allocation or connection to the mixer.
- No flight-mode/FSM integration, fallback policy, or deadline monitor.
- No target-MCU execution-time or closed-loop validation.

## Work required before integration

1. Select and implement a fixed-memory dynamics and QP backend.
2. Supply measured mass, inertia, rotor thrust and yaw-moment coefficients, rotor limits, and actuator delays.
3. Define state/input constraints, reference generation, warm-start and failure behavior, and a bounded fallback controller.
4. Measure worst-case execution time and memory use on the target under RTOS load.
5. Validate numerical behavior in simulation, then hardware-in-loop and restrained bench tests.
6. Integrate only behind an explicit mode/feature gate after review of safety behavior.

The PID cascade remains the active controller. This scaffold is not flight-ready and must not be treated as a motor command generator.
