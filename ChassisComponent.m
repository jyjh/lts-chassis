classdef (Abstract) ChassisComponent
    % CHASSISCOMPONENT Abstract interface for chassis attitude dynamics
    %
    % Chassis components own the sprung-mass attitude state used for future
    % transient suspension, aero platform, and load-transfer calculations.

    properties (Abstract)
        state  % lts.components.Chassis.ChassisState
    end

    methods (Abstract)
        % Reset dynamic chassis attitude state to static equilibrium
        reset(obj)

        % Update heave, pitch, and roll from vehicle accelerations
        updateFromAccelerations(obj, ax, ay, aeroForces, dt, yawAccel)

        % Return current per-corner chassis displacement/velocity structs
        cornerKinematics = computeCornerKinematics(obj)

        % Convenience attitude accessors for simulator/aero users
        heave = getHeave(obj)
        pitchAngle = getPitchAngle(obj)
        rollAngle = getRollAngle(obj)
    end
end
