classdef ChassisState < handle
    % CHASSISSTATE Mutable chassis attitude and derived corner state
    %
    % State is measured from static equilibrium. Positive heave is downward
    % body motion, positive pitch is nose-up, and positive roll means the
    % right side moves downward relative to the left side.

    properties
        % Body attitude state
        heave = 0          % Sprung-mass vertical displacement [m], positive down
        heaveRate = 0      % Sprung-mass vertical velocity [m/s], positive down
        heaveAccel = 0     % Sprung-mass vertical acceleration [m/s^2]

        pitchAngle = 0     % Body pitch angle [rad], positive nose-up
        pitchRate = 0      % Body pitch rate [rad/s]
        pitchAccel = 0     % Body pitch acceleration [rad/s^2]

        rollAngle = 0      % Body roll angle [rad], positive right-side-down
        rollRate = 0       % Body roll rate [rad/s]
        rollAccel = 0      % Body roll acceleration [rad/s^2]

        % Front/rear split roll DOFs. With finite chassis torsional rigidity
        % the two ends of the body can roll by different amounts under
        % asymmetric load; the twist angle is frontRollAngle - rearRollAngle.
        % rollAngle above is kept as the average for backward compatibility.
        frontRollAngle = 0   % Front-end roll angle [rad]
        frontRollRate  = 0   % Front-end roll rate [rad/s]
        frontRollAccel = 0   % Front-end roll acceleration [rad/s^2]
        rearRollAngle  = 0   % Rear-end roll angle [rad]
        rearRollRate   = 0   % Rear-end roll rate [rad/s]
        rearRollAccel  = 0   % Rear-end roll acceleration [rad/s^2]

        % Derived corner chassis displacement/velocity at suspension pickups
        cornerDisplacement = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0)
        cornerVelocity = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0)

        % Derived load-transfer terms for telemetry/debugging [N]
        longitudinalLoadTransfer = 0
        lateralLoadTransfer = 0
        additionalLongitudinalLoadTransfer = 0
        frontAdditionalLateralLoadTransfer = 0
        rearAdditionalLateralLoadTransfer = 0
        frontGeometricLateralLoadTransfer = 0
        rearGeometricLateralLoadTransfer = 0
        frontRollCenterLateral = 0
        rearRollCenterLateral = 0
        downforcePitchMoment = 0
        dragPitchMoment = 0
        aeroPitchMoment = 0
    end

    methods
        function obj = ChassisState()
            obj.reset();
        end

        function reset(obj)
            obj.heave = 0;
            obj.heaveRate = 0;
            obj.heaveAccel = 0;
            obj.pitchAngle = 0;
            obj.pitchRate = 0;
            obj.pitchAccel = 0;
            obj.rollAngle = 0;
            obj.rollRate = 0;
            obj.rollAccel = 0;
            obj.frontRollAngle = 0;
            obj.frontRollRate  = 0;
            obj.frontRollAccel = 0;
            obj.rearRollAngle  = 0;
            obj.rearRollRate   = 0;
            obj.rearRollAccel  = 0;
            obj.cornerDisplacement = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            obj.cornerVelocity = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            obj.longitudinalLoadTransfer = 0;
            obj.lateralLoadTransfer = 0;
            obj.additionalLongitudinalLoadTransfer = 0;
            obj.frontAdditionalLateralLoadTransfer = 0;
            obj.rearAdditionalLateralLoadTransfer = 0;
            obj.frontGeometricLateralLoadTransfer = 0;
            obj.rearGeometricLateralLoadTransfer = 0;
            obj.frontRollCenterLateral = 0;
            obj.rearRollCenterLateral = 0;
            obj.downforcePitchMoment = 0;
            obj.dragPitchMoment = 0;
            obj.aeroPitchMoment = 0;
        end

        function updateCornerKinematics(obj, wheelbase, trackWidth, staticFrontWeight)
            % UPDATECORNERKINEMATICS Convert body attitude to corner motion
            % Outputs are positive for compression-producing body motion.
            frontArm = wheelbase * (1 - staticFrontWeight);
            rearArm = wheelbase * staticFrontWeight;
            halfTrack = trackWidth / 2;

            frontRoll = obj.frontRollAngle;
            rearRoll = obj.rearRollAngle;
            frontRollRate = obj.frontRollRate;
            rearRollRate = obj.rearRollRate;
            if frontRoll == 0 && rearRoll == 0 && obj.rollAngle ~= 0
                frontRoll = obj.rollAngle;
                rearRoll = obj.rollAngle;
            end
            if frontRollRate == 0 && rearRollRate == 0 && obj.rollRate ~= 0
                frontRollRate = obj.rollRate;
                rearRollRate = obj.rollRate;
            end

            obj.cornerDisplacement.FL = obj.heave - obj.pitchAngle * frontArm - frontRoll * halfTrack;
            obj.cornerDisplacement.FR = obj.heave - obj.pitchAngle * frontArm + frontRoll * halfTrack;
            obj.cornerDisplacement.RL = obj.heave + obj.pitchAngle * rearArm - rearRoll * halfTrack;
            obj.cornerDisplacement.RR = obj.heave + obj.pitchAngle * rearArm + rearRoll * halfTrack;

            obj.cornerVelocity.FL = obj.heaveRate - obj.pitchRate * frontArm - frontRollRate * halfTrack;
            obj.cornerVelocity.FR = obj.heaveRate - obj.pitchRate * frontArm + frontRollRate * halfTrack;
            obj.cornerVelocity.RL = obj.heaveRate + obj.pitchRate * rearArm - rearRollRate * halfTrack;
            obj.cornerVelocity.RR = obj.heaveRate + obj.pitchRate * rearArm + rearRollRate * halfTrack;
        end
    end
end
