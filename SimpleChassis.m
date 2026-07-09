classdef SimpleChassis < lts.components.Chassis.ChassisComponent
    % SIMPLECHASSIS Lumped sprung-mass heave/pitch/roll chassis model
    %
    % Provides body-attitude state and derived corner kinematics consumed by
    % SuspensionManager for chassis-driven tire normal loads.

    properties
        state  % lts.components.Chassis.ChassisState

        % Vehicle geometry/mass. Pulled from lts.vehicle.VehicleManager at construction
        % (no defaults — a bare instance is invalid and must not be simulated).
        totalMass
        sprungMass
        wheelbase
        trackWidth
        cgHeight
        staticFrontWeight

        % Lumped inertias [kg*m^2]. Derived from mass + geometry at
        % construction unless explicitly passed as constructor args.
        pitchInertia
        rollInertia
        % Per-axle roll inertias [kg*m^2], split by static weight at
        % construction.
        frontRollInertia
        rearRollInertia

        % CG-to-axle moment arms [m], derived from wheelbase and static weight
        % at construction. Run invariants; precomputed to avoid recomputing
        % the pitch-moment bookkeeping every step.
        frontArm
        rearArm

        % Linear platform stiffness/damping from static equilibrium.
        % No defaults: set by lts.vehicle.VehicleManager.fromConfig from lts.vehicle.VehicleConfig.
        heaveStiffness    % [N/m]
        heaveDamping      % [N*s/m]
        pitchStiffness    % [N*m/rad]
        pitchDamping      % [N*m*s/rad]
        rollStiffness     % [N*m/rad]  (legacy whole-car; superseded by axle model)
        rollDamping       % [N*m*s/rad]

        % Chassis torsional rigidity [N*m/rad]. Couples the front and rear
        % roll DOFs via a torsion spring on (frontRollAngle - rearRollAngle).
        % Inf = perfectly rigid torsionally (front and rear roll together);
        % a finite value lets the body twist under asymmetric load.
        % Set by lts.vehicle.VehicleManager.fromConfig (e.g. 4000 N*m/deg ~ 229000 N*m/rad).
        torsionalRigidity  % [N*m/rad]
        torsionalDamping   % [N*m*s/rad]

        % Reference to the suspension manager, used to read the per-axle
        % wheel-rate roll stiffness so the chassis roll model and the
        % load-transfer split share the same numbers. Optional.
        suspension

        % Internal integration cap for the stiff vertical attitude states.
        maxIntegrationStep = 0.001
    end

    properties (Transient = true) %#ok<MCNPC>
        % Lazily-cached run invariant: whether the linked suspension exposes
        % getAxleRollStiffness. Empty = uncached.
        cachedSuspensionHasRollStiffness
    end

    methods
        function obj = SimpleChassis(vehicleManager, sprungMass, pitchInertia, rollInertia)
            % SIMPLECHASSIS Construct from lts.vehicle.VehicleManager geometry
            %   SimpleChassis(vehicleManager)
            %   SimpleChassis(vehicleManager, sprungMass, pitchInertia, rollInertia)
            if nargin >= 1 && ~isempty(vehicleManager)
                obj.totalMass = vehicleManager.totalMass;
                obj.sprungMass = vehicleManager.totalMass;
                obj.wheelbase = vehicleManager.wheelbase;
                obj.trackWidth = vehicleManager.trackWidth;
                obj.cgHeight = vehicleManager.cgHeight;
                obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            end
            if nargin >= 2 && ~isempty(sprungMass)
                obj.sprungMass = sprungMass;
            end
            if nargin >= 3 && ~isempty(pitchInertia)
                obj.pitchInertia = pitchInertia;
            else
                obj.pitchInertia = max(1, obj.sprungMass * obj.wheelbase^2 / 12);
            end
            if nargin >= 4 && ~isempty(rollInertia)
                obj.rollInertia = rollInertia;
            else
                obj.rollInertia = max(1, obj.sprungMass * obj.trackWidth^2 / 12);
            end
            % Split the whole-car roll inertia by static weight distribution.
            obj.frontRollInertia = max(1, obj.rollInertia * obj.staticFrontWeight);
            obj.rearRollInertia  = max(1, obj.rollInertia * (1 - obj.staticFrontWeight));

            % Precompute the CG-to-axle moment arms (run invariants).
            obj.frontArm = obj.wheelbase * (1 - obj.staticFrontWeight);
            obj.rearArm  = obj.wheelbase * obj.staticFrontWeight;

            obj.state = lts.components.Chassis.ChassisState();
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function obj = setSuspension(obj, suspension)
            % SETSUSPENSION Optional link to the suspension manager so the
            % chassis roll model can read the per-axle wheel-rate roll
            % stiffness (springs + anti-roll bars), keeping the two roll
            % models consistent. Call after both are constructed.
            obj.suspension = suspension;
            obj.cachedSuspensionHasRollStiffness = ...
                ~isempty(suspension) && ...
                isa(suspension, 'lts.components.Suspension.SuspensionManager');
        end

        function reset(obj)
            obj.state.reset();
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function updateFromAccelerations(obj, ax, ay, aeroForces, dt, yawAccel)
            % UPDATEFROMACCELERATIONS Integrate heave, pitch, and roll
            % ax > 0 creates nose-up pitch. ay > 0 creates right-side-down roll.
            if nargin < 4 || isempty(aeroForces)
                aeroForces = struct('Fz_front', 0, 'Fz_rear', 0, ...
                    'F_drag', 0, 'dragHeight', 0);
            end
            if nargin < 6 || isempty(yawAccel)
                yawAccel = 0;
            end

            nSubsteps = obj.integrationSubsteps(dt);
            if nSubsteps > 1
                subDt = dt / nSubsteps;
                for idx = 1:nSubsteps
                    obj.updateFromAccelerations(ax, ay, aeroForces, subDt, yawAccel);
                end
                return;
            end

            FzFront = obj.getStructField(aeroForces, 'Fz_front', 0);
            FzRear = obj.getStructField(aeroForces, 'Fz_rear', 0);
            Fdrag = obj.getStructField(aeroForces, 'F_drag', 0);
            dragHeight = obj.getStructField(aeroForces, 'dragHeight', 0);

            frontArm = obj.frontArm;
            rearArm = obj.rearArm;
            downforcePitchMoment = FzRear * rearArm - FzFront * frontArm;
            dragPitchMoment = Fdrag * dragHeight;
            aeroPitchMoment = downforcePitchMoment + dragPitchMoment;

            heaveForce = FzFront + FzRear ...
                - obj.heaveStiffness * obj.state.heave ...
                - obj.heaveDamping * obj.state.heaveRate;

            pitchMoment = obj.sprungMass * ax * obj.cgHeight + aeroPitchMoment ...
                - obj.pitchStiffness * obj.state.pitchAngle ...
                - obj.pitchDamping * obj.state.pitchRate;

            % --- Roll: front/rear split DOFs coupled by a torsion spring ---
            % The sprung-mass roll moment is evaluated at each axle center:
            % ay_front = ay + yawAccel*frontArm, ay_rear = ay - yawAccel*rearArm.
            % Each axle is resisted by its own roll stiffness (wheel springs
            % + ARB, read from the suspension so the chassis and load-transfer
            % models agree) and coupled to the other axle by the chassis
            % torsion spring on the twist angle (frontRollAngle - rearRollAngle). With
            % torsionalRigidity = Inf the two ends roll together (perfectly
            % rigid tub); a finite value lets the body twist under asymmetric
            % load. The legacy whole-car rollAngle is kept as the average.
            massFrac = obj.staticFrontWeight;
            rearMassFrac = 1 - massFrac;
            frontAxleAy = ay + yawAccel * obj.frontArm;
            rearAxleAy  = ay - yawAccel * obj.rearArm;
            rollMomentF = obj.sprungMass * massFrac * frontAxleAy * obj.cgHeight;
            rollMomentR = obj.sprungMass * rearMassFrac * rearAxleAy * obj.cgHeight;

            [KrollF, KrollR] = obj.getAxleRollStiffnessRad();
            CrollF = obj.rollDamping * massFrac;
            CrollR = obj.rollDamping * rearMassFrac;

            % If no per-axle stiffness is available, fall back to the legacy
            % whole-car rollStiffness split so the model remains stable.
            if KrollF <= 0 && KrollR <= 0
                KrollF = obj.rollStiffness * massFrac;
                KrollR = obj.rollStiffness * rearMassFrac;
            end

            twist = obj.state.frontRollAngle - obj.state.rearRollAngle;
            Kt = obj.torsionalRigidity;
            Ct = obj.torsionalDamping;
            twistRate = obj.state.frontRollRate - obj.state.rearRollRate;

            frontRollMoment = rollMomentF ...
                - KrollF * obj.state.frontRollAngle ...
                - CrollF * obj.state.frontRollRate ...
                - obj.safeTorsion(Kt, twist) ...
                - Ct * twistRate;
            rearRollMoment = rollMomentR ...
                - KrollR * obj.state.rearRollAngle ...
                - CrollR * obj.state.rearRollRate ...
                + obj.safeTorsion(Kt, twist) ...
                + Ct * twistRate;

            obj.state.heaveAccel = heaveForce / max(obj.sprungMass, eps);
            obj.state.pitchAccel = pitchMoment / max(obj.pitchInertia, eps);
            obj.state.frontRollAccel = frontRollMoment / max(obj.frontRollInertia, eps);
            obj.state.rearRollAccel  = rearRollMoment  / max(obj.rearRollInertia,  eps);

            obj.state.heaveRate = obj.state.heaveRate + obj.state.heaveAccel * dt;
            obj.state.pitchRate = obj.state.pitchRate + obj.state.pitchAccel * dt;
            obj.state.frontRollRate = obj.state.frontRollRate + obj.state.frontRollAccel * dt;
            obj.state.rearRollRate  = obj.state.rearRollRate  + obj.state.rearRollAccel  * dt;

            obj.state.heave = obj.state.heave + obj.state.heaveRate * dt;
            obj.state.pitchAngle = obj.state.pitchAngle + obj.state.pitchRate * dt;
            obj.state.frontRollAngle = obj.state.frontRollAngle + obj.state.frontRollRate * dt;
            obj.state.rearRollAngle  = obj.state.rearRollAngle  + obj.state.rearRollRate  * dt;

            % Legacy whole-car roll state = average of the two axle DOFs.
            obj.state.rollAngle = 0.5 * (obj.state.frontRollAngle + obj.state.rearRollAngle);
            obj.state.rollRate  = 0.5 * (obj.state.frontRollRate  + obj.state.rearRollRate);
            obj.state.rollAccel = 0.5 * (obj.state.frontRollAccel + obj.state.rearRollAccel);

            obj.state.longitudinalLoadTransfer = ...
                obj.totalMass * ax * obj.cgHeight / max(obj.wheelbase, eps);
            obj.state.lateralLoadTransfer = ...
                obj.totalMass * ay * obj.cgHeight / max(obj.trackWidth, eps);
            obj.state.downforcePitchMoment = downforcePitchMoment;
            obj.state.dragPitchMoment = dragPitchMoment;
            obj.state.aeroPitchMoment = aeroPitchMoment;

            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function cornerKinematics = computeCornerKinematics(obj)
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
            if nargout > 0
                cornerKinematics.displacement = obj.state.cornerDisplacement;
                cornerKinematics.velocity = obj.state.cornerVelocity;
            end
        end

        function heave = getHeave(obj)
            heave = obj.state.heave;
        end

        function pitchAngle = getPitchAngle(obj)
            pitchAngle = obj.state.pitchAngle;
        end

        function rollAngle = getRollAngle(obj)
            % Whole-car roll angle [rad] (average of front/rear DOFs).
            rollAngle = obj.state.rollAngle;
        end

        function rollRate = getRollRate(obj)
            % Whole-car roll rate [rad/s] (average of front/rear DOFs).
            rollRate = obj.state.rollRate;
        end

        function rollAngle = getFrontRollAngle(obj)
            rollAngle = obj.state.frontRollAngle;
        end

        function rollAngle = getRearRollAngle(obj)
            rollAngle = obj.state.rearRollAngle;
        end

        function rollRate = getFrontRollRate(obj)
            rollRate = obj.state.frontRollRate;
        end

        function rollRate = getRearRollRate(obj)
            rollRate = obj.state.rearRollRate;
        end

        function twist = getTwistAngle(obj)
            % Chassis twist angle [rad] = front - rear roll. Zero when the
            % tub is torsionally rigid or under symmetric load.
            twist = obj.state.frontRollAngle - obj.state.rearRollAngle;
        end

        function twistRate = getTwistRate(obj)
            % Chassis twist rate [rad/s] = front - rear roll rate.
            twistRate = obj.state.frontRollRate - obj.state.rearRollRate;
        end
    end

    methods (Access = private)
        function [KrollF, KrollR] = getAxleRollStiffnessRad(obj)
            % GETAXLEROLLSTIFFNESSRAD Per-axle roll stiffness [N*m/rad].
            % Reads the per-axle wheel-rate roll stiffness (springs + ARB)
            % from the linked suspension manager and converts to a roll
            % rate about the roll axis: K_roll = Kw * (t/2)^2 * 2 = Kw*t^2/2.
            % Returns 0,0 when no suspension is linked.
            KrollF = 0;
            KrollR = 0;
            hasRollStiffness = obj.cachedSuspensionHasRollStiffness;
            if isempty(hasRollStiffness)
                hasRollStiffness = ~isempty(obj.suspension) && ...
                    isa(obj.suspension, 'lts.components.Suspension.SuspensionManager');
            end
            if hasRollStiffness
                [KwF, KwR] = obj.suspension.getAxleRollStiffness();
                KrollF = KwF * obj.trackWidth^2 / 2;
                KrollR = KwR * obj.trackWidth^2 / 2;
            end
        end

        function torque = safeTorsion(~, Kt, twist)
            % SAFETORSION Torsion-spring torque, robust to Kt = Inf.
            % With infinite rigidity the front/rear DOFs should lock together;
            % an Inf*0 product would yield NaN, so a very large finite cap is
            % used instead, which numerically enforces near-equal roll angles.
            if isinf(Kt)
                torque = 1e9 * twist;
            else
                torque = Kt * twist;
            end
        end

        function n = integrationSubsteps(obj, dt)
            maxStep = obj.maxIntegrationStep;
            if ~isfinite(maxStep) || maxStep <= 0
                n = 1;
            else
                n = max(1, ceil(dt / maxStep));
            end
        end
    end

    methods (Static, Access = private)
        function value = getStructField(s, fieldName, defaultValue)
            if isstruct(s) && isfield(s, fieldName)
                value = s.(fieldName);
            else
                value = defaultValue;
            end
        end
    end
end
