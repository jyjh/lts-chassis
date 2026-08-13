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
        % NaN = derive from the linked suspension's spring/damper rates (the
        % default); a finite value overrides the derivation. Set by
        % lts.vehicle.VehicleManager.fromConfig from lts.vehicle.VehicleConfig.
        heaveStiffness    = NaN  % [N/m]  NaN = derive from suspension
        heaveDamping      = NaN  % [N*s/m]  NaN = derive
        pitchStiffness    = NaN  % [N*m/rad]  NaN = derive
        pitchDamping      = NaN  % [N*m*s/rad]  NaN = derive
        rollStiffness     % [N*m/rad]  (legacy whole-car; superseded by axle model)
        rollDamping       = NaN  % [N*m*s/rad]  NaN = derive

        % Chassis torsional rigidity [N*m/rad]. Couples the front and rear
        % roll DOFs via a torsion spring on (frontRollAngle - rearRollAngle).
        % A finite value lets the body twist under asymmetric load. Inf is
        % rejected (numerically unstable with explicit Euler at dt <= 1 ms);
        % use a large finite value (e.g. 1e6 N*m/rad) for near-rigid torsion.
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

        % Lazily-cached resolved platform stiffness/damping. Derived from
        % the linked suspension on first use, or overridden by finite
        % property values. Empty = uncached.
        cachedDerivedPlatform
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
            % Clear the derived-platform cache so heave/pitch/roll
            % stiffness+damping are re-derived with the linked suspension.
            obj.cachedDerivedPlatform = [];
        end

        function reset(obj)
            if isinf(obj.torsionalRigidity)
                error('lts_components_Chassis_SimpleChassis:InfTorsionalRigidity', ...
                    ['torsionalRigidity = Inf is numerically unstable with explicit Euler ' ...
                    'at dt <= 1 ms. Use a large finite value (e.g. 1e6 N*m/rad) for ' ...
                    'near-rigid torsion.']);
            end
            obj.state.reset();
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function updateFromAccelerations(obj, ax, ay, aeroForces, dt, yawAccel)
            % UPDATEFROMACCELERATIONS Integrate heave, pitch, and roll
            % ax > 0 creates nose-up pitch. ay > 0 creates right-side-down roll.
            if isinf(obj.torsionalRigidity)
                error('lts_components_Chassis_SimpleChassis:InfTorsionalRigidity', ...
                    ['torsionalRigidity = Inf is numerically unstable with explicit ' ...
                    'Euler at dt <= 1 ms. Use a large finite value (e.g. 1e6 ' ...
                    'N*m/rad) for near-rigid torsion.']);
            end
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

            p = obj.effectivePlatform();

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
                - p.Kheave * obj.state.heave ...
                - p.Cheave * obj.state.heaveRate;

            pitchMoment = obj.sprungMass * ax * obj.cgHeight + aeroPitchMoment ...
                - p.Kpitch * obj.state.pitchAngle ...
                - p.Cpitch * obj.state.pitchRate;

            % --- Roll: front/rear split DOFs coupled by a torsion spring ---
            % The sprung-mass roll moment is evaluated at each axle center:
            % ay_front = ay + yawAccel*frontArm, ay_rear = ay - yawAccel*rearArm.
            % Each axle is resisted by its own roll stiffness (wheel springs
            % + ARB, read from the suspension so the chassis and load-transfer
            % models agree) and coupled to the other axle by the chassis
            % torsion spring on the twist angle (frontRollAngle - rearRollAngle).
            % A large finite torsionalRigidity makes the two ends roll nearly
            % together; a smaller value lets the body twist under asymmetric
            % load. The legacy whole-car rollAngle is kept as the average.
            massFrac = obj.staticFrontWeight;
            rearMassFrac = 1 - massFrac;
            frontAxleAy = ay + yawAccel * obj.frontArm;
            rearAxleAy  = ay - yawAccel * obj.rearArm;
            frontSprungMass = obj.sprungMass * massFrac;
            rearSprungMass = obj.sprungMass * rearMassFrac;
            [hrcF, hrcR, rclFAt1g, rclRAt1g] = obj.getRollCenterConfig();
            rclF = obj.scaledRollCenterLateral(rclFAt1g, frontAxleAy);
            rclR = obj.scaledRollCenterLateral(rclRAt1g, rearAxleAy);
            rollMomentF = frontSprungMass * ...
                (frontAxleAy * (obj.cgHeight - hrcF) + ...
                lts.vehicle.VehicleManager.g * rclF);
            rollMomentR = rearSprungMass * ...
                (rearAxleAy * (obj.cgHeight - hrcR) + ...
                lts.vehicle.VehicleManager.g * rclR);
            geoLatFront = frontSprungMass * frontAxleAy * hrcF / ...
                max(obj.trackWidth, eps);
            geoLatRear = rearSprungMass * rearAxleAy * hrcR / ...
                max(obj.trackWidth, eps);

            [KrollF, KrollR] = obj.getAxleRollStiffnessRad();
            CrollF = p.Croll * massFrac;
            CrollR = p.Croll * rearMassFrac;

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
                (rollMomentF + rollMomentR) / max(obj.trackWidth, eps) + ...
                geoLatFront + geoLatRear;
            obj.state.frontGeometricLateralLoadTransfer = geoLatFront;
            obj.state.rearGeometricLateralLoadTransfer = geoLatRear;
            obj.state.frontRollCenterLateral = rclF;
            obj.state.rearRollCenterLateral = rclR;
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
        function [hrcF, hrcR, rclF, rclR] = getRollCenterConfig(obj)
            hrcF = 0;
            hrcR = 0;
            rclF = 0;
            rclR = 0;
            if isempty(obj.suspension) || ...
                    ~isa(obj.suspension, 'lts.components.Suspension.SuspensionManager')
                return;
            end

            hrcF = obj.suspension.frontRollCenterHeight;
            hrcR = obj.suspension.rearRollCenterHeight;
            rclF = obj.suspension.frontRollCenterLateral;
            rclR = obj.suspension.rearRollCenterLateral;
        end

        function value = scaledRollCenterLateral(~, lateralAt1g, axleAy)
            value = lateralAt1g * axleAy / lts.vehicle.VehicleManager.g;
            if ~isfinite(value)
                value = 0;
            end
        end

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
            % SAFETORSION Torsion-spring torque.
            % Inf torsionalRigidity is rejected in reset(), but a defensive
            % guard is kept here for robustness.
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

        %% ---- Platform parameter derivation ----

        function p = effectivePlatform(obj)
            % EFFECTIVEPLATFORM Resolve heave/pitch/roll stiffness + damping.
            % NaN properties are derived from the linked suspension's
            % spring/damper rates; finite values are used directly as
            % overrides. Memoized (run invariant).
            if ~isempty(obj.cachedDerivedPlatform)
                p = obj.cachedDerivedPlatform;
                return;
            end
            p = struct();
            if isnan(obj.heaveStiffness)
                p.Kheave = obj.deriveHeaveStiffness();
            else
                p.Kheave = obj.heaveStiffness;
            end
            if isnan(obj.heaveDamping)
                p.Cheave = obj.deriveHeaveDamping();
            else
                p.Cheave = obj.heaveDamping;
            end
            if isnan(obj.pitchStiffness)
                p.Kpitch = obj.derivePitchStiffness();
            else
                p.Kpitch = obj.pitchStiffness;
            end
            if isnan(obj.pitchDamping)
                p.Cpitch = obj.derivePitchDamping();
            else
                p.Cpitch = obj.pitchDamping;
            end
            if isnan(obj.rollDamping)
                p.Croll = obj.deriveRollDamping();
            else
                p.Croll = obj.rollDamping;
            end
            obj.cachedDerivedPlatform = p;
        end

        function has = hasLinkedSuspension(obj)
            has = ~isempty(obj.suspension) && ...
                isa(obj.suspension, 'lts.components.Suspension.SuspensionManager');
        end

        function mr = cornerMotionRatio(~, cornerUnit)
            mr = cornerUnit.motionRatio;
            if cornerUnit.state.motionRatioEffective > 0
                mr = cornerUnit.state.motionRatioEffective;
            end
            mr = max(mr, eps);
        end

        function K = deriveHeaveStiffness(obj)
            % 4 corners: 2*(Kf*MRf^2 + Kr*MRr^2)
            if ~obj.hasLinkedSuspension(); K = 0; return; end
            susp = obj.suspension;
            mrF = obj.cornerMotionRatio(susp.frontLeft);
            mrR = obj.cornerMotionRatio(susp.rearLeft);
            K = 2 * (susp.frontLeft.springRate * mrF^2 + ...
                     susp.rearLeft.springRate * mrR^2);
        end

        function C = deriveHeaveDamping(obj)
            % 4 corners, compression damping
            if ~obj.hasLinkedSuspension(); C = 0; return; end
            susp = obj.suspension;
            mrF = obj.cornerMotionRatio(susp.frontLeft);
            mrR = obj.cornerMotionRatio(susp.rearLeft);
            C = 2 * (susp.frontLeft.dampingCoeff * mrF^2 + ...
                     susp.rearLeft.dampingCoeff * mrR^2);
        end

        function K = derivePitchStiffness(obj)
            % Kf*MRf^2*frontArm^2 + Kr*MRr^2*rearArm^2
            if ~obj.hasLinkedSuspension(); K = 0; return; end
            susp = obj.suspension;
            mrF = obj.cornerMotionRatio(susp.frontLeft);
            mrR = obj.cornerMotionRatio(susp.rearLeft);
            K = susp.frontLeft.springRate * mrF^2 * obj.frontArm^2 + ...
                susp.rearLeft.springRate * mrR^2 * obj.rearArm^2;
        end

        function C = derivePitchDamping(obj)
            if ~obj.hasLinkedSuspension(); C = 0; return; end
            susp = obj.suspension;
            mrF = obj.cornerMotionRatio(susp.frontLeft);
            mrR = obj.cornerMotionRatio(susp.rearLeft);
            C = susp.frontLeft.dampingCoeff * mrF^2 * obj.frontArm^2 + ...
                susp.rearLeft.dampingCoeff * mrR^2 * obj.rearArm^2;
        end

        function C = deriveRollDamping(obj)
            % (Cf*MRf^2 + Cr*MRr^2) * trackWidth^2 / 2
            % Mirrors the wheel-rate → roll-rate conversion in
            % getAxleRollStiffnessRad, applied to damper rates.
            if ~obj.hasLinkedSuspension(); C = 0; return; end
            susp = obj.suspension;
            mrF = obj.cornerMotionRatio(susp.frontLeft);
            mrR = obj.cornerMotionRatio(susp.rearLeft);
            C = (susp.frontLeft.dampingCoeff * mrF^2 + ...
                 susp.rearLeft.dampingCoeff * mrR^2) * ...
                obj.trackWidth^2 / 2;
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
