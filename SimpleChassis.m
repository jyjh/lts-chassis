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

        % Linear platform stiffness/damping fallback used when no physical
        % suspension is linked. With a linked SuspensionManager, the chassis
        % reacts against the actual corner spring/damper/bump-stop forces so
        % those internal forces cancel in the whole-vehicle balance.
        heaveStiffness    % [N/m]
        heaveDamping      % [N*s/m]
        pitchStiffness    % [N*m/rad]
        pitchDamping      % [N*m*s/rad]
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
        % equivalent per-corner axle roll rate so the chassis roll model and
        % the load-transfer split share the same numbers. Optional.
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
                    'F_drag', 0, 'dragHeight', 0, ...
                    'dragXPosition', 0, 'F_drag_longitudinal', 0, ...
                    'F_drag_lateral', 0);
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
            % Signed component magnitudes along the body-frame velocity
            % direction. The actual aerodynamic force is their negative.
            % Legacy callers omit these fields and are assumed to be moving
            % straight forward.
            FdragLongitudinal = obj.getStructField( ...
                aeroForces, 'F_drag_longitudinal', Fdrag);
            FdragLateral = obj.getStructField( ...
                aeroForces, 'F_drag_lateral', 0);
            % Absolute drag-center height above ground and longitudinal
            % position from CG. These reference datums are shared by all
            % AeroComponent implementations.
            dragHeight = obj.getStructField(aeroForces, 'dragHeight', 0);
            dragXPosition = obj.getStructField( ...
                aeroForces, 'dragXPosition', 0);
            currentCgHeight = obj.cgHeight - obj.state.heave;
            if ~isfinite(currentCgHeight)
                currentCgHeight = obj.cgHeight;
            end

            % The passed accelerations are net accelerations and already
            % contain aerodynamic drag. Recover the non-aero acceleration
            % before forming the ground-force load-transfer moments, then
            % add the actual aero moment about the CG exactly once.
            nonAeroAx = ax + FdragLongitudinal / max(obj.totalMass, eps);
            nonAeroAy = ay + FdragLateral / max(obj.totalMass, eps);

            frontArm = obj.frontArm;
            rearArm = obj.rearArm;
            downforcePitchMoment = FzRear * rearArm - FzFront * frontArm;
            dragPitchMoment = FdragLongitudinal * ...
                (dragHeight - currentCgHeight);
            aeroPitchMoment = downforcePitchMoment + dragPitchMoment;

            [suspensionReaction, useSuspensionReaction] = ...
                obj.getSuspensionReactionDeltas();
            if useSuspensionReaction
                % Forces are expressed as increments from static equilibrium.
                % Positive suspension force acts upward on the sprung mass,
                % while positive chassis heave is downward.
                heaveReaction = suspensionReaction.FL + suspensionReaction.FR + ...
                    suspensionReaction.RL + suspensionReaction.RR;
                heaveForce = FzFront + FzRear - heaveReaction;

                % Positive pitch is nose-up. An increased front suspension
                % reaction creates a nose-up moment; an increased rear
                % reaction creates a nose-down moment.
                pitchReaction = ...
                    (suspensionReaction.FL + suspensionReaction.FR) * frontArm - ...
                    (suspensionReaction.RL + suspensionReaction.RR) * rearArm;
                pitchMoment = obj.sprungMass * nonAeroAx * obj.cgHeight + ...
                    aeroPitchMoment + pitchReaction;
            else
                heaveForce = FzFront + FzRear ...
                    - obj.heaveStiffness * obj.state.heave ...
                    - obj.heaveDamping * obj.state.heaveRate;

                pitchMoment = obj.sprungMass * nonAeroAx * obj.cgHeight + aeroPitchMoment ...
                    - obj.pitchStiffness * obj.state.pitchAngle ...
                    - obj.pitchDamping * obj.state.pitchRate;
            end

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
            frontAxleAy = nonAeroAy + yawAccel * obj.frontArm;
            rearAxleAy  = nonAeroAy - yawAccel * obj.rearArm;
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

            % Direct lateral-drag roll moment about the CG. Resolve the
            % force-weighted longitudinal CP between the front/rear roll
            % DOFs without clamping so an outboard CP preserves its moment.
            dragRollMoment = FdragLateral * (dragHeight - currentCgHeight);
            dragFrontFraction = (rearArm + dragXPosition) / ...
                max(obj.wheelbase, eps);
            rollMomentF = rollMomentF + dragRollMoment * dragFrontFraction;
            rollMomentR = rollMomentR + dragRollMoment * ...
                (1 - dragFrontFraction);
            geoLatFront = frontSprungMass * frontAxleAy * hrcF / ...
                max(obj.trackWidth, eps);
            geoLatRear = rearSprungMass * rearAxleAy * hrcR / ...
                max(obj.trackWidth, eps);

            % The attitude DOFs represent sprung mass only. Account for the
            % remainder of the configured total vehicle mass directly at the
            % contact patches, otherwise load transfer is understated by the
            % sprung/total mass ratio when unsprung CG data is unavailable.
            additionalMass = max(obj.totalMass - obj.sprungMass, 0);
            additionalLongitudinalTransfer = additionalMass * nonAeroAx * ...
                obj.cgHeight / max(obj.wheelbase, eps);
            additionalFrontMass = additionalMass * massFrac;
            additionalRearMass = additionalMass * rearMassFrac;
            additionalGeoFront = additionalFrontMass * frontAxleAy * hrcF / ...
                max(obj.trackWidth, eps);
            additionalGeoRear = additionalRearMass * rearAxleAy * hrcR / ...
                max(obj.trackWidth, eps);
            additionalElasticMoment = ...
                additionalFrontMass * (frontAxleAy * (obj.cgHeight - hrcF) + ...
                    lts.vehicle.VehicleManager.g * rclF) + ...
                additionalRearMass * (rearAxleAy * (obj.cgHeight - hrcR) + ...
                    lts.vehicle.VehicleManager.g * rclR);
            frontRollStiffnessFraction = massFrac;
            if ~isempty(obj.suspension) && ...
                    ismethod(obj.suspension, 'deriveFrontRollStiffnessFraction')
                frontRollStiffnessFraction = ...
                    obj.suspension.deriveFrontRollStiffnessFraction();
            end
            frontRollStiffnessFraction = lts.util.clamp( ...
                frontRollStiffnessFraction, 0, 1);
            additionalElasticTransfer = additionalElasticMoment / ...
                max(obj.trackWidth, eps);
            additionalLatFront = additionalGeoFront + ...
                additionalElasticTransfer * frontRollStiffnessFraction;
            additionalLatRear = additionalGeoRear + ...
                additionalElasticTransfer * (1 - frontRollStiffnessFraction);

            twist = obj.state.frontRollAngle - obj.state.rearRollAngle;
            Kt = obj.torsionalRigidity;
            Ct = obj.torsionalDamping;
            twistRate = obj.state.frontRollRate - obj.state.rearRollRate;

            % Infinite rigidity is a holonomic equality constraint, not a
            % numerically large penalty spring. Project any incoming mismatch
            % to the inertia-weighted common coordinate, then integrate one
            % shared roll DOF below.
            rigidTorsion = isinf(Kt);
            if rigidTorsion
                totalRollInertia = max( ...
                    obj.frontRollInertia + obj.rearRollInertia, eps);
                commonRollAngle = ...
                    (obj.frontRollInertia * obj.state.frontRollAngle + ...
                    obj.rearRollInertia * obj.state.rearRollAngle) / ...
                    totalRollInertia;
                commonRollRate = ...
                    (obj.frontRollInertia * obj.state.frontRollRate + ...
                    obj.rearRollInertia * obj.state.rearRollRate) / ...
                    totalRollInertia;
                obj.state.frontRollAngle = commonRollAngle;
                obj.state.rearRollAngle = commonRollAngle;
                obj.state.frontRollRate = commonRollRate;
                obj.state.rearRollRate = commonRollRate;
                twist = 0;
                twistRate = 0;
            end

            if useSuspensionReaction
                halfTrack = obj.trackWidth / 2;
                % Positive roll is right-side-down. Upward suspension force
                % on the right therefore supplies a negative/restoring
                % moment. Suspension damping and dynamically engaged bump
                % stops are already present in these reaction forces.
                frontSuspensionMoment = ...
                    (suspensionReaction.FL - suspensionReaction.FR) * halfTrack;
                rearSuspensionMoment = ...
                    (suspensionReaction.RL - suspensionReaction.RR) * halfTrack;
                frontRollMoment = rollMomentF + frontSuspensionMoment;
                rearRollMoment = rollMomentR + rearSuspensionMoment;
            else
                [KrollF, KrollR] = obj.getAxleRollStiffnessRad();
                CrollF = obj.rollDamping * massFrac;
                CrollR = obj.rollDamping * rearMassFrac;

                % If no per-axle stiffness is available, fall back to the
                % legacy whole-car rollStiffness split.
                if KrollF <= 0 && KrollR <= 0
                    KrollF = obj.rollStiffness * massFrac;
                    KrollR = obj.rollStiffness * rearMassFrac;
                end
                frontRollMoment = rollMomentF ...
                    - KrollF * obj.state.frontRollAngle ...
                    - CrollF * obj.state.frontRollRate;
                rearRollMoment = rollMomentR ...
                    - KrollR * obj.state.rearRollAngle ...
                    - CrollR * obj.state.rearRollRate;
            end

            if ~rigidTorsion
                torsionMoment = obj.safeTorsion(Kt, twist) + Ct * twistRate;
                frontRollMoment = frontRollMoment - torsionMoment;
                rearRollMoment = rearRollMoment + torsionMoment;
            end

            obj.state.heaveAccel = heaveForce / max(obj.sprungMass, eps);
            obj.state.pitchAccel = pitchMoment / max(obj.pitchInertia, eps);
            if rigidTorsion
                commonRollAccel = (frontRollMoment + rearRollMoment) / ...
                    totalRollInertia;
                obj.state.frontRollAccel = commonRollAccel;
                obj.state.rearRollAccel = commonRollAccel;
            else
                obj.state.frontRollAccel = frontRollMoment / ...
                    max(obj.frontRollInertia, eps);
                obj.state.rearRollAccel = rearRollMoment / ...
                    max(obj.rearRollInertia, eps);
            end

            obj.state.heaveRate = obj.state.heaveRate + obj.state.heaveAccel * dt;
            obj.state.pitchRate = obj.state.pitchRate + obj.state.pitchAccel * dt;

            obj.state.heave = obj.state.heave + obj.state.heaveRate * dt;
            obj.state.pitchAngle = obj.state.pitchAngle + obj.state.pitchRate * dt;
            if rigidTorsion
                commonRollRate = obj.state.frontRollRate + ...
                    obj.state.frontRollAccel * dt;
                commonRollAngle = obj.state.frontRollAngle + commonRollRate * dt;
                obj.state.frontRollRate = commonRollRate;
                obj.state.rearRollRate = commonRollRate;
                obj.state.frontRollAngle = commonRollAngle;
                obj.state.rearRollAngle = commonRollAngle;
            else
                obj.state.frontRollRate = obj.state.frontRollRate + ...
                    obj.state.frontRollAccel * dt;
                obj.state.rearRollRate = obj.state.rearRollRate + ...
                    obj.state.rearRollAccel * dt;
                obj.state.frontRollAngle = obj.state.frontRollAngle + ...
                    obj.state.frontRollRate * dt;
                obj.state.rearRollAngle = obj.state.rearRollAngle + ...
                    obj.state.rearRollRate * dt;
            end

            % Legacy whole-car roll state = average of the two axle DOFs.
            obj.state.rollAngle = 0.5 * (obj.state.frontRollAngle + obj.state.rearRollAngle);
            obj.state.rollRate  = 0.5 * (obj.state.frontRollRate  + obj.state.rearRollRate);
            obj.state.rollAccel = 0.5 * (obj.state.frontRollAccel + obj.state.rearRollAccel);

            obj.state.longitudinalLoadTransfer = ...
                (obj.totalMass * nonAeroAx * obj.cgHeight + ...
                dragPitchMoment) / max(obj.wheelbase, eps);
            obj.state.lateralLoadTransfer = ...
                (rollMomentF + rollMomentR) / max(obj.trackWidth, eps) + ...
                geoLatFront + geoLatRear + additionalLatFront + additionalLatRear;
            obj.state.additionalLongitudinalLoadTransfer = ...
                additionalLongitudinalTransfer;
            obj.state.frontAdditionalLateralLoadTransfer = additionalLatFront;
            obj.state.rearAdditionalLateralLoadTransfer = additionalLatRear;
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
        function [reaction, available] = getSuspensionReactionDeltas(obj)
            % GETSUSPENSIONREACTIONDELTAS Corner reactions above static load.
            % The SuspensionState forces act upward on the sprung chassis;
            % subtracting staticLoad keeps the chassis coordinates referenced
            % to their zero-force static equilibrium.
            reaction = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            available = ~isempty(obj.suspension) && ...
                isa(obj.suspension, 'lts.components.Suspension.SuspensionManager');
            if ~available
                return;
            end

            reaction.FL = obj.suspension.frontLeft.state.suspensionForce - ...
                obj.suspension.frontLeft.state.staticLoad;
            reaction.FR = obj.suspension.frontRight.state.suspensionForce - ...
                obj.suspension.frontRight.state.staticLoad;
            reaction.RL = obj.suspension.rearLeft.state.suspensionForce - ...
                obj.suspension.rearLeft.state.staticLoad;
            reaction.RR = obj.suspension.rearRight.state.suspensionForce - ...
                obj.suspension.rearRight.state.staticLoad;

            values = [reaction.FL, reaction.FR, reaction.RL, reaction.RR];
            if any(~isfinite(values))
                reaction = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
                available = false;
            end
        end

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
            % Reads the equivalent per-corner axle rate (mean spring tangent
            % rate + twice the ARB differential coupling rate) and converts
            % it to roll rate: K_roll = Kw*(t/2)^2*2 = Kw*t^2/2.
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
            % SAFETORSION Finite torsion-spring torque. Infinite stiffness is
            % handled as an exact constrained coordinate in the caller.
            torque = Kt * twist;
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
