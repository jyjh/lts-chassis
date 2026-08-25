function tests = ConformanceTest
% CONFORMANCETEST Pin the contract between this repository and the main
% lts repository (contract items 1-3 of the repository split; see the
% Contracts page of the main repository's documentation).
%
%  1. cfg schema — lts.components.Chassis.validateConfig accepts the
%     canonical cfg.chassis and rejects bad input with typed errors.
%  2. Interface — SimpleChassis subclasses ChassisComponent, implements
%     the ABC methods, and satisfies the structural probes the Simulator
%     and VehicleState perform (roll/twist accessors).
%  3. Telemetry producer fields — the ChassisState property names below
%     feed the pitch/roll/twist/ride-height telemetry channels and the
%     aero pitch moments in lts.telemetry.StateLogBuilder. Renaming any
%     of them is a contract change: update this test and the Contracts
%     page in the same PR, then coordinate the main-repository change
%     (see "Changing the contract" there).
tests = functiontests(localfunctions);
end

function cfg = canonicalConfig()
% Mirrors the baseline cfg.chassis from the main repository's VehicleConfig.
cfg = struct( ...
    'heaveStiffness', 160000, ...
    'heaveDamping', 12000, ...
    'pitchStiffness', 90000, ...
    'pitchDamping', 6000, ...
    'rollStiffness', 55000, ...
    'rollDamping', 5000, ...
    'torsionalRigidity', 229183, ...
    'torsionalDamping', 2000);
end

function chassis = chassisFixture()
% Mirror lts.vehicle.VehicleManager.fromConfig: sprung mass plus the
% eight platform coefficients, then settle.
chassis = lts.components.Chassis.SimpleChassis(vehicleFixture(), 226.8);
chassis.heaveStiffness = 160000;
chassis.heaveDamping = 12000;
chassis.pitchStiffness = 90000;
chassis.pitchDamping = 6000;
chassis.rollStiffness = 55000;
chassis.rollDamping = 5000;
chassis.torsionalRigidity = 229183;
chassis.torsionalDamping = 2000;
chassis.reset();
end

function vm = vehicleFixture()
vm = struct( ...
    'totalMass', 264, ...
    'wheelbase', 1.558, ...
    'trackWidth', 1.21, ...
    'cgHeight', 0.30, ...
    'staticFrontWeight', 0.5038, ...
    'tire', []);
end

%% ---- 1. Config schema --------------------------------------------------

function testValidateConfigAcceptsCanonicalConfig(testCase)
verifyTrue(testCase, ...
    lts.components.Chassis.validateConfig(canonicalConfig()));
end

function testValidateConfigAcceptsInfiniteTorsionalRigidity(testCase)
% +Inf is the supported exact rigid-torsion constraint.
cfg = canonicalConfig();
cfg.torsionalRigidity = Inf;
verifyTrue(testCase, ...
    lts.components.Chassis.validateConfig(cfg));
end

function testValidateConfigRejectsMissingRequiredField(testCase)
cfg = canonicalConfig();
cfg = rmfield(cfg, 'rollStiffness');
verifyError(testCase, ...
    @() lts.components.Chassis.validateConfig(cfg), ...
    'lts_chassis_validateConfig:MissingField');
end

function testValidateConfigRejectsNegativeStiffness(testCase)
cfg = canonicalConfig();
cfg.heaveStiffness = -1;
verifyError(testCase, ...
    @() lts.components.Chassis.validateConfig(cfg), ...
    'lts_chassis_validateConfig:InvalidScalar');
end

function testValidateConfigRejectsNanTorsionalRigidity(testCase)
cfg = canonicalConfig();
cfg.torsionalRigidity = NaN;
verifyError(testCase, ...
    @() lts.components.Chassis.validateConfig(cfg), ...
    'lts_chassis_validateConfig:InvalidScalar');
end

%% ---- 2. Interface (contract item 1) -------------------------------------

function testSimpleChassisSubclassesChassisComponent(testCase)
mc = meta.class.fromName('lts.components.Chassis.SimpleChassis');
supers = {mc.SuperclassList.Name};
verifyTrue(testCase, ...
    any(endsWith(supers, 'ChassisComponent')), ...
    'SimpleChassis must subclass ChassisComponent.');
end

function testSimpleChassisImplementsAbstractInterface(testCase)
abc = meta.class.fromName('lts.components.Chassis.ChassisComponent');
mc = meta.class.fromName('lts.components.Chassis.SimpleChassis');
methods = {mc.MethodList.Name};
props = {mc.PropertyList.Name};
isAbs = [abc.MethodList.Abstract];
abstractMethods = {abc.MethodList(isAbs).Name};
for i = 1:numel(abstractMethods)
    verifyTrue(testCase, ismember(abstractMethods{i}, methods), ...
        sprintf('SimpleChassis must implement %s.', abstractMethods{i}));
end
isAbstract = [abc.PropertyList.Abstract];
abstractProps = {abc.PropertyList(isAbstract).Name};
for i = 1:numel(abstractProps)
    verifyTrue(testCase, ismember(abstractProps{i}, props), ...
        sprintf('SimpleChassis must implement property %s.', ...
        abstractProps{i}));
end
end

function testSimpleChassisSatisfiesSimulatorStructuralProbes(testCase)
% VehicleState probes these accessors with ismethod to populate the
% rollRate/frontRollAngle/rearRollAngle/twist telemetry channels; the
% Simulator reads the aero pitch moments off state directly.
chassis = chassisFixture();
probes = {'getRollRate', 'getFrontRollRate', 'getRearRollRate', ...
    'getFrontRollAngle', 'getRearRollAngle', ...
    'getTwistAngle', 'getTwistRate', 'getPitchAngle', ...
    'getRollAngle', 'getHeave', 'reset', ...
    'updateFromAccelerations', 'computeCornerKinematics'};
for i = 1:numel(probes)
    verifyTrue(testCase, ismethod(chassis, probes{i}), ...
        sprintf('SimpleChassis must implement %s.', probes{i}));
end
end

%% ---- 3. Telemetry producer fields (contract item 3) ---------------------

function testChassisStatePinsTelemetryFieldNames(testCase)
% Exact property names StateLogBuilder/VehicleState/Simulator consume
% for the attitude channels (pitchAngle, rollAngle, rollRate,
% front/rearRollAngle/Rate, twist via front-rear, rideHeight via heave)
% and the aero pitch-moment fields.
mc = meta.class.fromName('lts.components.Chassis.ChassisState');
props = {mc.PropertyList.Name};
required = {'heave', 'pitchAngle', 'pitchRate', ...
    'rollAngle', 'rollRate', ...
    'frontRollAngle', 'frontRollRate', ...
    'rearRollAngle', 'rearRollRate', ...
    'cornerDisplacement', 'cornerVelocity', ...
    'downforcePitchMoment', 'dragPitchMoment', 'aeroPitchMoment'};
for i = 1:numel(required)
    verifyTrue(testCase, ismember(required{i}, props), ...
        sprintf('ChassisState must keep property %s.', required{i}));
end
end

function testCornerKinematicsCornerNames(testCase)
% computeCornerKinematics must key its corners FL/FR/RL/RR — the names
% the suspension and per-corner telemetry use.
chassis = chassisFixture();
chassis.updateFromAccelerations(1.0, 1.0, struct(), 0.001, 0);
ck = chassis.computeCornerKinematics();
corners = {'displacement', 'velocity'};
for i = 1:numel(corners)
    names = fieldnames(ck.(corners{i}));
    verifyEqual(testCase, sort(names), {'FL'; 'FR'; 'RL'; 'RR'});
end
end
