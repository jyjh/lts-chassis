function tests = ChassisSmokeTest
tests = functiontests(localfunctions);
end

function vm = vehicleFixture()
% Plain struct with exactly the fields SimpleChassis reads at construction
% (tire left empty = legacy cgHeight hub fallback) — proves the package
% works without the main repository's classes.
vm = struct( ...
    'totalMass', 264, ...
    'wheelbase', 1.558, ...
    'trackWidth', 1.21, ...
    'cgHeight', 0.30, ...
    'staticFrontWeight', 0.5038, ...
    'tire', []);
end

function chassis = chassisFixture()
chassis = lts.components.Chassis.SimpleChassis(vehicleFixture(), 226.8, 60, 35);
end

function testConstructorDerivesInertiasAndArms(testCase)
chassis = chassisFixture();
verifyGreaterThan(testCase, chassis.pitchInertia, 0);
verifyGreaterThan(testCase, chassis.rollInertia, 0);
verifyEqual(testCase, chassis.frontArm + chassis.rearArm, 1.558, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, chassis.hubHeight, 0.30, ...
    'empty tire must fall back to cgHeight');
end

function testUpdateFromAccelerationsStaysFinite(testCase)
chassis = chassisFixture();
for step = 1:50
    chassis.updateFromAccelerations(2.0, 3.0, struct(), 0.001, 0);
end
verifyTrue(testCase, isfinite(chassis.getHeave()));
verifyTrue(testCase, isfinite(chassis.getPitchAngle()));
verifyTrue(testCase, isfinite(chassis.getRollAngle()));
end

function testCornerKinematicsShape(testCase)
% The return shape consumed by the suspension's
% computeCornerLoadsFromChassis: displacement/velocity per corner.
chassis = chassisFixture();
chassis.updateFromAccelerations(1.0, 1.0, struct(), 0.001, 0);
ck = chassis.computeCornerKinematics();
required = {'FL', 'FR', 'RL', 'RR'};
verifyTrue(testCase, isfield(ck, 'displacement'));
verifyTrue(testCase, isfield(ck, 'velocity'));
for i = 1:numel(required)
    verifyTrue(testCase, isfinite(ck.displacement.(required{i})));
    verifyTrue(testCase, isfinite(ck.velocity.(required{i})));
end
end

function testSetSuspensionAcceptsEmpty(testCase)
% With no linked suspension the chassis must run on its linear platform
% fallback, not error.
chassis = chassisFixture().setSuspension([]);
chassis.updateFromAccelerations(1.0, 1.0, struct(), 0.001, 0);
verifyTrue(testCase, isfinite(chassis.getRollAngle()));
end
