function tests = UnsprungTransferTest
tests = functiontests(localfunctions);
end

function [c, s] = fixture()
vm = struct('totalMass', 256, 'wheelbase', 1.558, 'trackWidth', 1.21, ...
    'cgHeight', .3, 'staticFrontWeight', .6, 'tire', []);
c = lts.components.Chassis.SimpleChassis(vm, 220);
c.hubHeight = .22;
c.torsionalRigidity = Inf;
c.torsionalDamping = 0;
s = UnsprungSuspensionSpy();
c = c.setSuspension(s);
end

function testUnsprungTransferIsAxleLocalAndIndependentOfBarRate(testCase)
[c, s] = fixture();
for rate = [1000, 45000, 1e6]
    s.frontRate = rate;
    c.updateFromAccelerations(0, 8, struct(), 0, 2);
    expectedF = 18 * (8 + 2 * c.frontArm) * .22 / c.trackWidth;
    expectedR = 18 * (8 - 2 * c.rearArm) * .22 / c.trackWidth;
    verifyEqual(testCase, c.state.frontAdditionalLateralLoadTransfer, expectedF, 'AbsTol', 1e-12);
    verifyEqual(testCase, c.state.rearAdditionalLateralLoadTransfer, expectedR, 'AbsTol', 1e-12);
end
end

function testMassHeightMomentIsConserved(testCase)
[c, ~] = fixture();
c.updateFromAccelerations(4, 8, struct(), 0, 0);
verifyEqual(testCase, c.state.lateralLoadTransfer, ...
    c.totalMass * 8 * c.cgHeight / c.trackWidth, 'AbsTol', 1e-10);
sprungPitchMoment = c.state.pitchAccel * c.pitchInertia;
hubPitchMoment = c.state.additionalLongitudinalLoadTransfer * c.wheelbase;
verifyEqual(testCase, sprungPitchMoment + hubPitchMoment, ...
    c.totalMass * 4 * c.cgHeight, 'AbsTol', 1e-10);
end

function testAxleMassesAndSignArePreserved(testCase)
[c, s] = fixture();
s.frontLeft.unsprungMass = 7;
s.frontRight.unsprungMass = 7;
s.rearLeft.unsprungMass = 11;
s.rearRight.unsprungMass = 11;
c.updateFromAccelerations(0, -6, struct(), 0, 0);
verifyEqual(testCase, c.state.frontAdditionalLateralLoadTransfer, ...
    14 * -6 * .22 / c.trackWidth, 'AbsTol', 1e-12);
verifyEqual(testCase, c.state.rearAdditionalLateralLoadTransfer, ...
    22 * -6 * .22 / c.trackWidth, 'AbsTol', 1e-12);
end

function testCachedCapabilityKeepsMassesAndReplacementCornersLive(testCase)
[c, s] = fixture();
c.updateFromAccelerations(0,6,struct(),0,0);
s.frontLeft.unsprungMass = 7; s.frontRight.unsprungMass = 7;
s.rearLeft.unsprungMass = 11; s.rearRight.unsprungMass = 11;
c.updateFromAccelerations(0,6,struct(),0,0);
verifyEqual(testCase,c.state.frontAdditionalLateralLoadTransfer, ...
    14*6*.22/c.trackWidth,'AbsTol',1e-12);
s.frontLeft = UnsprungCornerSpy(); s.frontLeft.unsprungMass = 7;
c.updateFromAccelerations(0,6,struct(),0,0);
verifyEqual(testCase,c.state.frontAdditionalLateralLoadTransfer, ...
    14*6*.22/c.trackWidth,'AbsTol',1e-12);
s.frontLeft.unsprungMass = 8;
verifyError(testCase,@() c.updateFromAccelerations(0,6,struct(),0,0), ...
    'lts_chassis_SimpleChassis:InvalidSprungMass');
end

function testDynamicMassPropertyInvalidatesCapability(testCase)
[c,s] = fixture();
s.frontLeft = DynamicUnsprungCornerSpy();
c.updateFromAccelerations(0,6,struct(),0,0);
verifyEqual(testCase,c.state.frontAdditionalLateralLoadTransfer, ...
    36*.6*6*.22/c.trackWidth,'AbsTol',1e-12);
p = addprop(s.frontLeft,'unsprungMass'); s.frontLeft.unsprungMass = 9;
c.updateFromAccelerations(0,6,struct(),0,0);
verifyEqual(testCase,c.state.frontAdditionalLateralLoadTransfer, ...
    18*6*.22/c.trackWidth,'AbsTol',1e-12);
delete(p);
c.updateFromAccelerations(0,6,struct(),0,0);
verifyEqual(testCase,c.state.frontAdditionalLateralLoadTransfer, ...
    36*.6*6*.22/c.trackWidth,'AbsTol',1e-12);
end
