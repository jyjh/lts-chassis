classdef UnsprungSuspensionSpy < handle
    properties
        frontLeft
        frontRight
        rearLeft
        rearRight
        frontRollCenterHeight = .03
        rearRollCenterHeight = .04
        frontRollCenterLateral = 0
        rearRollCenterLateral = 0
        frontAntiDiveFraction = 0
        rearAntiSquatFraction = 0
        frontRate = 45000
        rearRate = 42000
    end
    methods
        function obj = UnsprungSuspensionSpy()
            obj.frontLeft = UnsprungCornerSpy();
            obj.frontRight = UnsprungCornerSpy();
            obj.rearLeft = UnsprungCornerSpy();
            obj.rearRight = UnsprungCornerSpy();
        end
        function [front, rear] = getAxleRollStiffness(obj)
            front = obj.frontRate;
            rear = obj.rearRate;
        end
        function frac = deriveFrontRollStiffnessFraction(obj)
            frac = obj.frontRate / (obj.frontRate + obj.rearRate);
        end
    end
end
