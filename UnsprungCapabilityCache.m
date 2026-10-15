classdef UnsprungCapabilityCache < handle
    % Cache capabilities, never masses or corner handles. Shared caches in
    % value-model copies stay safe because the corner classes are the key.
    properties
        cornerClasses = {}
        hasMass = false
    end
end
