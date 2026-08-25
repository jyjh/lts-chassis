function ok = validateConfig(cfg)
% VALIDATECONFIG Validate the cfg.chassis struct consumed by this package.
%   ok = lts.components.Chassis.validateConfig(cfg)
%
% Contract item 2 of the repository split (see the Contracts page of the
% main repository's documentation): each component repository owns the
% schema of its cfg sub-struct and validates it at the build boundary.
%
% Fields (all SI, all finite real scalars, all >= 0 unless noted):
%   heaveStiffness    [N/m]      fallback platform heave stiffness
%   heaveDamping      [N*s/m]    fallback platform heave damping
%   pitchStiffness    [N*m/rad]  fallback platform pitch stiffness
%   pitchDamping      [N*m*s/rad] fallback platform pitch damping
%   rollStiffness     [N*m/rad]  fallback platform roll stiffness
%   rollDamping       [N*m*s/rad] fallback platform roll damping
%   torsionalRigidity [N*m/rad]  couples front/rear roll DOFs; +Inf is
%                                the supported exact rigid constraint
%   torsionalDamping  [N*m*s/rad] damps the twist rate
%
% Returns logical true on success; otherwise throws with identifier
% lts_chassis_validateConfig:<Case> (MissingField | InvalidScalar |
% OutOfRange).

required = {'heaveStiffness', 'heaveDamping', 'pitchStiffness', ...
    'pitchDamping', 'rollStiffness', 'rollDamping', ...
    'torsionalRigidity', 'torsionalDamping'};
for i = 1:numel(required)
    if ~isfield(cfg, required{i}) || isempty(cfg.(required{i}))
        error('lts_chassis_validateConfig:MissingField', ...
            'cfg.chassis.%s is required.', required{i});
    end
end

for i = 1:numel(required)
    name = required{i};
    value = cfg.(name);
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ...
            isnan(value) || value < 0 || value == -Inf
        error('lts_chassis_validateConfig:InvalidScalar', ...
            ['cfg.chassis.%s must be a nonnegative real scalar ' ...
            '(+Inf allowed for torsionalRigidity; got %s).'], ...
            name, mat2str(value));
    end
end

ok = true;
end
