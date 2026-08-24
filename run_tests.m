function run_tests()
% RUN_TESTS Standalone test runner for this component repository.
%   Assembles a +lts package sandbox under build/src (gitignored): this
%   repository's class files become +lts/+components/+Chassis, and the
%   lts-kit submodule's files become +lts/+util. Then runs the MATLAB
%   unittest suite in tests/.
%
%   Requires the lts-kit submodule:
%       git submodule update --init --recursive
root = fileparts(mfilename('fullpath'));
pkg = 'Chassis';

kitDir = fullfile(root, 'kit');
if ~isfolder(kitDir) || isempty(dir(fullfile(kitDir, '*.m')))
    error('run_tests:MissingKit', ['lts-kit is not initialized. Run: ' ...
        'git submodule update --init --recursive']);
end

sb = fullfile(root, 'build', 'src');
if exist(sb, 'dir')
    rmdir(sb, 's');
end

pkgDir = fullfile(sb, '+lts', '+components', ['+' pkg]);
mkdir(pkgDir);
copyfile(fullfile(root, '*.m'), pkgDir);
delete(fullfile(pkgDir, 'run_tests.m'));
if isfolder(fullfile(root, 'data'))
    copyfile(fullfile(root, 'data', '*'), fullfile(pkgDir, 'data'));
end

utilDir = fullfile(sb, '+lts', '+util');
mkdir(utilDir);
copyfile(fullfile(kitDir, '*.m'), utilDir);

addpath(sb);
suite = testsuite(fullfile(root, 'tests'));
runner = matlab.unittest.TestRunner.withTextOutput;
results = runner.run(suite);
assertSuccess(results);
end
