function result = run_forward_kinematics_example
%RUN_FORWARD_KINEMATICS_EXAMPLE Reconstruct five known interior positions.
%   RESULT = RUN_FORWARD_KINEMATICS_EXAMPLE needs only MATLAB. It derives
%   cable lengths from a centre point and one point in each quadrant, then
%   converts the lengths to the signed inputs of FORWARD_KINEMATICS.
%   All lengths, displacements, positions and errors are in millimetres.
%
%   RESULT fields: anchors_mm (2x4); known_position_mm and position_mm
%   (2x5, bottom-left frame); centred_position_mm (2x5); cable_lengths_mm
%   and motor_displacements_mm (4x5, controller cable order); error_mm
%   (1x5, Euclidean norm); passed (logical scalar, errors <= 1e-8 mm).
%   Cable rows: 1 bottom-left, 2 top-left, 3 top-right, 4 bottom-right.
%   The function has no persistent state. This example records no timings.

root = fileparts(fileparts(mfilename('fullpath')));
original_path = path;
path_cleanup = onCleanup(@() path(original_path)); %#ok<NASGU>
addpath(fullfile(root,'src'));

anchors = [0 0 630 630; 0 630 630 0];
known = [315 395 235 235 395; 315 360 360 270 270];
reference_length = 630*sqrt(2)/2;
lengths = zeros(4,size(known,2));
motors = zeros(size(lengths));
positions = zeros(size(known));
errors = zeros(1,size(known,2));

fprintf('\nExperimental CDPR forward kinematics\n');
fprintf('All quantities below are in mm. Cable order: BL, TL, TR, BR.\n');
for k = 1:size(known,2)
    lengths(:,k) = sqrt(sum((anchors-known(:,k)).^2,1))';
    motors(:,k) = [reference_length-lengths(1,k); ...
        lengths(2,k)-reference_length; reference_length-lengths(3,k); ...
        lengths(4,k)-reference_length];
    positions(:,k) = forward_kinematics(motors(1,k),motors(2,k), ...
        motors(3,k),motors(4,k));
    errors(k) = norm(positions(:,k)-known(:,k),2);
    fprintf('\nPoint %d\n',k);
    fprintf('  Known position [x y]:       [% .9f % .9f]\n',known(:,k));
    fprintf('  Cable lengths [l1..l4]:     [% .9f % .9f % .9f % .9f]\n',lengths(:,k));
    fprintf('  Function inputs [M1..M4]:   [% .9f % .9f % .9f % .9f]\n',motors(:,k));
    fprintf('  Reconstructed [x y]:       [% .9f % .9f]\n',positions(:,k));
    fprintf('  Frame-centred [x y]:       [% .9f % .9f]\n',positions(:,k)-[315;315]);
    fprintf('  Position error:            %.3e\n',errors(k));
end
passed = isreal(positions) && all(isfinite(positions(:))) && all(errors<=1e-8);
assert(passed,'run_forward_kinematics_example:PositionError', ...
    'A reconstructed position differs from its known position by more than 1e-8 mm.');
result = struct('anchors_mm',anchors,'known_position_mm',known, ...
    'position_mm',positions,'centred_position_mm',positions-[315;315], ...
    'cable_lengths_mm',lengths,'motor_displacements_mm',motors, ...
    'error_mm',errors,'passed',passed);
fprintf('\nAll five checks passed.\n');
end
