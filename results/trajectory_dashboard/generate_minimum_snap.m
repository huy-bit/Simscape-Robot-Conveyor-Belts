function generate_minimum_snap()
% Generate Minimum Snap trajectories for the 6DOF robot.
% Source profile: Minimum_Snap_trajectory.csv
% Output: minimum_snap_waypoints.mat

root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';
csvFile = fullfile(root,'results','trajectory_methods', ...
    'Minimum_Snap_trajectory.csv');
linearFile = fullfile(root,'results','trajectory_methods', ...
    'linear','linear_waypoints.mat');
outFolder = fullfile(root,'results','trajectory_methods', ...
    'minimum_snap');
outFile = fullfile(outFolder,'minimum_snap_waypoints.mat');

assert(isfile(csvFile),'Khong tim thay Minimum Snap CSV.');
assert(isfile(linearFile),'Khong tim thay Linear waypoint MAT.');

T = readtable(csvFile);
A = table2array(T);
assert(size(A,2)==17,'CSV phai co 17 cot.');
assert(all(isfinite(A(:))),'CSV co du lieu khong huu han.');

sourceTime = A(:,1);
sourceQ = A(:,2:5);
sourceQd = A(:,6:9);
sourceQdd = A(:,10:13);
sourceQddd = A(:,14:17);

sourceDuration = sourceTime(end)-sourceTime(1);
assert(sourceDuration>0,'Thoi gian nguon khong hop le.');

delta = sourceQ(end,1)-sourceQ(1,1);
assert(abs(delta)>eps,'Bien do Pivot khong hop le.');

% Normalize source profile using Pivot.
sSource = (sourceQ(:,1)-sourceQ(1,1))/delta;
sdSource = sourceQd(:,1)/delta;
sddSource = sourceQdd(:,1)/delta;
sdddSource = sourceQddd(:,1)/delta;
uSource = (sourceTime-sourceTime(1))/sourceDuration;

linear = load(linearFile);
sourceNames = { ...
    'linear_home2in', ...
    'linear_in2out', ...
    'linear_out2home'};
targetNames = { ...
    'minimum_snap_home2in', ...
    'minimum_snap_in2out', ...
    'minimum_snap_out2home'};

results = struct();
Ts = 0.002;

for k = 1:3
    src = linear.(sourceNames{k});
    t = src.t(:).';
    Ttarget = t(end)-t(1);
    u = (t-t(1))/Ttarget;

    % Interpolate normalized position and derivatives.
    s = interp1(uSource,sSource,u,'pchip');
    sd = interp1(uSource,sdSource,u,'pchip') ...
        * (sourceDuration/Ttarget);
    sdd = interp1(uSource,sddSource,u,'pchip') ...
        * (sourceDuration/Ttarget)^2;
    sddd = interp1(uSource,sdddSource,u,'pchip') ...
        * (sourceDuration/Ttarget)^3;

    q0 = src.q(1:4,1);
    qf = src.q(1:4,end);
    jointDelta = qf-q0;

    out = struct();
    out.t = t;
    out.q = zeros(5,numel(t));
    out.qd = zeros(4,numel(t));
    out.qdd = zeros(4,numel(t));
    out.qddd = zeros(4,numel(t));

    for j = 1:4
        out.q(j,:) = q0(j)+jointDelta(j)*s;
        out.qd(j,:) = jointDelta(j)*sd;
        out.qdd(j,:) = jointDelta(j)*sdd;
        out.qddd(j,:) = jointDelta(j)*sddd;
    end

    % Preserve the original Gripper signal exactly.
    out.q(5,:) = src.q(5,:);
    out.method = 'Minimum Snap';
    out.sampleTime = Ts;
    out.originalWaypoints = src.originalWaypoints;
    out.originalTime = src.originalTime;

    results.(targetNames{k}) = out;
    fprintf('\n%s\n',targetNames{k});
    fprintf('Duration: %.6f s\n',Ttarget);
    fprintf('Samples: %d\n',numel(t));
    fprintf('Median sample time: %.8f s\n',median(diff(t)));
    fprintf('Position endpoint error: %.12g\n', ...
        max(abs([out.q(1:4,1)-q0; out.q(1:4,end)-qf])));
    fprintf('Finite: %d\n',all(isfinite([out.q(:); ...
        out.qd(:);out.qdd(:);out.qddd(:)])));
    fprintf('Gripper preserved: %d\n', ...
        isequal(out.q(5,:),src.q(5,:)));
end

minimum_snap_home2in = results.minimum_snap_home2in;
minimum_snap_in2out = results.minimum_snap_in2out;
minimum_snap_out2home = results.minimum_snap_out2home;

if ~isfolder(outFolder)
    mkdir(outFolder);
end
save(outFile,'results','minimum_snap_home2in', ...
    'minimum_snap_in2out','minimum_snap_out2home','Ts','targetNames');

fprintf('\nSaved: %s\n',outFile);
fprintf('File size: %.3f MB\n',dir(outFile).bytes/1024^2);
end
