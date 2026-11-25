function generate_bezier

clc;

root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';

inputFile = fullfile(root,'results','trajectory_methods', ...
    'linear','linear_waypoints.mat');

outFolder = fullfile(root,'results','trajectory_methods','bezier');
outFile = fullfile(outFolder,'bezier_waypoints.mat');

if ~isfile(inputFile)
    error('Khong tim thay Linear waypoint MAT.');
end

if ~isfolder(outFolder)
    mkdir(outFolder);
end

D = load(inputFile);

sourceNames = {
    'linear_home2in'
    'linear_in2out'
    'linear_out2home'
};

outputNames = {
    'bezier_home2in'
    'bezier_in2out'
    'bezier_out2home'
};

for k = 1:3
    if ~isfield(D,sourceNames{k})
        error('Thieu bien dau vao: %s',sourceNames{k});
    end
end

if isfile(outFile)
    backupFile = fullfile(outFolder, ...
        ['bezier_waypoints_before_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
    copyfile(outFile,backupFile);
    fprintf('Da sao luu: %s\n',backupFile);
end

results = struct();
Ts = 0.002;
method = 'Bezier Quintic';
names = outputNames;

fprintf('\n===== GENERATE BEZIER QUINTIC =====\n');

for k = 1:3

    src = D.(sourceNames{k});
    T = src.t(end) - src.t(1);
    N = numel(src.t);
    t = src.t - src.t(1);
    u = t/T;

    s = 10*u.^3 - 15*u.^4 + 6*u.^5;
    sd = (30*u.^2 - 60*u.^3 + 30*u.^4)/T;
    sdd = (60*u - 180*u.^2 + 120*u.^3)/T^2;
    sddd = (60 - 360*u + 360*u.^2)/T^3;

    q = zeros(5,N);
    qd = zeros(4,N);
    qdd = zeros(4,N);
    qddd = zeros(4,N);

    q0 = src.q(1:4,1);
    qf = src.q(1:4,end);
    delta = qf - q0;

    for j = 1:4
        q(j,:) = q0(j) + delta(j)*s;
        qd(j,:) = delta(j)*sd;
        qdd(j,:) = delta(j)*sdd;
        qddd(j,:) = delta(j)*sddd;
    end

    q(5,:) = src.q(5,:);

    S = struct();
    S.t = t;
    S.q = q;
    S.qd = qd;
    S.qdd = qdd;
    S.qddd = qddd;
    S.method = method;
    S.sampleTime = median(diff(t));
    S.originalWaypoints = [q0 qf];
    S.originalTime = T;

    results.(outputNames{k}) = S;
    assignin('base',outputNames{k},S);

    fprintf('\nDoan %d: %s\n',k,outputNames{k});
    fprintf('Thoi gian: %.4f s\n',T);
    fprintf('So mau: %d\n',N);
    fprintf('Sample time: %.8f s\n',S.sampleTime);
    fprintf('Sai so vi tri dau/cuoi: %.3g\n', ...
        max(abs([q(:,1)-src.q(:,1); q(:,end)-src.q(:,end)])));
    fprintf('Van toc dau/cuoi: %.3g / %.3g\n', ...
        max(abs(qd(:,1))),max(abs(qd(:,end))));
    fprintf('Gia toc dau/cuoi: %.3g / %.3g\n', ...
        max(abs(qdd(:,1))),max(abs(qdd(:,end))));
    fprintf('Du lieu huu han: %d\n', ...
        all(isfinite([q(:);qd(:);qdd(:);qddd(:)])));

end

bezier_home2in = results.bezier_home2in;
bezier_in2out = results.bezier_in2out;
bezier_out2home = results.bezier_out2home;

save(outFile,'results','bezier_home2in','bezier_in2out', ...
    'bezier_out2home','Ts','names','-v7.3');

fprintf('\n===== KIEM TRA FILE =====\n');
assert(isfile(outFile));
info = dir(outFile);
fprintf('File: %s\n',outFile);
fprintf('Dung luong: %.3f MB\n',info.bytes/1024^2);
fprintf('\nTAO BEZIER THANH CONG.\n');

end
