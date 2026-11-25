
function generate_sinusoidal_cycloidal

clc;

root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';

%% PATHS

linearFolder = fullfile(root,'results', ...
    'trajectory_methods','linear');

sinusoidalFolder = fullfile(root,'results', ...
    'trajectory_methods','sinusoidal');

cycloidalFolder = fullfile(root,'results', ...
    'trajectory_methods','cycloidal');

backupFolder = fullfile(root,'results', ...
    'trajectory_dashboard','backups');

if ~isfolder(sinusoidalFolder)
    mkdir(sinusoidalFolder);
end

if ~isfolder(cycloidalFolder)
    mkdir(cycloidalFolder);
end

if ~isfolder(backupFolder)
    mkdir(backupFolder);
end

sourceFile = fullfile(linearFolder,'linear_waypoints.mat');

sinusoidalFile = fullfile(sinusoidalFolder, ...
    'sinusoidal_waypoints.mat');

cycloidalFile = fullfile(cycloidalFolder, ...
    'cycloidal_waypoints.mat');

if ~isfile(sourceFile)
    error('Khong tim thay Linear waypoint: %s',sourceFile);
end

%% BACKUP OLD SINUSOIDAL DATA

if isfile(sinusoidalFile)

    backupFile = fullfile(backupFolder, ...
        ['sinusoidal_before_rebuild_' ...
        datestr(now,'yyyymmdd_HHMMSS') '.mat']);

    copyfile(sinusoidalFile,backupFile);

    fprintf('Da sao luu Sinusoidal cu:\n%s\n',backupFile);

end

%% LOAD LINEAR SOURCE

D = load(sourceFile);

sourceNames = {
    'linear_home2in'
    'linear_in2out'
    'linear_out2home'
};

sinusoidalNames = {
    'sinusoidal_home2in'
    'sinusoidal_in2out'
    'sinusoidal_out2home'
};

cycloidalNames = {
    'cycloidal_home2in'
    'cycloidal_in2out'
    'cycloidal_out2home'
};

sinusoidalResults = struct();
cycloidalResults = struct();

Ts = zeros(3,1);

%% GENERATE BOTH METHODS

fprintf('\n====================================\n');
fprintf('GENERATE SINUSOIDAL AND CYCLOIDAL\n');
fprintf('====================================\n');

for k = 1:3

    source = D.(sourceNames{k});

    validateSource(source,k);

    t = source.t(:)';
    t = t - t(1);

    T = t(end);

    N = numel(t);

    u = t/T;

    q0 = source.q(1:4,1);
    qf = source.q(1:4,end);

    delta = qf-q0;

    %% SINUSOIDAL: HALF-COSINE

    s = (1-cos(pi*u))/2;

    sd = (pi/(2*T))*sin(pi*u);

    sdd = (pi^2/(2*T^2))*cos(pi*u);

    sddd = -(pi^3/(2*T^3))*sin(pi*u);

    S = source;

    S.t = t;

    S.q(1:4,:) = q0 + delta*s;
    S.qd = delta*sd;
    S.qdd = delta*sdd;
    S.qddd = delta*sddd;

    % Preserve original Gripper trajectory.
    S.q(5,:) = source.q(5,:);

    S.method = 'Sinusoidal Half-Cosine';
    S.sampleTime = median(diff(t));

    S.originalWaypoints = source.q(:,[1 end]);
    S.originalTime = source.t;

    sinusoidalResults.(segmentName(k)) = S;

    %% CYCLOIDAL

    c = u-sin(2*pi*u)/(2*pi);

    cd = (1-cos(2*pi*u))/T;

    cdd = (2*pi/T^2)*sin(2*pi*u);

    cddd = (4*pi^2/T^3)*cos(2*pi*u);

    C = source;

    C.t = t;

    C.q(1:4,:) = q0 + delta*c;
    C.qd = delta*cd;
    C.qdd = delta*cdd;
    C.qddd = delta*cddd;

    % Preserve original Gripper trajectory.
    C.q(5,:) = source.q(5,:);

    C.method = 'Cycloidal';
    C.sampleTime = median(diff(t));

    C.originalWaypoints = source.q(:,[1 end]);
    C.originalTime = source.t;

    cycloidalResults.(segmentName(k)) = C;

    Ts(k) = median(diff(t));

    %% CHECK RESULTS

    checkSegment(S,source,'Sinusoidal',k);
    checkSegment(C,source,'Cycloidal',k);

    fprintf('\nSegment %d: %s\n',k,segmentName(k));
    fprintf('Duration: %.4f s\n',T);
    fprintf('Samples: %d\n',N);
    fprintf('Sample time: %.8f s\n',Ts(k));

end

%% CREATE NAMED VARIABLES FOR SINUSOIDAL

sinusoidal_home2in = sinusoidalResults.home2in;
sinusoidal_in2out = sinusoidalResults.in2out;
sinusoidal_out2home = sinusoidalResults.out2home;

%% CREATE NAMED VARIABLES FOR CYCLOIDAL

cycloidal_home2in = cycloidalResults.home2in;
cycloidal_in2out = cycloidalResults.in2out;
cycloidal_out2home = cycloidalResults.out2home;

names = {
    'home2in'
    'in2out'
    'out2home'
};

%% SAVE SINUSOIDAL

save(sinusoidalFile, ...
    'sinusoidalResults', ...
    'sinusoidal_home2in', ...
    'sinusoidal_in2out', ...
    'sinusoidal_out2home', ...
    'Ts', ...
    'names', ...
    '-v7.3');

%% SAVE CYCLOIDAL

save(cycloidalFile, ...
    'cycloidalResults', ...
    'cycloidal_home2in', ...
    'cycloidal_in2out', ...
    'cycloidal_out2home', ...
    'Ts', ...
    'names', ...
    '-v7.3');

%% VERIFY FILES

fprintf('\n====================================\n');
fprintf('VERIFY GENERATED FILES\n');
fprintf('====================================\n');

verifyFile(sinusoidalFile, ...
    {'sinusoidal_home2in', ...
     'sinusoidal_in2out', ...
     'sinusoidal_out2home'});

verifyFile(cycloidalFile, ...
    {'cycloidal_home2in', ...
     'cycloidal_in2out', ...
     'cycloidal_out2home'});

fprintf('\nSINUSOIDAL FILE:\n%s\n',sinusoidalFile);

fprintf('\nCYCLOIDAL FILE:\n%s\n',cycloidalFile);

fprintf('\nDA TAO THANH CONG HAI BO QUY DAO.\n');

end

%% SEGMENT NAME

function name = segmentName(k)

switch k
    case 1
        name = 'home2in';
    case 2
        name = 'in2out';
    case 3
        name = 'out2home';
    otherwise
        error('Segment index khong hop le.');
end

end

%% VALIDATE SOURCE

function validateSource(S,k)

required = {'t','q','qd','qdd','qddd'};

for n = 1:numel(required)

    if ~isfield(S,required{n})
        error('Source segment %d thieu truong %s.', ...
            k,required{n});
    end

end

N = numel(S.t);

if N < 2
    error('Segment %d khong du mau.',k);
end

if size(S.q,1) ~= 5 || size(S.q,2) ~= N
    error('Kich thuoc q segment %d khong hop le.',k);
end

fields = {'qd','qdd','qddd'};

for n = 1:numel(fields)

    A = S.(fields{n});

    if size(A,1) ~= 4 || size(A,2) ~= N
        error('Kich thuoc %s segment %d khong hop le.', ...
            fields{n},k);
    end

end

if any(~isfinite(S.t(:))) || any(diff(S.t(:)) <= 0)
    error('Thoi gian source segment %d khong hop le.',k);
end

if any(~isfinite(S.q(:)))
    error('q source segment %d co NaN hoac Inf.',k);
end

end

%% CHECK GENERATED SEGMENT

function checkSegment(A,source,method,k)

N = numel(A.t);

if size(A.q,1) ~= 5 || size(A.q,2) ~= N
    error('%s segment %d: q sai kich thuoc.',method,k);
end

if size(A.qd,1) ~= 4 || size(A.qd,2) ~= N
    error('%s segment %d: qd sai kich thuoc.',method,k);
end

if size(A.qdd,1) ~= 4 || size(A.qdd,2) ~= N
    error('%s segment %d: qdd sai kich thuoc.',method,k);
end

if size(A.qddd,1) ~= 4 || size(A.qddd,2) ~= N
    error('%s segment %d: qddd sai kich thuoc.',method,k);
end

allData = [
    A.t(:)
    A.q(:)
    A.qd(:)
    A.qdd(:)
    A.qddd(:)
];

if any(~isfinite(allData))
    error('%s segment %d co NaN hoac Inf.',method,k);
end

positionError = max(abs( ...
    A.q(1:4,[1 end])-source.q(1:4,[1 end])),[],'all');

gripperError = max(abs(A.q(5,:)-source.q(5,:)));

if positionError > 1e-10
    error('%s segment %d sai vi tri dau/cuoi.',method,k);
end

if gripperError > 1e-12
    error('%s segment %d lam thay doi Gripper.',method,k);
end

if any(diff(A.t) <= 0)
    error('%s segment %d co thoi gian khong tang.',method,k);
end

fprintf('%s position error: %.12g\n',method,positionError);
fprintf('%s Gripper error: %.12g\n',method,gripperError);

fprintf('%s endpoint velocity:\n',method);
disp(A.qd(:,[1 end]));

fprintf('%s endpoint acceleration:\n',method);
disp(A.qdd(:,[1 end]));

end

%% VERIFY SAVED FILE

function verifyFile(file,required)

if ~isfile(file)
    error('Khong tao duoc file: %s',file);
end

V = load(file);

for k = 1:numel(required)

    if ~isfield(V,required{k})
        error('File %s thieu bien %s.',file,required{k});
    end

end

info = dir(file);

fprintf('\nVerified: %s\n',file);
fprintf('File size: %.3f MB\n',info.bytes/1024^2);

end

