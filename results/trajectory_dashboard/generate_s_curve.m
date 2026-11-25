function generate_s_curve

clc;

root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';

linearFolder = fullfile(root,'results','trajectory_methods','linear');
outputFolder = fullfile(root,'results','trajectory_methods','s_curve');
backupFolder = fullfile(root,'results','trajectory_dashboard','backups');

if ~isfolder(outputFolder), mkdir(outputFolder); end
if ~isfolder(backupFolder), mkdir(backupFolder); end

sourceFile = fullfile(linearFolder,'linear_waypoints.mat');
outputFile = fullfile(outputFolder,'s_curve_waypoints.mat');

if ~isfile(sourceFile)
    error('Khong tim thay Linear waypoint: %s',sourceFile);
end

if isfile(outputFile)
    backupFile = fullfile(backupFolder, ...
        ['s_curve_before_rebuild_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
    copyfile(outputFile,backupFile);
    fprintf('Da sao luu S-Curve cu: %s\n',backupFile);
end

D = load(sourceFile);
sourceNames = {'linear_home2in','linear_in2out','linear_out2home'};
names = {'home2in','in2out','out2home'};
results = struct();
Ts = zeros(3,1);

fprintf('\n===== GENERATE S-CURVE 7 PHASE =====\n');

for k = 1:3
    source = D.(sourceNames{k});
    validateSource(source,k);

    t = source.t(:)';
    t = t-t(1);
    T = t(end);
    q0 = source.q(1:4,1);
    qf = source.q(1:4,end);
    delta = qf-q0;

    [s,sd,sdd,sddd,profile] = buildSCurve(t,T);

    S = source;
    S.t = t;
    S.q(1:4,:) = q0+delta*s;
    S.qd = delta*sd;
    S.qdd = delta*sdd;
    S.qddd = delta*sddd;
    S.q(5,:) = source.q(5,:);
    S.method = 'S-Curve 7 Phase Jerk Limited';
    S.sampleTime = median(diff(t));
    S.originalWaypoints = source.q(:,[1 end]);
    S.originalTime = source.t;
    S.profile = profile;

    results.(names{k}) = S;
    Ts(k) = S.sampleTime;

    checkSegment(S,source,k);

    fprintf('\nSegment %d: %s\n',k,names{k});
    fprintf('Duration: %.4f s\n',T);
    fprintf('Samples: %d\n',numel(t));
    fprintf('Sample time: %.8f s\n',Ts(k));
    fprintf('Max normalized jerk: %.8f\n',max(abs(sddd)));
end

s_curve_home2in = results.home2in;
s_curve_in2out = results.in2out;
s_curve_out2home = results.out2home;
sCurveResults = results;

save(outputFile,'sCurveResults','s_curve_home2in', ...
    's_curve_in2out','s_curve_out2home','Ts','names','-v7.3');

V = load(outputFile);
required = {'s_curve_home2in','s_curve_in2out','s_curve_out2home'};
for k = 1:numel(required)
    if ~isfield(V,required{k})
        error('File thieu bien %s.',required{k});
    end
end

info = dir(outputFile);
fprintf('\nDa kiem tra file: %s\n',outputFile);
fprintf('Dung luong: %.3f MB\n',info.bytes/1024^2);
fprintf('\nTAO S-CURVE THANH CONG.\n');
end

function [s,sd,sdd,sddd,profile] = buildSCurve(t,T)

tj = 0.1*T;
ta = 0.2*T;
tv = 0.2*T;
durations = [tj;ta;tj;tv;tj;ta;tj];
jerks = [1;0;-1;0;-1;0;1];

if abs(sum(durations)-T) > 1e-10
    error('Tong thoi gian 7 pha khong bang T.');
end

phaseStart = zeros(7,1);
phaseEnd = zeros(7,1);
xStart = zeros(7,1);
vStart = zeros(7,1);
aStart = zeros(7,1);
x = 0; v = 0; a = 0; cursor = 0;

for k = 1:7
    d = durations(k);
    j = jerks(k);
    phaseStart(k) = cursor;
    phaseEnd(k) = cursor+d;
    xStart(k) = x;
    vStart(k) = v;
    aStart(k) = a;
    x = x+v*d+0.5*a*d^2+j*d^3/6;
    v = v+a*d+0.5*j*d^2;
    a = a+j*d;
    cursor = cursor+d;
end

scale = 1/x;
s = zeros(size(t));
sd = zeros(size(t));
sdd = zeros(size(t));
sddd = zeros(size(t));

for n = 1:numel(t)
    phase = find(t(n) <= phaseEnd+1e-12,1,'first');
    if isempty(phase), phase = 7; end
    tau = max(0,min(t(n)-phaseStart(phase),durations(phase)));
    j = jerks(phase);
    s(n) = (xStart(phase)+vStart(phase)*tau+ ...
        0.5*aStart(phase)*tau^2+j*tau^3/6)*scale;
    sd(n) = (vStart(phase)+aStart(phase)*tau+0.5*j*tau^2)*scale;
    sdd(n) = (aStart(phase)+j*tau)*scale;
    sddd(n) = j*scale;
end

s(1) = 0;
s(end) = 1;
sd([1 end]) = 0;
sdd([1 end]) = 0;

profile = struct('type','Seven Phase Jerk Limited', ...
    'phaseDurations',durations,'phaseJerks',jerks, ...
    'normalizationScale',scale,'totalDuration',T, ...
    'phaseBoundaries',phaseEnd);
end

function validateSource(S,k)
required = {'t','q','qd','qdd','qddd'};
for n = 1:numel(required)
    if ~isfield(S,required{n})
        error('Source segment %d thieu truong %s.',k,required{n});
    end
end
N = numel(S.t);
if N < 2 || size(S.q,1) ~= 5 || size(S.q,2) ~= N
    error('Kich thuoc q source segment %d khong hop le.',k);
end
fields = {'qd','qdd','qddd'};
for n = 1:numel(fields)
    A = S.(fields{n});
    if size(A,1) ~= 4 || size(A,2) ~= N
        error('Kich thuoc %s source segment %d khong hop le.',fields{n},k);
    end
end
if any(~isfinite(S.t(:))) || any(diff(S.t(:)) <= 0) || any(~isfinite(S.q(:)))
    error('Du lieu Linear segment %d khong hop le.',k);
end
end

function checkSegment(A,source,k)
N = numel(A.t);
if size(A.q,1) ~= 5 || size(A.q,2) ~= N || ...
        size(A.qd,1) ~= 4 || size(A.qd,2) ~= N || ...
        size(A.qdd,1) ~= 4 || size(A.qdd,2) ~= N || ...
        size(A.qddd,1) ~= 4 || size(A.qddd,2) ~= N
    error('S-Curve segment %d sai kich thuoc.',k);
end
allData = [A.t(:);A.q(:);A.qd(:);A.qdd(:);A.qddd(:)];
if any(~isfinite(allData)) || any(diff(A.t) <= 0)
    error('S-Curve segment %d co du lieu khong hop le.',k);
end
positionError = max(abs(A.q(1:4,[1 end])-source.q(1:4,[1 end])),[],'all');
gripperError = max(abs(A.q(5,:)-source.q(5,:)));
if positionError > 1e-10 || gripperError > 1e-12
    error('S-Curve segment %d sai waypoint hoac Gripper.',k);
end
if max(abs(A.qd(:,[1 end])),[],'all') > 1e-9 || ...
        max(abs(A.qdd(:,[1 end])),[],'all') > 1e-9
    error('S-Curve segment %d sai dieu kien dau/cuoi.',k);
end
fprintf('Position error: %.12g\n',positionError);
fprintf('Gripper error: %.12g\n',gripperError);
fprintf('All finite: 1\n');
fprintf('Endpoint velocity/acceleration: PASS\n');
end
