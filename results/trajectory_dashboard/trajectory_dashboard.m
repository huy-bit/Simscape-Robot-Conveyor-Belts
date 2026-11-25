
function trajectory_dashboard

clc;

root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';
mdl = 'youBot_Arm_recovery';

modelFile = fullfile(root,'results','models', ...
    'youBot_Arm_recovery.slx');

backupFile = fullfile(root,'results','trajectory_dashboard', ...
    'backups','lookup_config_before_linear.mat');

methods = {
    'Linear'
    'Trapezoidal'
    'LSPB'
    'Cubic Polynomial'
    'Quintic Polynomial'
    'Septic Polynomial'
    'Sinusoidal'
    'Cycloidal'
    'S-Curve'
    'Bezier'
    'Minimum Jerk'
    'Minimum Snap'
};

chartTitles = {
    'Joint Position'
    'Joint Velocity'
    'Joint Acceleration'
    'Joint Jerk'
};

jointNames = {'Pivot','Bicep','Forearm','Wrist'};

segmentNames = {
    'Home to BeltIn'
    'BeltIn to BeltOut'
    'BeltOut to Home'
};

jointBlocks = {'Pivot','Bicep','Forearm','Wrist'};

methodStatus = repmat({'Chua mo phong'},12,1);

running = false;

%% PATHS

addpath(fullfile(root,'Scripts_Data'));

if ~isfile(modelFile)
    error('Khong tim thay model: %s',modelFile);
end

if ~isfile(backupFile)
    error('Khong tim thay backup Lookup Table: %s',backupFile);
end

%% LOAD NORMALIZED KINEMATICS

kinFile = fullfile(root,'results','trajectory_dashboard', ...
    'normalized_metrics','initial_three_methods_kinematics.mat');

if ~isfile(kinFile)
    error('Khong tim thay file kinematics: %s',kinFile);
end

K = load(kinFile,'kinematics');

if ~isfield(K,'kinematics')
    error('File khong co bien kinematics.');
end

kinematics = K.kinematics; %#ok<NASGU>

%% LOAD ORIGINAL LOOKUP CONFIGURATION

B = load(backupFile,'lookupBackup');
lookupBackup = B.lookupBackup;

if numel(lookupBackup) ~= 12
    error('Backup khong co dung 12 Lookup Table.');
end

%% LOAD MODEL

if ~bdIsLoaded(mdl)
    load_system(modelFile);
end

%% CREATE GUI

fig = uifigure( ...
    'Name','Robot Trajectory Dashboard', ...
    'Position',[80 60 1450 850]);

main = uigridlayout(fig,[1 2]);
main.ColumnWidth = {310,'1x'};
main.Padding = [10 10 10 10];

%% LEFT PANEL

left = uipanel(main,'Title','12 TRAJECTORY METHODS');
left.Layout.Column = 1;

lg = uigridlayout(left,[14 1]);
lg.RowHeight = [repmat({42},1,12),{45},{45}];
lg.Padding = [8 8 8 8];

buttons = gobjects(12,1);

for k = 1:12
    buttons(k) = uibutton(lg,'push', ...
        'Text',sprintf('%02d. %s',k,methods{k}), ...
        'FontSize',12, ...
        'ButtonPushedFcn',@(~,~)selectMethod(k));
end

uibutton(lg,'push', ...
    'Text','CHAY 12 PHUONG PHAP DA CO', ...
    'FontWeight','bold', ...
    'BackgroundColor',[0.1 0.4 0.7], ...
    'FontColor',[1 1 1], ...
    'ButtonPushedFcn',@(~,~)runAvailableMethods());

uibutton(lg,'push', ...
    'Text','DONG DASHBOARD', ...
    'ButtonPushedFcn',@(~,~)delete(fig));

%% RIGHT PANEL

right = uipanel(main,'Title','SIMULATION RESULTS');
right.Layout.Column = 2;

rg = uigridlayout(right,[4 2]);
rg.RowHeight = {45,'1x','1x',230};
rg.ColumnWidth = {'1x','1x'};
rg.Padding = [10 10 10 10];

selectedLabel = uilabel(rg, ...
    'Text','Chua chon phuong phap', ...
    'FontSize',17, ...
    'FontWeight','bold');

selectedLabel.Layout.Row = 1;
selectedLabel.Layout.Column = [1 2];

ax = gobjects(4,1);

for k = 1:4
    ax(k) = uiaxes(rg);

    ax(k).Layout.Row = ceil(k/2)+1;
    ax(k).Layout.Column = mod(k-1,2)+1;

    title(ax(k),chartTitles{k});
    xlabel(ax(k),'Time (s)');
    grid(ax(k),'on');
end

tbl = uitable(rg);
tbl.Layout.Row = 4;
tbl.Layout.Column = [1 2];
tbl.ColumnName = {'Metric','Value'};
tbl.ColumnWidth = {250,'auto'};

resetTable('Chua mo phong');

%% SELECT METHOD AND RUN SIMULATION

function selectMethod(k)

    if running
        return;
    end

    if ~ismember(k,1:12)
        methodStatus{k} = 'Chua co du lieu quy dao';

        selectedLabel.Text = ...
            ['Phuong phap: ' methods{k}];

        resetTable('Chua co du lieu quy dao');
        clearCharts();

        fprintf('Chua co du lieu cho %s.\n',methods{k});
        return;
    end

    running = true;
    setButtonsEnabled(false);

    selectedLabel.Text = ...
        ['Dang chay: ' methods{k}];

    resetTable('Dang mo phong');
    clearCharts();

    drawnow;

    try
        %% LOAD METHOD DATA

        [segments,segmentVariables,trajectory,folder] = ...
            loadMethodData(k);

        validateSegments(segments);
        validateTrajectory(trajectory);

        %% ASSIGN VARIABLES TO BASE WORKSPACE

        for s = 1:3
            assignin('base',segmentVariables{s},segments{s});
        end

        %% UPDATE 12 LOOKUP TABLES

        if ismember(k,[1 2 3])
            interpolation = 'Linear point-slope';
        else
            interpolation = 'Cubic spline';
        end

        updateLookupTables(segmentVariables,interpolation);

        %% CONFIGURE SIMULINK

        youBot_configureActuation(mdl,'Motion');
        youBot_configureLoad(mdl,'Damper');
        youBot_configureInput(mdl,'Control');

        set_param(mdl,'SimulationMode','normal');
        set_param(mdl,'StopTime','20');

        %% UPDATE AND SAVE MODEL

        set_param(mdl,'SimulationCommand','update');

        save_system(mdl,modelFile);

        %% RUN ACTUAL SIMULATION

        selectedLabel.Text = ...
            ['Dang mo phong robot: ' methods{k}];

        drawnow;

        fprintf('\n====================================\n');
        fprintf('RUNNING METHOD: %s\n',methods{k});
        fprintf('====================================\n');

        simOut = sim(mdl,'ReturnWorkspaceOutputs','on');

        %% GET FRESH SIMULATION DATA

        if ~isa(simOut,'Simulink.SimulationOutput')
            error('Simulink khong tra ve SimulationOutput.');
        end

        logsDataset = simOut.get('logsout_youBot_Arm_recovery');

        validateDataset(logsDataset);

        %% SAVE RAW RESULTS

        simulationFolder = fullfile(folder,'simulation');

        if ~isfolder(simulationFolder)
            mkdir(simulationFolder);
        end

        switch k
            case 1
                rawName = 'linear_simulation_raw.mat';

            case 2
                rawName = 'trapezoidal_simulation_raw.mat';

            case 3
                rawName = 'lspb_simulation_raw.mat';

            case 4
                rawName = 'cubic_polynomial_simulation_raw.mat';

            case 5
                rawName = 'quintic_polynomial_simulation_raw.mat';

            case 6
                rawName = 'septic_polynomial_simulation_raw.mat';

            case 7
                rawName = 'sinusoidal_simulation_raw.mat';

            case 8
                rawName = 'cycloidal_simulation_raw.mat';

            case 9
                rawName = 's_curve_simulation_raw.mat';

            case 10
                rawName = 'bezier_simulation_raw.mat';

            case 11
                rawName = 'minimum_jerk_simulation_raw.mat';

            case 12
                rawName = 'minimum_snap_simulation_raw.mat';

            otherwise
                error('Chua co ten file ket qua.');
        end

        rawFile = fullfile(simulationFolder,rawName);

        save(rawFile, ...
            'simOut', ...
            'logsDataset', ...
            'trajectory', ...
            '-v7.3');

        %% DRAW RESULTS FROM THIS RUN

        drawResults(trajectory,logsDataset,methods{k});

        methodStatus{k} = 'Da mo phong thanh cong';

        buttons(k).BackgroundColor = [0.70 0.90 0.70];

        selectedLabel.Text = ...
            ['Da mo phong: ' methods{k}];

        fprintf('\nMO PHONG THANH CONG: %s\n',methods{k});
        fprintf('Raw result: %s\n',rawFile);

    catch ME

        methodStatus{k} = 'Loi mo phong';

        buttons(k).BackgroundColor = [0.95 0.65 0.65];

        selectedLabel.Text = ...
            ['Mo phong that bai: ' methods{k}];

        resetTable('Loi mo phong');
        clearCharts();

        fprintf(2,'\nLOI MO PHONG %s\n',methods{k});
        fprintf(2,'%s\n', ...
            getReport(ME,'extended','hyperlinks','off'));

        uialert(fig, ...
            getReport(ME,'basic','hyperlinks','off'), ...
            ['Loi mo phong ' methods{k}]);

    end

    running = false;
    setButtonsEnabled(true);
    drawnow;

end

%% LOAD WAYPOINT DATA

function [segments,variables,T,folder] = loadMethodData(k)

    switch k

        case 1

            folder = fullfile(root,'results', ...
                'trajectory_methods','linear');

            file = fullfile(folder,'linear_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Linear waypoint MAT.');
            end

            D = load(file);

            segments = {
                D.linear_home2in
                D.linear_in2out
                D.linear_out2home
            };

            variables = {
                'linear_home2in'
                'linear_in2out'
                'linear_out2home'
            };

        case 2

            folder = fullfile(root,'results', ...
                'trajectory_methods','trapezoidal');

            file = fullfile(folder,'trapezoidal_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Trapezoidal waypoint MAT.');
            end

            D = load(file);

            segments = {
                D.trapezoidal_home2in
                D.trapezoidal_in2out
                D.trapezoidal_out2home
            };

            variables = {
                'trapezoidal_home2in'
                'trapezoidal_in2out'
                'trapezoidal_out2home'
            };

        case 3

            folder = fullfile(root,'results', ...
                'trajectory_methods','lspb');

            file = fullfile(folder,'lspb_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay LSPB waypoint MAT.');
            end

            D = load(file);

            segments = {
                D.lspb_home2in
                D.lspb_in2out
                D.lspb_out2home
            };

            variables = {
                'lspb_home2in'
                'lspb_in2out'
                'lspb_out2home'
            };

        case 4

            folder = fullfile(root,'results', ...
                'trajectory_methods','cubic_polynomial');

            file = fullfile(folder,'cubic_polynomial_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Cubic waypoint MAT.');
            end

            D = load(file);

            segments = {
                D.cubic_home2in
                D.cubic_in2out
                D.cubic_out2home
            };

            variables = {
                'cubic_home2in'
                'cubic_in2out'
                'cubic_out2home'
            };

        case 5

            folder = fullfile(root,'results', ...
                'trajectory_methods','quintic_polynomial');

            file = fullfile(folder,'quintic_polynomial_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Quintic waypoint MAT.');
            end

            D = load(file);

            Q = D.quinticResults;

            segments = {
                Q.home2in
                Q.in2out
                Q.out2home
            };

            variables = {
                'quintic_home2in'
                'quintic_in2out'
                'quintic_out2home'
            };

        case 6

            folder = fullfile(root,'results', ...
                'trajectory_methods','septic_polynomial');

            file = fullfile(folder,'septic_polynomial_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Septic waypoint MAT: %s',file);
            end

            D = load(file);

            segments = {
                D.septic_home2in
                D.septic_in2out
                D.septic_out2home
            };

            variables = {
                'septic_home2in'
                'septic_in2out'
                'septic_out2home'
            };

        case 7

            folder = fullfile(root,'results', ...
                'trajectory_methods','sinusoidal');

            file = fullfile(folder,'sinusoidal_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Sinusoidal waypoint MAT: %s',file);
            end

            D = load(file);

            requiredVariables = {
                'sinusoidal_home2in'
                'sinusoidal_in2out'
                'sinusoidal_out2home'
            };

            for n = 1:numel(requiredVariables)
                if ~isfield(D,requiredVariables{n})
                    error('File Sinusoidal thieu bien: %s', ...
                        requiredVariables{n});
                end
            end

            segments = {
                D.sinusoidal_home2in
                D.sinusoidal_in2out
                D.sinusoidal_out2home
            };

            variables = {
                'sinusoidal_home2in'
                'sinusoidal_in2out'
                'sinusoidal_out2home'
            };

        case 8

            folder = fullfile(root,'results', ...
                'trajectory_methods','cycloidal');

            file = fullfile(folder,'cycloidal_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Cycloidal waypoint MAT: %s',file);
            end

            D = load(file);

            requiredVariables = {
                'cycloidal_home2in'
                'cycloidal_in2out'
                'cycloidal_out2home'
            };

            for n = 1:numel(requiredVariables)
                if ~isfield(D,requiredVariables{n})
                    error('File Cycloidal thieu bien: %s', ...
                        requiredVariables{n});
                end
            end

            segments = {
                D.cycloidal_home2in
                D.cycloidal_in2out
                D.cycloidal_out2home
            };

            variables = {
                'cycloidal_home2in'
                'cycloidal_in2out'
                'cycloidal_out2home'
            };

        case 9

            folder = fullfile(root,'results', ...
                'trajectory_methods','s_curve');

            file = fullfile(folder,'s_curve_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay S-Curve waypoint MAT: %s',file);
            end

            D = load(file);

            requiredVariables = {
                's_curve_home2in'
                's_curve_in2out'
                's_curve_out2home'
            };

            for n = 1:numel(requiredVariables)
                if ~isfield(D,requiredVariables{n})
                    error('File S-Curve thieu bien: %s', ...
                        requiredVariables{n});
                end
            end

            segments = {
                D.s_curve_home2in
                D.s_curve_in2out
                D.s_curve_out2home
            };

            variables = {
                's_curve_home2in'
                's_curve_in2out'
                's_curve_out2home'
            };

        case 10

            folder = fullfile(root,'results', ...
                'trajectory_methods','bezier');

            file = fullfile(folder,'bezier_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Bezier waypoint MAT: %s',file);
            end

            D = load(file);

            requiredVariables = {
                'bezier_home2in'
                'bezier_in2out'
                'bezier_out2home'
            };

            for n = 1:numel(requiredVariables)
                if ~isfield(D,requiredVariables{n})
                    error('File Bezier thieu bien: %s', ...
                        requiredVariables{n});
                end
            end

            segments = {
                D.bezier_home2in
                D.bezier_in2out
                D.bezier_out2home
            };

            variables = {
                'bezier_home2in'
                'bezier_in2out'
                'bezier_out2home'
            };

                case 11

            folder = fullfile(root,'results', ...
                'trajectory_methods','minimum_jerk');

            file = fullfile(folder,'minimum_jerk_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Minimum Jerk waypoint MAT: %s',file);
            end

            D = load(file);

            requiredVariables = {
                'minimum_jerk_home2in'
                'minimum_jerk_in2out'
                'minimum_jerk_out2home'
            };

            for n = 1:numel(requiredVariables)
                if ~isfield(D,requiredVariables{n})
                    error('File Minimum Jerk thieu bien: %s', ...
                        requiredVariables{n});
                end
            end

            segments = {
                D.minimum_jerk_home2in
                D.minimum_jerk_in2out
                D.minimum_jerk_out2home
            };

            variables = {
                'minimum_jerk_home2in'
                'minimum_jerk_in2out'
                'minimum_jerk_out2home'
            };

        case 12

            folder = fullfile(root,'results', ...
                'trajectory_methods','minimum_snap');

            file = fullfile(folder,'minimum_snap_waypoints.mat');

            if ~isfile(file)
                error('Khong tim thay Minimum Snap waypoint MAT: %s',file);
            end

            D = load(file);

            requiredVariables = {
                'minimum_snap_home2in'
                'minimum_snap_in2out'
                'minimum_snap_out2home'
            };

            for n = 1:numel(requiredVariables)
                if ~isfield(D,requiredVariables{n})
                    error('File Minimum Snap thieu bien: %s', ...
                        requiredVariables{n});
                end
            end

            segments = {
                D.minimum_snap_home2in
                D.minimum_snap_in2out
                D.minimum_snap_out2home
            };

            variables = {
                'minimum_snap_home2in'
                'minimum_snap_in2out'
                'minimum_snap_out2home'
            };

otherwise
            error('Phuong phap chua co du lieu.');
    end

    T = buildTrajectory(segments);

end

%% BUILD CONTINUOUS TRAJECTORY FOR DASHBOARD

function T = buildTrajectory(segments)

    T.t = [];
    T.q = [];
    T.qd = [];
    T.qdd = [];
    T.qddd = [];

    timeOffset = 0;

    for s = 1:3

        S = segments{s};

        t = S.t(:)';

        if s > 1
            first = 2;
        else
            first = 1;
        end

        globalTime = t(first:end) + timeOffset;

        T.t = [T.t globalTime];

        T.q = [T.q S.q(1:4,first:end)];
        T.qd = [T.qd S.qd(1:4,first:end)];
        T.qdd = [T.qdd S.qdd(1:4,first:end)];
        T.qddd = [T.qddd S.qddd(1:4,first:end)];

        timeOffset = timeOffset + t(end);

    end

    T.jointNames = jointNames;
    T.sampleTime = median(diff(T.t));

end

%% VALIDATE SEGMENTS

function validateSegments(segments)

    if numel(segments) ~= 3
        error('Can co dung 3 phan doan quy dao.');
    end

    required = {'t','q','qd','qdd','qddd'};

    for s = 1:3

        S = segments{s};

        for n = 1:numel(required)
            if ~isfield(S,required{n})
                error('Segment %d thieu truong %s.', ...
                    s,required{n});
            end
        end

        N = numel(S.t);

        if N < 2
            error('Segment %d khong du mau.',s);
        end

        if size(S.q,1) ~= 5 || size(S.q,2) ~= N
            error('Kich thuoc q cua segment %d sai.',s);
        end

        fields = {'qd','qdd','qddd'};

        for n = 1:numel(fields)

            A = S.(fields{n});

            if size(A,1) < 4 || size(A,2) ~= N
                error('Kich thuoc %s segment %d sai.', ...
                    fields{n},s);
            end

            if any(~isfinite(A(:)))
                error('Du lieu %s co NaN hoac Inf.',fields{n});
            end

        end

        if any(~isfinite(S.t(:))) || any(diff(S.t(:)) <= 0)
            error('Thoi gian segment %d khong tang.',s);
        end

        if any(~isfinite(S.q(:)))
            error('Du lieu q segment %d co NaN hoac Inf.',s);
        end

    end

end

%% UPDATE 12 LOOKUP TABLES

function updateLookupTables(variables,interpolation)

    for s = 1:3

        for j = 1:4

            block = [mdl ...
                '/Input/Control/Joint Commands/' ...
                segmentNames{s} '/' jointBlocks{j}];

            if getSimulinkBlockHandle(block) == -1
                error('Khong tim thay Lookup Table: %s',block);
            end

            tableExpression = sprintf('%s.q(%d,:)', ...
                variables{s},j);

            timeExpression = sprintf('%s.t',variables{s});

            set_param(block, ...
                'Table',tableExpression, ...
                'BreakpointsForDimension1',timeExpression, ...
                'InterpMethod',interpolation, ...
                'ExtrapMethod','Clip');

        end

    end

    fprintf('Da cap nhat 12 Lookup Table.\n');

end

%% VALIDATE TRAJECTORY

function validateTrajectory(T)

    required = {'t','q','qd','qdd','qddd'};

    for n = 1:numel(required)
        if ~isfield(T,required{n})
            error('Thieu truong trajectory: %s',required{n});
        end
    end

    N = numel(T.t);

    if N < 2
        error('Trajectory khong du mau.');
    end

    fields = {'q','qd','qdd','qddd'};

    for n = 1:numel(fields)

        A = T.(fields{n});

        if size(A,1) ~= 4 || size(A,2) ~= N
            error('Kich thuoc %s khong hop le.',fields{n});
        end

        if any(~isfinite(A(:)))
            error('%s co NaN hoac Inf.',fields{n});
        end

    end

    if any(~isfinite(T.t(:))) || any(diff(T.t(:)) <= 0)
        error('Vector thoi gian khong tang.');
    end

end

%% VALIDATE SIMULATION DATASET

function validateDataset(D)

    if ~isa(D,'Simulink.SimulationData.Dataset')
        error('logsout khong phai Dataset.');
    end

    required = {
        'Torque_Pivot'
        'Torque_Bicep'
        'Torque_Forearm'
        'Torque_Wrist'
        'motor_currents'
        'electrical_cost'
        'torque_cost'
    };

    for n = 1:numel(required)
        getSignal(D,required{n});
    end

end

%% DRAW CHARTS AND METRICS

function drawResults(T,D,methodName)

    signals = {
        T.q
        T.qd
        T.qdd
        T.qddd
    };

    signalNames = {
        'Position (deg)'
        'Velocity (deg/s)'
        'Acceleration (deg/s^2)'
        'Jerk (deg/s^3)'
    };

    for p = 1:4

        cla(ax(p));
        hold(ax(p),'on');

        for j = 1:4

            plot(ax(p),T.t,signals{p}(j,:), ...
                'DisplayName',jointNames{j}, ...
                'LineWidth',1.0);

        end

        title(ax(p),chartTitles{p});
        xlabel(ax(p),'Time (s)');
        ylabel(ax(p),signalNames{p});

        legend(ax(p),'Location','best');
        grid(ax(p),'on');

        hold(ax(p),'off');

    end

    torqueNames = {
        'Torque_Pivot'
        'Torque_Bicep'
        'Torque_Forearm'
        'Torque_Wrist'
    };

    torqueMax = zeros(4,1);

    for j = 1:4
        ts = getSignal(D,torqueNames{j});
        torqueMax(j) = max(abs(ts.Data(:)));
    end

    currentTS = getSignal(D,'motor_currents');
    maxCurrent = max(abs(currentTS.Data(:)));

    electricalTS = getSignal(D,'electrical_cost');
    torqueCostTS = getSignal(D,'torque_cost');

    electricalCost = electricalTS.Data(end);
    torqueCost = torqueCostTS.Data(end);

    tbl.Data = {
        'Status','Da mo phong thanh cong'
        'Max Pivot Torque (Nm)',sprintf('%.5f',torqueMax(1))
        'Max Bicep Torque (Nm)',sprintf('%.5f',torqueMax(2))
        'Max Forearm Torque (Nm)',sprintf('%.5f',torqueMax(3))
        'Max Wrist Torque (Nm)',sprintf('%.5f',torqueMax(4))
        'Max Motor Current (A)',sprintf('%.5f',maxCurrent)
        'Electrical Cost (J)',sprintf('%.5f',electricalCost)
        'Torque Cost (Nm^2.s)',sprintf('%.5f',torqueCost)
    };

    selectedLabel.Text = ['Phuong phap: ' methodName];

    drawnow;

    fprintf('\n===== FRESH SIMULATION METRICS =====\n');
    fprintf('Method: %s\n',methodName);
    fprintf('Pivot max torque: %.6f Nm\n',torqueMax(1));
    fprintf('Bicep max torque: %.6f Nm\n',torqueMax(2));
    fprintf('Forearm max torque: %.6f Nm\n',torqueMax(3));
    fprintf('Wrist max torque: %.6f Nm\n',torqueMax(4));
    fprintf('Max motor current: %.6f A\n',maxCurrent);
    fprintf('Electrical cost: %.6f J\n',electricalCost);
    fprintf('Torque cost: %.6f Nm^2.s\n',torqueCost);

end

%% GET DATASET SIGNAL

function ts = getSignal(D,name)

    ts = [];

    for n = 1:D.numElements

        element = D.getElement(n);

        if strcmp(element.Name,name)
            ts = element.Values;
            break;
        end

    end

    if isempty(ts)
        error('Khong tim thay tin hieu: %s',name);
    end

    if ~isa(ts,'timeseries')
        error('Tin hieu %s khong phai timeseries.',name);
    end

    if isempty(ts.Data) || any(~isfinite(ts.Data(:)))
        error('Tin hieu %s rong hoac co NaN/Inf.',name);
    end

end

%% RUN EIGHT AVAILABLE METHODS

function runAvailableMethods()

    if running
        return;
    end

    for idx = 1:12

        if ~isvalid(fig)
            return;
        end

        selectMethod(idx);

        if ~strcmp(methodStatus{idx},'Da mo phong thanh cong')
            break;
        end

    end

end

%% BUTTON ENABLE

function setButtonsEnabled(state)

    for n = 1:numel(buttons)
        buttons(n).Enable = matlab.lang.OnOffSwitchState(state);
    end

end

%% RESET TABLE

function resetTable(status)

    tbl.Data = {
        'Status',status
        'Max Pivot Torque (Nm)','-'
        'Max Bicep Torque (Nm)','-'
        'Max Forearm Torque (Nm)','-'
        'Max Wrist Torque (Nm)','-'
        'Max Motor Current (A)','-'
        'Electrical Cost (J)','-'
        'Torque Cost (Nm^2.s)','-'
    };

end

%% CLEAR CHARTS

function clearCharts()

    for j = 1:4
        cla(ax(j));
        title(ax(j),chartTitles{j});
        xlabel(ax(j),'Time (s)');
        grid(ax(j),'on');
    end

end

end
