
clc;
clear;

%% 1. DUONG DAN

projectDir = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';
resultDir = fullfile(projectDir,'results');

excludedDir = fullfile(projectDir,'excluded_research_files');

if ~isfolder(resultDir)
    error('Khong tim thay thu muc results.');
end

%% 2. TAO THU MUC

folderNames = {
    'models'
    'simulation_data'
    'trajectory_methods'
    'analysis'
    'figures'
    'reports'
    'scripts'
};

for k = 1:numel(folderNames)
    p = fullfile(resultDir,folderNames{k});

    if ~isfolder(p)
        mkdir(p);
    end
end

if ~isfolder(excludedDir)
    mkdir(excludedDir);
end

%% 3. LAY FILE O THU MUC RESULTS GOC

files = dir(resultDir);
files = files(~[files.isdir]);

moved = 0;
excluded = 0;
skipped = 0;
errors = 0;

fprintf('\n===== SAP XEP DU LIEU SIMSCAPE =====\n');

%% 4. PHAN LOAI FILE

for k = 1:numel(files)

    name = files(k).name;
    lowerName = lower(name);

    source = fullfile(resultDir,name);

    [~,~,ext] = fileparts(lowerName);

    destinationFolder = '';

    %% FILE KHONG LIEN QUAN

    isExcluded = ...
        contains(lowerName,'factory_') || ...
        contains(lowerName,'gripper_') || ...
        contains(lowerName,'product_and_trajectory');

    if isExcluded

        destinationFolder = excludedDir;
        excluded = excluded + 1;

    %% FILE HINH ANH

    elseif any(strcmp(ext,{'.png','.fig','.jpg','.jpeg'}))

        destinationFolder = fullfile(resultDir,'figures');

    %% FILE BAO CAO

    elseif any(strcmp(ext,{'.pdf','.docx','.txt'})) && ...
            (contains(lowerName,'report') || ...
             contains(lowerName,'summary') || ...
             contains(lowerName,'manifest'))

        destinationFolder = fullfile(resultDir,'reports');

    %% FILE SCRIPT

    elseif contains(lowerName,'source') || ...
           startsWith(lowerName,'run_') || ...
           startsWith(lowerName,'update_')

        destinationFolder = fullfile(resultDir,'scripts');

    %% FILE QUY DAO

    elseif contains(lowerName,'trajectory') || ...
           contains(lowerName,'scurve') || ...
           contains(lowerName,'comparison_12_methods')

        destinationFolder = fullfile(resultDir,'trajectory_methods');

    %% FILE PHAN TICH

    elseif contains(lowerName,'cad_') || ...
           contains(lowerName,'dh_') || ...
           contains(lowerName,'ik_') || ...
           contains(lowerName,'force') || ...
           contains(lowerName,'torque') || ...
           contains(lowerName,'kinematic') || ...
           contains(lowerName,'endpoint') || ...
           contains(lowerName,'joint_response') || ...
           contains(lowerName,'worldframe') || ...
           contains(lowerName,'sixdof') || ...
           contains(lowerName,'integrity') || ...
           contains(lowerName,'input_check')

        destinationFolder = fullfile(resultDir,'analysis');

    %% DU LIEU MO PHONG

    elseif any(strcmp(ext,{'.mat','.csv'}))

        destinationFolder = fullfile(resultDir,'simulation_data');

    else
        skipped = skipped + 1;
        fprintf('[GIU LAI] %s\n',name);
        continue;
    end

    %% 5. DI CHUYEN FILE AN TOAN

    if ~isfolder(destinationFolder)
        mkdir(destinationFolder);
    end

    destination = fullfile(destinationFolder,name);

    if isfile(destination)
        skipped = skipped + 1;
        fprintf('[TRUNG FILE] %s\n',name);
        continue;
    end

    try
        [ok,msg] = movefile(source,destination);

        if ok
            moved = moved + 1;
            fprintf('[DA CHUYEN] %s\n',name);
        else
            errors = errors + 1;
            fprintf('[LOI] %s | %s\n',name,msg);
        end

    catch ME
        errors = errors + 1;
        fprintf('[LOI] %s | %s\n',name,ME.message);
    end
end

%% 6. CHUYEN THU MUC 12 PHUONG PHAP

methodSource = fullfile(resultDir,'simscape_12_methods');

methodTarget = fullfile( ...
    resultDir,'trajectory_methods','simscape_12_methods');

if isfolder(methodSource)

    if ~isfolder(methodTarget)

        [ok,msg] = movefile(methodSource,methodTarget);

        if ok
            fprintf('\n[DA CHUYEN] Thu muc simscape_12_methods\n');
        else
            fprintf('\n[LOI THU MUC] %s\n',msg);
            errors = errors + 1;
        end

    else
        fprintf('\n[GIU LAI] Thu muc dich da ton tai.\n');
    end
end

%% 7. KIEM TRA MO HINH

modelPath = fullfile(resultDir,'models','youBot_Arm.slx');

if isfile(modelPath)
    fprintf('\n[OK] Mo hinh youBot_Arm.slx\n');
else
    fprintf('\n[CHU Y] Chua tim thay mo hinh trong models.\n');
end

%% 8. THONG KE

fprintf('\n===== KET QUA SAP XEP =====\n');
fprintf('File da chuyen: %d\n',moved);
fprintf('File khong lien quan: %d\n',excluded);
fprintf('File bo qua: %d\n',skipped);
fprintf('Loi: %d\n',errors);

fprintf('\nThu muc ket qua:\n%s\n',resultDir);
fprintf('\nThu muc file khong lien quan:\n%s\n',excludedDir);

fprintf('\nHOAN TAT.\n');
