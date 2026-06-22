% setup_project.m
% Run once to create the full project folder structure.
% After running, delete or keep for reference — never needs to run again.

root = fileparts(mfilename('fullpath'));

folders = {
    'data/raw/authentic'
    'data/raw/spoofed'
    'data/rinex/observation'
    'data/rinex/navigation'
    'data/reference'
    'stage0_osnma/keys'
    'stage1_detection'
    'stage2_identification'
    'stage3_exclusion'
    'stage4_recovery'
    'utils'
    'results/figures'
    'results/logs'
    'results/pvt'
    'results/simulated_scenarios'
    'tests'
};

fprintf('Creating project structure...\n');
for i = 1:length(folders)
    full_path = fullfile(root, folders{i});
    if ~exist(full_path, 'dir')
        mkdir(full_path);
        fprintf('  created: %s\n', folders{i});
    else
        fprintf('  already exists: %s\n', folders{i});
    end
end
fprintf('\nDone. Project structure ready.\n');