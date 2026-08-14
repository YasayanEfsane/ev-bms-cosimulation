%% Advanced autonomous EV powertrain and BMS co-simulation
% No Simulink or Simscape is used. All continuous states are integrated by
% one fixed-step RK4 solver. Set simulationPreset before running this file:
%   simulationPreset = "smoke";   % 0.20 s, architecture check
%   simulationPreset = "short";   % 10 s, engineering preview
%   simulationPreset = "full";    % 600 s, 60 million RK4 steps

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));

if ~exist('simulationPreset', 'var')
    simulationPreset = "full";
end

cfg = defaultEVConfig(simulationPreset);
results = runEVSimulation(cfg);
figures = plotSimulationResults(results, cfg); %#ok<NASGU>

if cfg.output.saveResults
    if ~exist(cfg.output.directory, 'dir')
        mkdir(cfg.output.directory);
    end
    save(fullfile(cfg.output.directory, 'ev_bms_results.mat'), ...
        'results', 'cfg', '-v7.3');
end

