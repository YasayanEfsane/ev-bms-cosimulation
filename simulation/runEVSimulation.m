function results = runEVSimulation(cfg)
%RUNEVSIMULATION Execute the only explicit integration loop in the project.

drive = generateRaceDriveCycle(cfg.drive);
system = buildEVSystem(cfg);
state = system.initialState();
step = cfg.sim.integrationStep;
numberOfSteps = round(cfg.sim.endTime/step);
logStride = round(cfg.sim.logStep/step);
bmsStride = round(cfg.sim.bmsStep/step);
numberOfLogs = floor(numberOfSteps/logStride)+1;
log = initializeSimulationLog(numberOfLogs, cfg.battery.nSeries);

system.updateBMS(0, state, drive);
initialObservation = system.observe(0, state, drive);
log = recordSimulationLog(log, 1, initialObservation);
logIndex = 1;
nextLogStep = logStride;
nextBmsStartStep = bmsStride+1;
progressStride = max(1, round(numberOfSteps*cfg.sim.progressFractions));
nextProgressStep = progressStride;
derivativeFunction = @(timeS, unifiedState) ...
    system.derivative(timeS, unifiedState, drive);

if cfg.sim.showProgress
    fprintf('RK4 simulation: %.3f s, dt = %.0f us, %d steps, %d states.\n', ...
        cfg.sim.endTime, 1e6*step, numberOfSteps, system.StateCount);
end
simulationTimer = tic;

% The project-wide explicit-loop constraint is intentionally concentrated
% here. Every component calculation inside the loop is vectorized.
for stepIndex = 1:numberOfSteps
    startTimeS = (stepIndex-1)*step;
    if stepIndex == nextBmsStartStep
        system.updateBMS(startTimeS, state, drive);
        nextBmsStartStep = nextBmsStartStep+bmsStride;
    end

    state = rk4Step(derivativeFunction, startTimeS, state, step);
    state = system.projectState(state);

    if stepIndex == nextLogStep
        logIndex = logIndex+1;
        observation = system.observe(stepIndex*step, state, drive);
        log = recordSimulationLog(log, logIndex, observation);
        nextLogStep = nextLogStep+logStride;
    end

    if cfg.sim.showProgress && stepIndex == nextProgressStep
        fprintf('  %5.1f %% complete (t = %.2f s).\n', ...
            100*stepIndex/numberOfSteps, stepIndex*step);
        nextProgressStep = min(numberOfSteps, ...
            nextProgressStep+progressStride);
    end
end

elapsedTimeS = toc(simulationTimer);
results.log = trimSimulationLog(log, logIndex);
results.finalState = state;
results.stateLayout = system.Layout;
results.stateLayoutTable = system.stateLayoutTable();
results.driveCycle = drive;
results.config = cfg;
results.meta.integrationSteps = numberOfSteps;
results.meta.continuousStateCount = system.StateCount;
results.meta.elapsedWallTimeS = elapsedTimeS;
results.meta.simulatedSecondsPerWallSecond = cfg.sim.endTime/max(elapsedTimeS, eps);
results.meta.bmsFinalStatus = system.Battery.balancingStatus();

if cfg.sim.showProgress
    fprintf('Completed in %.1f s wall time (%.3f simulated s/wall s).\n', ...
        elapsedTimeS, results.meta.simulatedSecondsPerWallSecond);
end
end
