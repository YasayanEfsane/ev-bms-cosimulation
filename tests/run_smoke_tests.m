function report = run_smoke_tests()
%RUN_SMOKE_TESTS Component invariants plus a short end-to-end RK4 execution.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));
cfg = defaultEVConfig("smoke");
cfg.sim.endTime = 0.02;
cfg.sim.showProgress = false;

motor = PMSMMotor(cfg.motor, cfg.inverter);
dqReference = [123.4; -87.6];
electricalAngle = 1.234;
dqRoundTrip = motor.abcToDq( ...
    motor.dqToAbc(dqReference, electricalAngle), electricalAngle);
assert(norm(dqReference-dqRoundTrip, inf) < 1e-10, ...
    'Clarke/Park matrix round-trip failed.');

battery = BatteryPack(cfg.battery, cfg.bms);
batteryState = battery.initialState();
temperatureC = cfg.thermal.initialCellTemperatureC * ...
    ones(cfg.battery.nSeries, 1);
[~, ~, soc] = battery.unpackState(batteryState);
parameter = battery.lookupParameters(soc, temperatureC);
assert(all(parameter.Voc > cfg.battery.minimumCellVoltageV) && ...
    all(parameter.Voc <= cfg.battery.maximumCellVoltageV), ...
    'OCV LUT interpolation left the configured voltage domain.');
assert(all(parameter.R0 > 0) && all(parameter.R1 > 0) && ...
    all(parameter.R2 > 0) && all(parameter.C1 > 0) && ...
    all(parameter.C2 > 0), 'Thevenin LUT returned a non-positive value.');

system = buildEVSystem(cfg);
initialState = system.initialState();
drive = generateRaceDriveCycle(cfg.drive);
finalDriveInput = sampleDriveCycle(drive, drive.duration);
assert(abs(finalDriveInput.targetSpeedMps-drive.speedMps(end)) < eps, ...
    'Drive-cycle endpoint interpolation failed.');
initialDerivative = system.derivative(0, initialState, drive);
assert(numel(initialState) == 393, ...
    'The default 96s system should have exactly 393 continuous states.');
assert(numel(initialDerivative) == numel(initialState) && ...
    all(isfinite(initialDerivative)), ...
    'Unified derivative has the wrong size or non-finite values.');

results = runEVSimulation(cfg);
assert(all(isfinite(results.finalState)), ...
    'End-to-end simulation produced a non-finite state.');
assert(all(diff(results.log.timeS) > 0), ...
    'Logged timestamps are not strictly increasing.');
assert(all(results.log.packVoltageV > 0), ...
    'Pack voltage became non-positive in the smoke test.');
assert(all(results.log.cellVoltageV(:) >= ...
    cfg.battery.minimumCellVoltageV-0.05), ...
    'A cell-group voltage violated the guarded lower domain.');

report.passed = true;
report.continuousStateCount = system.StateCount;
report.integrationSteps = results.meta.integrationSteps;
report.maximumTransformError = norm(dqReference-dqRoundTrip, inf);
report.minimumPackVoltageV = min(results.log.packVoltageV);
fprintf('All EV/BMS smoke tests passed (%d states, %d RK4 steps).\n', ...
    report.continuousStateCount, report.integrationSteps);
end
