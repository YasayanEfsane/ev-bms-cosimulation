function figures = plotSimulationResults(results, cfg)
%PLOTSIMULATIONRESULTS Generate the required speed, FOC, BMS and thermal plots.

log = results.log;
timeS = log.timeS;

figures.speed = figure('Name', 'Speed tracking', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile;
plot(timeS, log.targetSpeedKph, 'k--', 'LineWidth', 1.2);
hold on;
plot(timeS, log.vehicleSpeedKph, 'b-', 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Speed (km/h)');
legend('Target', 'Actual', 'Location', 'best');
title('Race-cycle speed tracking');
nexttile;
plot(timeS, log.targetAccelerationMps2, 'k--', 'LineWidth', 1.0);
hold on;
plot(timeS, log.actualAccelerationMps2, 'Color', [0.85 0.25 0.10], ...
    'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Target profile derivative', 'Vehicle', 'Location', 'best');

figures.motor = figure('Name', 'PMSM operating map', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile;
rpmAxis = linspace(0, cfg.motor.maximumSpeedRpm, 500).';
omegaAxis = rpmAxis*2*pi/60;
positiveEnvelope = min(cfg.motor.maximumTorqueNm, ...
    cfg.motor.maximumPowerW./max(omegaAxis, 1));
negativeEnvelope = -min(cfg.motor.maximumRegenTorqueNm, ...
    cfg.motor.maximumPowerW./max(omegaAxis, 1));
positiveHandle = plot(rpmAxis, positiveEnvelope, 'k-', 'LineWidth', 1.4);
hold on;
negativeHandle = plot(rpmAxis, negativeEnvelope, 'k-', 'LineWidth', 1.4);
normalMask = ~log.fieldWeakening;
normalHandle = scatter(log.motorSpeedRpm(normalMask), ...
    log.motorTorqueNm(normalMask), 12, ...
    log.motorDcPowerKw(normalMask), 'filled', 'MarkerFaceAlpha', 0.45);
weakeningHandle = scatter(log.motorSpeedRpm(log.fieldWeakening), ...
    log.motorTorqueNm(log.fieldWeakening), 17, [0.85 0.08 0.08], ...
    'filled', 'MarkerFaceAlpha', 0.65);
baseSpeedHandle = xline(cfg.motor.baseSpeedRpm, 'b--', 'Base speed');
grid on;
xlabel('Motor speed (rpm)');
ylabel('Torque (N m)');
title('Torque-speed operating points and field weakening');
legend([positiveHandle, negativeHandle, normalHandle, weakeningHandle, ...
    baseSpeedHandle], 'Motoring envelope', 'Regen envelope', ...
    'Base-region points', 'Field-weakening points', 'Base speed', ...
    'Location', 'best');
colorbar;
nexttile;
plot(timeS, log.idA, 'b-', timeS, log.idReferenceA, 'b--', ...
    timeS, log.iqA, 'r-', timeS, log.iqReferenceA, 'r--', ...
    'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('dq current (A)');
legend('i_d', 'i_d^*', 'i_q', 'i_q^*', 'Location', 'best');
title('FOC current tracking');

figures.battery = figure('Name', 'Battery and balancing', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile;
plot(timeS, log.packSocPercent, 'b-', 'LineWidth', 1.3);
hold on;
plot(timeS, log.minimumSocPercent, 'Color', [0.45 0.65 1.0], ...
    'LineStyle', '--');
plot(timeS, log.maximumSocPercent, 'Color', [0.10 0.30 0.75], ...
    'LineStyle', '--');
grid on;
xlabel('Time (s)');
ylabel('SoC (%)');
legend('Mean', 'Minimum group', 'Maximum group', 'Location', 'best');
title('Pack state of charge');
nexttile;
groupHandles = plot(timeS, double(log.cellVoltageV), ...
    'Color', [0.72 0.78 0.88]);
hold on;
minimumHandle = plot(timeS, min(double(log.cellVoltageV), [], 2), ...
    'k-', 'LineWidth', 1.3);
maximumHandle = plot(timeS, max(double(log.cellVoltageV), [], 2), ...
    'r-', 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Series-group voltage (V)');
title(sprintf('Cell-group voltage spread and %s balancing', cfg.bms.mode));
legend([groupHandles(1), minimumHandle, maximumHandle], ...
    'Individual groups', 'Minimum', 'Maximum', 'Location', 'best');

figures.thermal = figure('Name', 'Electro-thermal management', 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile;
minimumCellTemperature = min(double(log.cellTemperatureC), [], 2);
meanCellTemperature = mean(double(log.cellTemperatureC), 2);
maximumCellTemperature = max(double(log.cellTemperatureC), [], 2);
plot(timeS, minimumCellTemperature, 'Color', [0.35 0.65 1.0]);
hold on;
plot(timeS, meanCellTemperature, 'b-', 'LineWidth', 1.2);
plot(timeS, maximumCellTemperature, 'Color', [0.05 0.20 0.65]);
plot(timeS, log.motorTemperatureC, 'r-', 'LineWidth', 1.2);
plot(timeS, log.coolantTemperatureC, 'Color', [0.0 0.60 0.55], ...
    'LineWidth', 1.2);
plot(timeS, log.ambientTemperatureC, 'k--');
grid on;
xlabel('Time (s)');
ylabel('Temperature (degC)');
legend('Cell min', 'Cell mean', 'Cell max', 'Motor', 'Coolant', ...
    'Ambient', 'Location', 'best');
title('Motor, battery and coolant temperatures');
nexttile;
yyaxis left;
plot(timeS, log.coolantMassFlowKgS, 'b-', 'LineWidth', 1.2);
ylabel('Coolant mass flow (kg/s)');
yyaxis right;
stairs(timeS, double(log.passiveBalanceCount), 'Color', [0.85 0.25 0.10]);
ylabel('Passively balanced groups');
grid on;
xlabel('Time (s)');
title('Cooling command and BMS activity');

if cfg.output.saveFigures
    if ~exist(cfg.output.directory, 'dir')
        mkdir(cfg.output.directory);
    end
    exportgraphics(figures.speed, fullfile(cfg.output.directory, ...
        '01_speed_tracking.png'), 'Resolution', 180);
    exportgraphics(figures.motor, fullfile(cfg.output.directory, ...
        '02_motor_operating_map.png'), 'Resolution', 180);
    exportgraphics(figures.battery, fullfile(cfg.output.directory, ...
        '03_battery_balancing.png'), 'Resolution', 180);
    exportgraphics(figures.thermal, fullfile(cfg.output.directory, ...
        '04_thermal_management.png'), 'Resolution', 180);
end
end
