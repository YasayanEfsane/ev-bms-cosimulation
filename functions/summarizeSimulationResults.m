function summary = summarizeSimulationResults(results, printToConsole)
%SUMMARIZESIMULATIONRESULTS Calculate energy, performance, BMS and thermal KPIs.
%   summary = summarizeSimulationResults(results) prints a compact report.
%   Passing false as the second argument suppresses console output for tests.

if nargin < 2
    printToConsole = true;
end

assert(isstruct(results) && isfield(results, 'log'), ...
    'summarizeSimulationResults:InvalidResults', ...
    'Input must be a simulation results structure containing a log field.');

log = results.log;
requiredFields = {'timeS', 'vehicleSpeedKph', 'targetSpeedKph', ...
    'packVoltageV', 'packCurrentA', 'packSocPercent', 'cellVoltageV', ...
    'cellTemperatureC', 'motorTemperatureC', 'coolantTemperatureC', ...
    'passiveBalanceCount', 'activeBalanceCount', 'fieldWeakening'};
assert(all(isfield(log, requiredFields)), ...
    'summarizeSimulationResults:MissingLogField', ...
    'Simulation log does not contain every channel required for KPI analysis.');

timeS = double(log.timeS(:));
assert(numel(timeS) >= 2 && all(isfinite(timeS)) && all(diff(timeS) > 0), ...
    'summarizeSimulationResults:InvalidTime', ...
    'Logged time must contain at least two finite, increasing samples.');

vehicleSpeedKph = double(log.vehicleSpeedKph(:));
targetSpeedKph = double(log.targetSpeedKph(:));
packVoltageV = double(log.packVoltageV(:));
packCurrentA = double(log.packCurrentA(:));
packPowerW = packVoltageV.*packCurrentA;
dischargePowerW = max(packPowerW, 0);
regenerativePowerW = max(-packPowerW, 0);

summary.simulatedTimeS = timeS(end)-timeS(1);
summary.distanceKm = trapz(timeS, max(vehicleSpeedKph, 0)/3.6)/1000;
summary.energyDrawnKWh = trapz(timeS, dischargePowerW)/3.6e6;
summary.regenerativeEnergyKWh = trapz(timeS, regenerativePowerW)/3.6e6;
summary.netEnergyKWh = summary.energyDrawnKWh-summary.regenerativeEnergyKWh;

if summary.distanceKm > eps
    summary.netConsumptionWhPerKm = ...
        1000*summary.netEnergyKWh/summary.distanceKm;
else
    summary.netConsumptionWhPerKm = NaN;
end

summary.regenerationRecoveryPercent = 100*summary.regenerativeEnergyKWh / ...
    max(summary.energyDrawnKWh, eps);
summary.maximumVehicleSpeedKph = max(vehicleSpeedKph);
summary.speedTrackingRmseKph = sqrt(mean( ...
    (targetSpeedKph-vehicleSpeedKph).^2));
summary.peakDischargeCurrentA = max(max(packCurrentA, 0));
summary.peakRegenerativeCurrentA = max(max(-packCurrentA, 0));
summary.minimumPackVoltageV = min(packVoltageV);
summary.initialPackSocPercent = double(log.packSocPercent(1));
summary.finalPackSocPercent = double(log.packSocPercent(end));
summary.maximumMotorTemperatureC = max(double(log.motorTemperatureC(:)));
summary.maximumCellTemperatureC = max(double(log.cellTemperatureC(:)));
summary.finalCoolantTemperatureC = double(log.coolantTemperatureC(end));

cellVoltageV = double(log.cellVoltageV);
voltageSpreadV = max(cellVoltageV, [], 2)-min(cellVoltageV, [], 2);
summary.initialVoltageSpreadmV = 1000*voltageSpreadV(1);
summary.finalVoltageSpreadmV = 1000*voltageSpreadV(end);
summary.maximumVoltageSpreadmV = 1000*max(voltageSpreadV);
summary.maximumPassivelyBalancedGroups = max( ...
    double(log.passiveBalanceCount(:)));
summary.maximumActivelyBalancedGroups = max( ...
    double(log.activeBalanceCount(:)));
summary.fieldWeakeningDurationS = trapz(timeS, ...
    double(log.fieldWeakening(:)));

if printToConsole
    fprintf('\n===== EV SIMULATION KPI SUMMARY =====\n');
    fprintf('%-33s %12.3f s\n', 'Simulated time', summary.simulatedTimeS);
    fprintf('%-33s %12.3f km\n', 'Distance travelled', summary.distanceKm);
    fprintf('%-33s %12.4f kWh\n', 'Energy drawn', summary.energyDrawnKWh);
    fprintf('%-33s %12.4f kWh\n', 'Regenerative energy', ...
        summary.regenerativeEnergyKWh);
    fprintf('%-33s %12.4f kWh\n', 'Net energy', summary.netEnergyKWh);
    fprintf('%-33s %12.1f Wh/km\n', 'Net consumption', ...
        summary.netConsumptionWhPerKm);
    fprintf('%-33s %12.2f %%\n', 'Regeneration recovery', ...
        summary.regenerationRecoveryPercent);
    fprintf('%-33s %12.2f km/h\n', 'Maximum vehicle speed', ...
        summary.maximumVehicleSpeedKph);
    fprintf('%-33s %12.2f km/h\n', 'Speed tracking RMSE', ...
        summary.speedTrackingRmseKph);
    fprintf('%-33s %12.1f A\n', 'Peak discharge current', ...
        summary.peakDischargeCurrentA);
    fprintf('%-33s %12.1f A\n', 'Peak regenerative current', ...
        summary.peakRegenerativeCurrentA);
    fprintf('%-33s %12.1f V\n', 'Minimum pack voltage', ...
        summary.minimumPackVoltageV);
    fprintf('%-33s %12.2f %%\n', 'Final pack SoC', ...
        summary.finalPackSocPercent);
    fprintf('%-33s %12.2f degC\n', 'Maximum motor temperature', ...
        summary.maximumMotorTemperatureC);
    fprintf('%-33s %12.2f degC\n', 'Maximum cell temperature', ...
        summary.maximumCellTemperatureC);
    fprintf('%-33s %12.2f mV\n', 'Final cell-voltage spread', ...
        summary.finalVoltageSpreadmV);
    fprintf('%-33s %12.2f s\n', 'Field-weakening duration', ...
        summary.fieldWeakeningDurationS);
    fprintf('=====================================\n\n');
end
end
