function log = initializeSimulationLog(numberOfLogs, groupCount)
%INITIALIZESIMULATIONLOG Allocate only the requested 100 Hz output channels.

log.timeS = zeros(numberOfLogs, 1);
log.targetSpeedKph = zeros(numberOfLogs, 1, 'single');
log.vehicleSpeedKph = zeros(numberOfLogs, 1, 'single');
log.targetAccelerationMps2 = zeros(numberOfLogs, 1, 'single');
log.actualAccelerationMps2 = zeros(numberOfLogs, 1, 'single');
log.gradePercent = zeros(numberOfLogs, 1, 'single');
log.motorTorqueNm = zeros(numberOfLogs, 1, 'single');
log.torqueCommandNm = zeros(numberOfLogs, 1, 'single');
log.motorSpeedRpm = zeros(numberOfLogs, 1, 'single');
log.idA = zeros(numberOfLogs, 1, 'single');
log.iqA = zeros(numberOfLogs, 1, 'single');
log.idReferenceA = zeros(numberOfLogs, 1, 'single');
log.iqReferenceA = zeros(numberOfLogs, 1, 'single');
log.fieldWeakening = false(numberOfLogs, 1);
log.motorMechanicalPowerKw = zeros(numberOfLogs, 1, 'single');
log.motorDcPowerKw = zeros(numberOfLogs, 1, 'single');
log.motorLossKw = zeros(numberOfLogs, 1, 'single');
log.packCurrentA = zeros(numberOfLogs, 1, 'single');
log.packVoltageV = zeros(numberOfLogs, 1, 'single');
log.packSocPercent = zeros(numberOfLogs, 1, 'single');
log.minimumSocPercent = zeros(numberOfLogs, 1, 'single');
log.maximumSocPercent = zeros(numberOfLogs, 1, 'single');
log.cellVoltageV = zeros(numberOfLogs, groupCount, 'single');
log.balanceCurrentA = zeros(numberOfLogs, groupCount, 'single');
log.passiveBalanceCount = zeros(numberOfLogs, 1, 'uint16');
log.activeBalanceCount = zeros(numberOfLogs, 1, 'uint16');
log.balanceHeatW = zeros(numberOfLogs, 1, 'single');
log.cellTemperatureC = zeros(numberOfLogs, groupCount, 'single');
log.motorTemperatureC = zeros(numberOfLogs, 1, 'single');
log.coolantTemperatureC = zeros(numberOfLogs, 1, 'single');
log.coolantMassFlowKgS = zeros(numberOfLogs, 1, 'single');
log.ambientTemperatureC = zeros(numberOfLogs, 1, 'single');
end

