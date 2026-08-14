function log = recordSimulationLog(log, index, observation)
%RECORDSIMULATIONLOG Store a vectorized 100 Hz observation.

log.timeS(index) = observation.timeS;
log.targetSpeedKph(index) = single(3.6*observation.input.targetSpeedMps);
log.vehicleSpeedKph(index) = single(3.6*observation.vehicleSpeedMps);
log.targetAccelerationMps2(index) = single( ...
    observation.input.targetAccelerationMps2);
log.actualAccelerationMps2(index) = single( ...
    observation.vehicle.accelerationMps2);
log.gradePercent(index) = single(100*tan(observation.input.gradeRad));
log.motorTorqueNm(index) = single(observation.motor.torqueNm);
log.torqueCommandNm(index) = single(observation.motor.torqueCommandNm);
log.motorSpeedRpm(index) = single(observation.motor.speedRpm);
log.idA(index) = single(observation.motor.idA);
log.iqA(index) = single(observation.motor.iqA);
log.idReferenceA(index) = single(observation.motor.idReferenceA);
log.iqReferenceA(index) = single(observation.motor.iqReferenceA);
log.fieldWeakening(index) = observation.motor.fieldWeakening;
log.motorMechanicalPowerKw(index) = single( ...
    observation.motor.mechanicalPowerW/1000);
log.motorDcPowerKw(index) = single(observation.motor.dcPowerW/1000);
log.motorLossKw(index) = single(observation.motor.totalHeatW/1000);
log.packCurrentA(index) = single(observation.battery.packCurrentA);
log.packVoltageV(index) = single(observation.battery.packVoltageV);
log.packSocPercent(index) = single(100*observation.battery.meanSoc);
log.minimumSocPercent(index) = single(100*observation.battery.minimumSoc);
log.maximumSocPercent(index) = single(100*observation.battery.maximumSoc);
log.cellVoltageV(index, :) = single(observation.battery.cellVoltageV(:).');
log.balanceCurrentA(index, :) = single( ...
    observation.battery.balanceCurrentA(:).');
log.passiveBalanceCount(index) = uint16( ...
    observation.battery.passiveBalanceCount);
log.activeBalanceCount(index) = uint16( ...
    observation.battery.activeBalanceCount);
log.balanceHeatW(index) = single(sum(observation.battery.balanceHeatW));
log.cellTemperatureC(index, :) = single( ...
    observation.thermal.cellTemperatureC(:).');
log.motorTemperatureC(index) = single( ...
    observation.thermal.motorTemperatureC);
log.coolantTemperatureC(index) = single( ...
    observation.thermal.coolantTemperatureC);
log.coolantMassFlowKgS(index) = single(observation.thermal.massFlowKgS);
log.ambientTemperatureC(index) = single(observation.input.ambientC);
end

