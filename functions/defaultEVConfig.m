function cfg = defaultEVConfig(preset)
%DEFAULTEVCONFIG Central configuration for the EV/BMS co-simulation.
%   cfg = defaultEVConfig("full") creates the specified 600 s, 10 us case.
%   "short" and "smoke" retain the 10 us integration step but truncate the
%   simulated time so that installation and component checks finish sooner.

if nargin == 0
    preset = "full";
end

preset = lower(string(preset));
validPresets = ["full", "short", "smoke"];
assert(any(preset == validPresets), ...
    'defaultEVConfig:UnknownPreset', ...
    'Preset must be "full", "short", or "smoke".');

cfg.meta.projectName = 'Advanced EV Powertrain and BMS Co-Simulation';
cfg.meta.modelVersion = '1.0.0';

cfg.sim.integrationStep = 10e-6;
cfg.sim.logStep = 0.01;
cfg.sim.bmsStep = 0.1;
cfg.sim.preset = char(preset);
cfg.sim.showProgress = preset ~= "smoke";
cfg.sim.progressFractions = 0.05;

if preset == "full"
    cfg.sim.endTime = 600;
elseif preset == "short"
    cfg.sim.endTime = 10;
else
    cfg.sim.endTime = 0.20;
end

cfg.drive.duration = 600;
cfg.drive.sampleRate = 100;
cfg.drive.maximumSpeedKph = 170;
cfg.drive.defaultAmbientC = 25;

cfg.vehicle.massKg = 305;
cfg.vehicle.payloadKg = 75;
cfg.vehicle.wheelRadiusM = 0.235;
cfg.vehicle.wheelInertiaKgM2 = 0.72;
cfg.vehicle.numberOfWheels = 4;
cfg.vehicle.dragCoefficient = 0.88;
cfg.vehicle.frontalAreaM2 = 1.05;
cfg.vehicle.airDensityKgM3 = 1.225;
cfg.vehicle.rollingCoefficient = 0.014;
cfg.vehicle.gravity = 9.80665;
cfg.vehicle.gearRatio = 8.5;
cfg.vehicle.gearEfficiency = 0.965;
cfg.vehicle.rollingVelocitySmoothing = 0.20;

cfg.motor.polePairs = 5;
cfg.motor.statorResistanceOhm = 0.012;
cfg.motor.ldH = 0.18e-3;
cfg.motor.lqH = 0.24e-3;
cfg.motor.pmFluxWb = 0.075;
cfg.motor.rotorInertiaKgM2 = 0.018;
cfg.motor.maximumCurrentA = 450;
cfg.motor.maximumTorqueNm = 185;
cfg.motor.maximumRegenTorqueNm = 130;
cfg.motor.maximumPowerW = 160e3;
cfg.motor.baseSpeedRpm = 5600;
cfg.motor.maximumSpeedRpm = 16000;
cfg.motor.speedKp = 34;
cfg.motor.speedKi = 11;
cfg.motor.speedAntiWindup = 0.12;
cfg.motor.currentBandwidthRadS = 2*pi*1500;
cfg.motor.currentAntiWindup = 0.55;
cfg.motor.ironLossCoefficient = 0.012;
cfg.motor.windageCoefficient = 9e-8;
cfg.motor.referenceVoltageFloorV = 60;

cfg.inverter.switchingFrequencyHz = 20e3;
cfg.inverter.modulationIndex = 0.95;
cfg.inverter.deviceDropV = 0.85;
cfg.inverter.onResistanceOhm = 1.2e-3;
cfg.inverter.switchingRiseFallS = 95e-9;
cfg.inverter.switchingEnergyScale = 1.12;

cfg.battery.nSeries = 96;
cfg.battery.nParallel = 10;
cfg.battery.packClassVoltageV = 400;
cfg.battery.nominalCellVoltageV = 3.70;
cfg.battery.maximumCellVoltageV = 4.20;
cfg.battery.minimumCellVoltageV = 2.85;
cfg.battery.nominalEnergyWh = 80e3;
cfg.battery.cellCapacityAh = cfg.battery.nominalEnergyWh / ...
    (cfg.battery.nSeries * cfg.battery.nParallel * ...
    cfg.battery.nominalCellVoltageV);
cfg.battery.initialSoc = 0.92;
cfg.battery.initialSocSpread = 0.006;
cfg.battery.coulombicEfficiencyCharge = 0.985;
cfg.battery.coulombicEfficiencyDischarge = 0.998;
cfg.battery.maximumDischargeCurrentA = 650;
cfg.battery.maximumChargeCurrentA = 360;
cfg.battery.auxiliaryPowerW = 850;

cfg.battery.socGrid = [0.02, 0.10, 0.25, 0.50, 0.75, 0.90, 1.00];
cfg.battery.temperatureGridC = [-20, -5, 10, 25, 45, 60];
[temperatureMesh, socMesh] = ndgrid( ...
    cfg.battery.temperatureGridC, cfg.battery.socGrid);

ocvBase = 3.02 + 1.14*socMesh + 0.085*tanh((socMesh - 0.52)/0.10) ...
    - 0.055*exp(-socMesh/0.035) + 0.025*exp(-(1-socMesh)/0.025);
cfg.battery.lut.Voc = min(cfg.battery.maximumCellVoltageV, ...
    ocvBase + 6.5e-4*(temperatureMesh - 25));
temperaturePenalty = 1 + 1.75*exp(-(temperatureMesh + 20)/16);
lowSocPenalty = 1 + 1.55*(1-socMesh).^2 + 0.32*exp(-socMesh/0.08);
cfg.battery.lut.R0 = 2.05e-3 .* temperaturePenalty .* lowSocPenalty;
cfg.battery.lut.R1 = 1.35e-3 .* (1 + 1.15*exp(-(temperatureMesh+20)/19)) ...
    .* (1 + 0.85*(1-socMesh).^2);
cfg.battery.lut.C1 = 3100 .* (0.68 + 0.32*socMesh) ...
    .* (0.72 + 0.28*min(max((temperatureMesh+20)/45, 0), 1));
cfg.battery.lut.R2 = 2.80e-3 .* (1 + 0.72*exp(-(temperatureMesh+20)/24)) ...
    .* (1 + 0.48*(socMesh-0.55).^2);
cfg.battery.lut.C2 = 13500 .* (0.82 + 0.18*socMesh) ...
    .* (0.78 + 0.22*min(max((temperatureMesh+20)/45, 0), 1));

cfg.bms.mode = 'passive';
cfg.bms.startVoltageSpreadV = 8e-3;
cfg.bms.stopVoltageSpreadV = 3e-3;
cfg.bms.passiveResistanceOhm = 18;
cfg.bms.minimumBalancingSoc = 0.15;
cfg.bms.activeBalanceCurrentA = 1.5;
cfg.bms.activeEfficiency = 0.88;

cfg.thermal.initialCellTemperatureC = 27;
cfg.thermal.initialMotorTemperatureC = 30;
cfg.thermal.initialCoolantTemperatureC = 25;
cfg.thermal.cellThermalCapacityJK = 1450;
cfg.thermal.motorThermalCapacityJK = 9200;
cfg.thermal.coolantMassKg = 7.5;
cfg.thermal.coolantSpecificHeatJKgK = 3680;
cfg.thermal.cellToCoolantRthKPerW = 0.42;
cfg.thermal.motorToCoolantRthKPerW = 0.030;
cfg.thermal.nominalMassFlowKgS = 0.42;
cfg.thermal.minimumMassFlowKgS = 0.05;
cfg.thermal.maximumMassFlowKgS = 0.65;
cfg.thermal.pumpControlStartC = 31;
cfg.thermal.pumpControlFullC = 48;
cfg.thermal.radiatorUaBaseWK = 95;
cfg.thermal.radiatorUaFlowWK = 310;
cfg.thermal.radiatorUaSpeedWKPerSqrtMS = 34;
cfg.thermal.pumpHeatWAtMaximum = 180;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
cfg.output.directory = fullfile(projectRoot, 'output');
cfg.output.saveResults = false;
cfg.output.saveFigures = false;

cfg = validateEVConfig(cfg);
end

