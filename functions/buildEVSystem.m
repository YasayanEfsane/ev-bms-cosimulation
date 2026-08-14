function system = buildEVSystem(cfg)
%BUILDEVSYSTEM Composition root implementing constructor dependency injection.

vehicle = VehicleDynamics(cfg.vehicle, cfg.motor.rotorInertiaKgM2);
motor = PMSMMotor(cfg.motor, cfg.inverter);
battery = BatteryPack(cfg.battery, cfg.bms);
thermal = ThermalManager(cfg.thermal, cfg.battery.nSeries);
system = EVPowertrainSystem(vehicle, motor, battery, thermal, cfg);
end

