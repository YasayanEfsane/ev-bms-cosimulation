classdef EVPowertrainSystem < handle
    %EVPOWERTRAINSYSTEM Dependency-injected multiphysics state orchestrator.
    %   Every continuous component state is packed into Layout and evaluated
    %   in one derivative function consumed by the single RK4 integrator.

    properties (SetAccess = private)
        Vehicle
        Motor
        Battery
        Thermal
        Config
        Layout
        StateCount
    end

    methods
        function obj = EVPowertrainSystem(vehicle, motor, battery, thermal, config)
            obj.Vehicle = vehicle;
            obj.Motor = motor;
            obj.Battery = battery;
            obj.Thermal = thermal;
            obj.Config = config;

            cursor = 1;
            layout.vehicleSpeed = cursor;
            cursor = cursor + 1;
            layout.motor = cursor:(cursor+motor.stateCount()-1);
            cursor = cursor + motor.stateCount();
            n = config.battery.nSeries;
            layout.batteryV1 = cursor:(cursor+n-1);
            cursor = cursor + n;
            layout.batteryV2 = cursor:(cursor+n-1);
            cursor = cursor + n;
            layout.batterySoc = cursor:(cursor+n-1);
            cursor = cursor + n;
            layout.thermalCell = cursor:(cursor+n-1);
            cursor = cursor + n;
            layout.thermalMotor = cursor;
            cursor = cursor + 1;
            layout.thermalCoolant = cursor;
            cursor = cursor + 1;
            layout.battery = [layout.batteryV1, layout.batteryV2, ...
                layout.batterySoc];
            layout.thermal = [layout.thermalCell, layout.thermalMotor, ...
                layout.thermalCoolant];
            layout.count = cursor-1;
            obj.Layout = layout;
            obj.StateCount = layout.count;
        end

        function state = initialState(obj)
            state = zeros(obj.StateCount, 1);
            state(obj.Layout.vehicleSpeed) = 0;
            state(obj.Layout.motor) = obj.Motor.initialState();
            state(obj.Layout.battery) = obj.Battery.initialState();
            state(obj.Layout.thermal) = obj.Thermal.initialState();
        end

        function derivative = derivative(obj, timeS, state, drive)
            derivative = obj.evaluate(timeS, state, drive);
        end

        function output = observe(obj, timeS, state, drive)
            [~, output] = obj.evaluate(timeS, state, drive);
        end

        function [stateDerivative, output] = evaluate(obj, timeS, state, drive)
            input = sampleDriveCycle(drive, timeS);
            layout = obj.Layout;
            vehicleSpeedMps = state(layout.vehicleSpeed);
            motorState = state(layout.motor);
            batteryState = state(layout.battery);
            thermalState = state(layout.thermal);
            [cellTemperatureC, ~, ~] = ...
                obj.Thermal.unpackState(thermalState);
            omegaMechanical = obj.Vehicle.motorSpeed(vehicleSpeedMps);

            openSnapshot = obj.Battery.openCircuitSnapshot( ...
                batteryState, cellTemperatureC);
            estimatedDcVoltage = max(openSnapshot.packTheveninVoltageV, ...
                obj.Config.motor.referenceVoltageFloorV);
            [~, motorPreview] = obj.Motor.evaluate(motorState, ...
                input.targetSpeedMps, vehicleSpeedMps, omegaMechanical, ...
                estimatedDcVoltage);
            requestedPowerPreviewW = motorPreview.dcPowerW + ...
                obj.Config.battery.auxiliaryPowerW;
            [~, batteryPreview] = obj.Battery.evaluateSnapshot( ...
                openSnapshot, requestedPowerPreviewW);

            [motorDerivative, motorOutput] = obj.Motor.evaluate(motorState, ...
                input.targetSpeedMps, vehicleSpeedMps, omegaMechanical, ...
                batteryPreview.packVoltageV);
            requestedPowerW = motorOutput.dcPowerW + ...
                obj.Config.battery.auxiliaryPowerW;
            [batteryDerivative, batteryOutput] = ...
                obj.Battery.evaluateSnapshot(openSnapshot, requestedPowerW);
            [vehicleDerivative, vehicleOutput] = obj.Vehicle.evaluate( ...
                vehicleSpeedMps, motorOutput.torqueNm, input.gradeRad);
            [thermalDerivative, thermalOutput] = obj.Thermal.evaluate( ...
                thermalState, batteryOutput.totalHeatW, ...
                motorOutput.totalHeatW, input.ambientC, vehicleSpeedMps);

            stateDerivative = zeros(obj.StateCount, 1);
            stateDerivative(layout.vehicleSpeed) = vehicleDerivative;
            stateDerivative(layout.motor) = motorDerivative;
            stateDerivative(layout.battery) = batteryDerivative;
            stateDerivative(layout.thermal) = thermalDerivative;

            if nargout > 1
                output.timeS = timeS;
                output.input = input;
                output.vehicle = vehicleOutput;
                output.motor = motorOutput;
                output.battery = batteryOutput;
                output.thermal = thermalOutput;
                output.vehicleSpeedMps = vehicleSpeedMps;
            end
        end

        function updateBMS(obj, timeS, state, drive)
            observation = obj.observe(timeS, state, drive);
            obj.Battery.updateBalancing(timeS, ...
                observation.battery.cellVoltageV, ...
                observation.battery.soc, ...
                observation.thermal.cellTemperatureC);
        end

        function projectedState = projectState(obj, state)
            % Projection only guards physical domains after a complete RK4
            % step; no component is integrated separately.
            projectedState = state;
            layout = obj.Layout;
            projectedState(layout.vehicleSpeed) = max(0, ...
                projectedState(layout.vehicleSpeed));
            currentLimit = 1.10*obj.Config.motor.maximumCurrentA;
            projectedState(layout.motor(1:2)) = min(max( ...
                projectedState(layout.motor(1:2)), -currentLimit), currentLimit);
            speedIntegralLimit = 2*obj.Config.motor.maximumTorqueNm / ...
                max(obj.Config.motor.speedKi, eps);
            projectedState(layout.motor(3)) = min(max( ...
                projectedState(layout.motor(3)), -speedIntegralLimit), ...
                speedIntegralLimit);
            currentIntegralLimit = obj.Config.battery.packClassVoltageV / ...
                max(obj.Motor.CurrentKi, eps);
            projectedState(layout.motor(4:5)) = min(max( ...
                projectedState(layout.motor(4:5)), -currentIntegralLimit), ...
                currentIntegralLimit);
            projectedState(layout.motor(6)) = mod( ...
                projectedState(layout.motor(6)), 2*pi);
            projectedState(layout.batteryV1) = min(max( ...
                projectedState(layout.batteryV1), -1.5), 1.5);
            projectedState(layout.batteryV2) = min(max( ...
                projectedState(layout.batteryV2), -1.5), 1.5);
            projectedState(layout.batterySoc) = min(max( ...
                projectedState(layout.batterySoc), 0), 1);
            projectedState(layout.thermal) = min(max( ...
                projectedState(layout.thermal), -40), 180);
        end

        function layoutTable = stateLayoutTable(obj)
            n = obj.Config.battery.nSeries;
            startIndex = [obj.Layout.vehicleSpeed; obj.Layout.motor(1); ...
                obj.Layout.batteryV1(1); obj.Layout.batteryV2(1); ...
                obj.Layout.batterySoc(1); obj.Layout.thermalCell(1); ...
                obj.Layout.thermalMotor; obj.Layout.thermalCoolant];
            endIndex = [obj.Layout.vehicleSpeed; obj.Layout.motor(end); ...
                obj.Layout.batteryV1(end); obj.Layout.batteryV2(end); ...
                obj.Layout.batterySoc(end); obj.Layout.thermalCell(end); ...
                obj.Layout.thermalMotor; obj.Layout.thermalCoolant];
            stateName = {'Vehicle speed'; 'PMSM + FOC states'; ...
                'Battery RC-1 voltages'; 'Battery RC-2 voltages'; ...
                'Parallel-group SoC'; 'Parallel-group temperature'; ...
                'Motor temperature'; 'Coolant temperature'};
            stateDimension = [1; obj.Motor.stateCount(); n; n; n; n; 1; 1];
            layoutTable = table(stateName, startIndex, endIndex, stateDimension, ...
                'VariableNames', {'State', 'StartIndex', 'EndIndex', ...
                'Dimension'});
        end
    end
end
