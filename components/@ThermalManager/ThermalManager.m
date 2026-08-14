classdef ThermalManager < DynamicComponent
    %THERMALMANAGER Lumped liquid-cooling and Newton cooling model.

    properties (SetAccess = private)
        Config
        GroupCount
        CoolantThermalCapacityJK
    end

    methods
        function obj = ThermalManager(config, groupCount)
            obj@DynamicComponent('ThermalManager');
            obj.Config = config;
            obj.GroupCount = groupCount;
            obj.CoolantThermalCapacityJK = config.coolantMassKg * ...
                config.coolantSpecificHeatJKgK;
        end

        function count = stateCount(obj)
            count = obj.GroupCount + 2;
        end

        function state = initialState(obj)
            groupIndex = (1:obj.GroupCount).';
            cellTemperatureC = obj.Config.initialCellTemperatureC + ...
                0.35*sin(2*pi*groupIndex/obj.GroupCount);
            state = [cellTemperatureC; ...
                obj.Config.initialMotorTemperatureC; ...
                obj.Config.initialCoolantTemperatureC];
        end

        function [cellTemperatureC, motorTemperatureC, coolantTemperatureC] = ...
                unpackState(obj, state)
            cellTemperatureC = state(1:obj.GroupCount);
            motorTemperatureC = state(obj.GroupCount+1);
            coolantTemperatureC = state(obj.GroupCount+2);
        end

        function [stateDerivative, output] = evaluate(obj, state, ...
                batteryHeatW, motorHeatW, ambientC, vehicleSpeedMps)
            [cellTemperatureC, motorTemperatureC, coolantTemperatureC] = ...
                obj.unpackState(state);
            controlTemperatureC = max([max(cellTemperatureC), ...
                motorTemperatureC]);
            pumpFraction = min(max((controlTemperatureC - ...
                obj.Config.pumpControlStartC) / ...
                (obj.Config.pumpControlFullC - ...
                obj.Config.pumpControlStartC), 0), 1);
            massFlowKgS = obj.Config.minimumMassFlowKgS + pumpFraction * ...
                (obj.Config.maximumMassFlowKgS - ...
                obj.Config.minimumMassFlowKgS);
            flowRatio = max(massFlowKgS / ...
                obj.Config.nominalMassFlowKgS, 0.05);
            cellRthKPerW = obj.Config.cellToCoolantRthKPerW / flowRatio^0.8;
            motorRthKPerW = obj.Config.motorToCoolantRthKPerW / flowRatio^0.8;

            cellToCoolantW = (cellTemperatureC-coolantTemperatureC) / ...
                cellRthKPerW;
            motorToCoolantW = (motorTemperatureC-coolantTemperatureC) / ...
                motorRthKPerW;
            radiatorUaWK = obj.Config.radiatorUaBaseWK + ...
                obj.Config.radiatorUaFlowWK*sqrt(flowRatio) + ...
                obj.Config.radiatorUaSpeedWKPerSqrtMS* ...
                sqrt(abs(vehicleSpeedMps));
            radiatorHeatW = radiatorUaWK*(coolantTemperatureC-ambientC);
            pumpHeatW = obj.Config.pumpHeatWAtMaximum*pumpFraction^3;

            cellDerivative = (batteryHeatW(:)-cellToCoolantW) / ...
                obj.Config.cellThermalCapacityJK;
            motorDerivative = (motorHeatW-motorToCoolantW) / ...
                obj.Config.motorThermalCapacityJK;
            coolantDerivative = (sum(cellToCoolantW)+motorToCoolantW ...
                - radiatorHeatW + pumpHeatW) / obj.CoolantThermalCapacityJK;
            stateDerivative = [cellDerivative; motorDerivative; ...
                coolantDerivative];

            output.cellTemperatureC = cellTemperatureC;
            output.motorTemperatureC = motorTemperatureC;
            output.coolantTemperatureC = coolantTemperatureC;
            output.massFlowKgS = massFlowKgS;
            output.pumpFraction = pumpFraction;
            output.cellToCoolantW = cellToCoolantW;
            output.motorToCoolantW = motorToCoolantW;
            output.radiatorHeatW = radiatorHeatW;
            output.radiatorUaWK = radiatorUaWK;
            output.pumpHeatW = pumpHeatW;
        end
    end
end
