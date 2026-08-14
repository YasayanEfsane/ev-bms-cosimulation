classdef BatteryPack < DynamicComponent
    %BATTERYPACK 96s10p 2-RC Thevenin pack and 100 ms balancing BMS.
    %   Each electrical/thermal state represents one parallel group. The
    %   nParallel scaling converts cell LUT parameters into group parameters.

    properties (SetAccess = private)
        Config
        BmsConfig
        PassiveMask
        ActiveCurrentCommandA
        BalancingEnabled
        LastBalanceUpdateS
        LastVoltageSpreadV
        Interpolants
    end

    methods
        function obj = BatteryPack(config, bmsConfig)
            obj@DynamicComponent('BatteryPack');
            obj.Config = config;
            obj.BmsConfig = bmsConfig;
            obj.PassiveMask = false(config.nSeries, 1);
            obj.ActiveCurrentCommandA = zeros(config.nSeries, 1);
            obj.BalancingEnabled = false;
            obj.LastBalanceUpdateS = -inf;
            obj.LastVoltageSpreadV = 0;
            grid = {config.temperatureGridC, config.socGrid};
            obj.Interpolants.Voc = griddedInterpolant(grid, ...
                config.lut.Voc, 'linear', 'nearest');
            obj.Interpolants.R0 = griddedInterpolant(grid, ...
                config.lut.R0, 'linear', 'nearest');
            obj.Interpolants.R1 = griddedInterpolant(grid, ...
                config.lut.R1, 'linear', 'nearest');
            obj.Interpolants.C1 = griddedInterpolant(grid, ...
                config.lut.C1, 'linear', 'nearest');
            obj.Interpolants.R2 = griddedInterpolant(grid, ...
                config.lut.R2, 'linear', 'nearest');
            obj.Interpolants.C2 = griddedInterpolant(grid, ...
                config.lut.C2, 'linear', 'nearest');
        end

        function count = stateCount(obj)
            count = 3*obj.Config.nSeries;
        end

        function state = initialState(obj)
            groupIndex = (1:obj.Config.nSeries).';
            deterministicSpread = sin(2*pi*groupIndex/obj.Config.nSeries) + ...
                0.35*cos(2*pi*groupIndex/11);
            deterministicSpread = deterministicSpread / ...
                max(abs(deterministicSpread));
            soc = obj.Config.initialSoc + ...
                obj.Config.initialSocSpread*deterministicSpread;
            state = [zeros(obj.Config.nSeries, 1); ...
                zeros(obj.Config.nSeries, 1); soc];
        end

        function [v1, v2, soc] = unpackState(obj, state)
            n = obj.Config.nSeries;
            v1 = state(1:n);
            v2 = state(n+(1:n));
            soc = state(2*n+(1:n));
        end

        function parameter = lookupParameters(obj, soc, temperatureC)
            socQuery = min(max(soc(:), obj.Config.socGrid(1)), ...
                obj.Config.socGrid(end));
            temperatureQuery = min(max(temperatureC(:), ...
                obj.Config.temperatureGridC(1)), ...
                obj.Config.temperatureGridC(end));
            parameter.Voc = obj.Interpolants.Voc(temperatureQuery, socQuery);
            parameter.R0 = obj.Interpolants.R0(temperatureQuery, socQuery) / ...
                obj.Config.nParallel;
            parameter.R1 = obj.Interpolants.R1(temperatureQuery, socQuery) / ...
                obj.Config.nParallel;
            parameter.C1 = obj.Interpolants.C1(temperatureQuery, socQuery) * ...
                obj.Config.nParallel;
            parameter.R2 = obj.Interpolants.R2(temperatureQuery, socQuery) / ...
                obj.Config.nParallel;
            parameter.C2 = obj.Interpolants.C2(temperatureQuery, socQuery) * ...
                obj.Config.nParallel;
        end

        function snapshot = openCircuitSnapshot(obj, state, temperatureC)
            [v1, v2, soc] = obj.unpackState(state);
            parameter = obj.lookupParameters(soc, temperatureC);
            polarizationFreeVoltage = parameter.Voc - v1 - v2;
            [balanceCurrentA, passiveHeatW, activeHeatW] = ...
                obj.balancingTerms(polarizationFreeVoltage);
            cellEquivalentVoltage = polarizationFreeVoltage - ...
                parameter.R0.*balanceCurrentA;
            snapshot.parameter = parameter;
            snapshot.v1V = v1;
            snapshot.v2V = v2;
            snapshot.soc = soc;
            snapshot.balanceCurrentA = balanceCurrentA;
            snapshot.passiveHeatW = passiveHeatW;
            snapshot.activeHeatW = activeHeatW;
            snapshot.cellEquivalentVoltageV = cellEquivalentVoltage;
            snapshot.packTheveninVoltageV = sum(cellEquivalentVoltage);
            snapshot.packResistanceOhm = sum(parameter.R0);
        end

        function [stateDerivative, output] = evaluate(obj, state, ...
                temperatureC, requestedPowerW)
            snapshot = obj.openCircuitSnapshot(state, temperatureC);
            [stateDerivative, output] = obj.evaluateSnapshot( ...
                snapshot, requestedPowerW);
        end

        function [stateDerivative, output] = evaluateSnapshot(obj, snapshot, ...
                requestedPowerW)
            % One LUT snapshot is reused for the algebraic voltage correction.
            parameter = snapshot.parameter;
            ePack = max(snapshot.packTheveninVoltageV, 1);
            rPack = max(snapshot.packResistanceOhm, eps);

            maximumPowerW = 0.98*ePack^2/(4*rPack);
            feasiblePowerW = min(requestedPowerW, maximumPowerW);
            discriminant = max(ePack^2-4*rPack*feasiblePowerW, 0);
            packCurrentA = 2*feasiblePowerW / ...
                max(ePack+sqrt(discriminant), eps);

            dischargeVoltageLimitA = max(0, min( ...
                (snapshot.cellEquivalentVoltageV - ...
                obj.Config.minimumCellVoltageV)./parameter.R0));
            chargeVoltageLimitA = max(0, min( ...
                (obj.Config.maximumCellVoltageV - ...
                snapshot.cellEquivalentVoltageV)./parameter.R0));
            dischargeLimitA = min(obj.Config.maximumDischargeCurrentA, ...
                dischargeVoltageLimitA);
            chargeLimitA = min(obj.Config.maximumChargeCurrentA, ...
                chargeVoltageLimitA);
            packCurrentA = min(max(packCurrentA, -chargeLimitA), ...
                dischargeLimitA);

            cellVoltageV = snapshot.cellEquivalentVoltageV - ...
                parameter.R0*packCurrentA;
            packVoltageV = sum(cellVoltageV);
            groupCurrentA = packCurrentA + snapshot.balanceCurrentA;
            v1Derivative = groupCurrentA./parameter.C1 - ...
                snapshot.v1V./(parameter.R1.*parameter.C1);
            v2Derivative = groupCurrentA./parameter.C2 - ...
                snapshot.v2V./(parameter.R2.*parameter.C2);

            effectiveCoulombCurrentA = max(groupCurrentA, 0) / ...
                obj.Config.coulombicEfficiencyDischarge + ...
                min(groupCurrentA, 0)*obj.Config.coulombicEfficiencyCharge;
            groupCapacityAs = obj.Config.cellCapacityAh * ...
                obj.Config.nParallel * 3600;
            socDerivative = -effectiveCoulombCurrentA/groupCapacityAs;
            stateDerivative = [v1Derivative; v2Derivative; socDerivative];

            electrochemicalHeatW = groupCurrentA.^2.*parameter.R0 + ...
                snapshot.v1V.^2./parameter.R1 + ...
                snapshot.v2V.^2./parameter.R2;
            balanceHeatW = snapshot.passiveHeatW + snapshot.activeHeatW;
            totalHeatW = electrochemicalHeatW + balanceHeatW;

            output.packCurrentA = packCurrentA;
            output.packVoltageV = packVoltageV;
            output.packPowerW = packVoltageV*packCurrentA;
            output.requestedPowerW = requestedPowerW;
            output.powerDeficitW = requestedPowerW-output.packPowerW;
            output.cellVoltageV = cellVoltageV;
            output.soc = snapshot.soc;
            output.meanSoc = mean(snapshot.soc);
            output.minimumSoc = min(snapshot.soc);
            output.maximumSoc = max(snapshot.soc);
            output.groupCurrentA = groupCurrentA;
            output.balanceCurrentA = snapshot.balanceCurrentA;
            output.balanceHeatW = balanceHeatW;
            output.totalHeatW = totalHeatW;
            output.parameter = parameter;
            output.activeBalanceCount = nnz( ...
                abs(obj.ActiveCurrentCommandA) > 1e-6);
            output.passiveBalanceCount = nnz(obj.PassiveMask);
            output.voltageSpreadV = max(cellVoltageV)-min(cellVoltageV);
        end

        function updateBalancing(obj, timeS, cellVoltageV, soc, ~)
            % Called only by the outer RK4 scheduler at exact 100 ms ticks.
            spreadV = max(cellVoltageV)-min(cellVoltageV);
            if spreadV >= obj.BmsConfig.startVoltageSpreadV
                obj.BalancingEnabled = true;
            elseif spreadV <= obj.BmsConfig.stopVoltageSpreadV
                obj.BalancingEnabled = false;
            end

            mode = lower(string(obj.BmsConfig.mode));
            passiveSelected = mode == "passive" || mode == "hybrid";
            activeSelected = mode == "active" || mode == "hybrid";
            minimumVoltage = min(cellVoltageV);
            maximumVoltage = max(cellVoltageV);
            highThreshold = minimumVoltage + max( ...
                obj.BmsConfig.stopVoltageSpreadV, 0.55*spreadV);
            lowThreshold = maximumVoltage - max( ...
                obj.BmsConfig.stopVoltageSpreadV, 0.55*spreadV);
            eligible = soc(:) > obj.BmsConfig.minimumBalancingSoc;
            sourceMask = obj.BalancingEnabled & eligible & ...
                (cellVoltageV(:) > highThreshold);
            sinkMask = obj.BalancingEnabled & ...
                (cellVoltageV(:) < lowThreshold);

            obj.PassiveMask = passiveSelected & sourceMask;
            sourceCount = max(nnz(sourceMask), 1);
            sinkCount = max(nnz(sinkMask), 1);
            activeSourceA = activeSelected * ...
                obj.BmsConfig.activeBalanceCurrentA*double(sourceMask) / ...
                sourceCount;
            activeSinkA = -obj.BmsConfig.activeEfficiency* ...
                sum(activeSourceA)*double(sinkMask)/sinkCount;
            obj.ActiveCurrentCommandA = activeSourceA + activeSinkA;
            obj.LastBalanceUpdateS = timeS;
            obj.LastVoltageSpreadV = spreadV;
        end

        function status = balancingStatus(obj)
            status.enabled = obj.BalancingEnabled;
            status.passiveMask = obj.PassiveMask;
            status.activeCurrentCommandA = obj.ActiveCurrentCommandA;
            status.lastUpdateS = obj.LastBalanceUpdateS;
            status.lastVoltageSpreadV = obj.LastVoltageSpreadV;
            status.mode = obj.BmsConfig.mode;
        end
    end

    methods (Access = private)
        function [balanceCurrentA, passiveHeatW, activeHeatW] = ...
                balancingTerms(obj, availableCellVoltageV)
            passiveCurrentA = double(obj.PassiveMask).* ...
                max(availableCellVoltageV, 0) / ...
                obj.BmsConfig.passiveResistanceOhm;
            balanceCurrentA = passiveCurrentA + ...
                obj.ActiveCurrentCommandA;
            passiveHeatW = passiveCurrentA.^2 * ...
                obj.BmsConfig.passiveResistanceOhm;
            activeSourcePowerW = max(obj.ActiveCurrentCommandA, 0) .* ...
                max(availableCellVoltageV, 0);
            activeHeatW = (1-obj.BmsConfig.activeEfficiency) * ...
                activeSourcePowerW;
        end
    end
end
