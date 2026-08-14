classdef PMSMMotor < DynamicComponent
    %PMSMMOTOR Nonlinear PMSM, FOC, field weakening and inverter losses.

    properties (SetAccess = private)
        Config
        Inverter
        CurrentKpD
        CurrentKpQ
        CurrentKi
        BaseSpeedRadS
        MaximumSpeedRadS
    end

    methods
        function obj = PMSMMotor(config, inverterConfig)
            obj@DynamicComponent('PMSMMotor');
            obj.Config = config;
            obj.Inverter = inverterConfig;
            obj.CurrentKpD = config.ldH*config.currentBandwidthRadS;
            obj.CurrentKpQ = config.lqH*config.currentBandwidthRadS;
            obj.CurrentKi = config.statorResistanceOhm* ...
                config.currentBandwidthRadS;
            obj.BaseSpeedRadS = config.baseSpeedRpm*2*pi/60;
            obj.MaximumSpeedRadS = config.maximumSpeedRpm*2*pi/60;
        end

        function count = stateCount(~)
            count = 6;
        end

        function state = initialState(~)
            % [id, iq, speed-PI integral, d-PI integral, q-PI integral, theta_e]
            state = zeros(6, 1);
        end

        function [stateDerivative, output] = evaluate(obj, state, ...
                targetSpeedMps, actualSpeedMps, omegaMechanical, dcVoltage)
            idState = state(1);
            iqState = state(2);
            speedIntegral = state(3);
            dIntegral = state(4);
            qIntegral = state(5);
            thetaElectrical = state(6);

            % The measured dq currents are reconstructed through explicit
            % inverse-Park/inverse-Clarke and Clarke/Park matrices.
            phaseCurrent = obj.dqToAbc([idState; iqState], thetaElectrical);
            measuredDq = obj.abcToDq(phaseCurrent, thetaElectrical);
            id = measuredDq(1);
            iq = measuredDq(2);

            speedError = targetSpeedMps - actualSpeedMps;
            torqueUnsaturated = obj.Config.speedKp*speedError + ...
                obj.Config.speedKi*speedIntegral;
            motoringLimit = min(obj.Config.maximumTorqueNm, ...
                obj.Config.maximumPowerW/max(abs(omegaMechanical), 1));
            regenLimit = min(obj.Config.maximumRegenTorqueNm, ...
                obj.Config.maximumPowerW/max(abs(omegaMechanical), 1));
            torqueCommand = min(max(torqueUnsaturated, -regenLimit), ...
                motoringLimit);
            speedIntegralDerivative = speedError + ...
                obj.Config.speedAntiWindup*(torqueCommand-torqueUnsaturated) / ...
                max(obj.Config.speedKi, eps);

            electricalSpeed = obj.Config.polePairs*omegaMechanical;
            usableDcVoltage = max(abs(dcVoltage), ...
                obj.Config.referenceVoltageFloorV);
            voltageLimit = obj.Inverter.modulationIndex*usableDcVoltage/sqrt(3);
            weakeningFraction = min(max((abs(omegaMechanical)-obj.BaseSpeedRadS) / ...
                (obj.MaximumSpeedRadS-obj.BaseSpeedRadS), 0), 1);
            idFromSpeed = -0.82*obj.Config.maximumCurrentA*weakeningFraction;
            idFromVoltage = min(0, ...
                (voltageLimit/max(abs(electricalSpeed), 1) - ...
                obj.Config.pmFluxWb)/obj.Config.ldH);
            idReference = max(-0.90*obj.Config.maximumCurrentA, ...
                min(0, min(idFromSpeed, idFromVoltage)));

            torquePerAmp = 1.5*obj.Config.polePairs* ...
                (obj.Config.pmFluxWb + ...
                (obj.Config.ldH-obj.Config.lqH)*idReference);
            iqLimit = sqrt(max(obj.Config.maximumCurrentA^2-idReference^2, 0));
            iqReference = min(max(torqueCommand/max(torquePerAmp, eps), ...
                -iqLimit), iqLimit);

            dError = idReference - id;
            qError = iqReference - iq;
            vdUnsaturated = obj.CurrentKpD*dError + obj.CurrentKi*dIntegral ...
                - electricalSpeed*obj.Config.lqH*iq;
            vqUnsaturated = obj.CurrentKpQ*qError + obj.CurrentKi*qIntegral ...
                + electricalSpeed*(obj.Config.ldH*id + obj.Config.pmFluxWb);
            unsaturatedMagnitude = hypot(vdUnsaturated, vqUnsaturated);
            voltageScale = min(1, voltageLimit/max(unsaturatedMagnitude, eps));
            vd = voltageScale*vdUnsaturated;
            vq = voltageScale*vqUnsaturated;
            dIntegralDerivative = dError + obj.Config.currentAntiWindup * ...
                (vd-vdUnsaturated)/max(obj.CurrentKpD, eps);
            qIntegralDerivative = qError + obj.Config.currentAntiWindup * ...
                (vq-vqUnsaturated)/max(obj.CurrentKpQ, eps);

            idDerivative = (vd - obj.Config.statorResistanceOhm*id + ...
                electricalSpeed*obj.Config.lqH*iq)/obj.Config.ldH;
            iqDerivative = (vq - obj.Config.statorResistanceOhm*iq - ...
                electricalSpeed*(obj.Config.ldH*id + ...
                obj.Config.pmFluxWb))/obj.Config.lqH;
            thetaDerivative = electricalSpeed;
            stateDerivative = [idDerivative; iqDerivative; ...
                speedIntegralDerivative; dIntegralDerivative; ...
                qIntegralDerivative; thetaDerivative];

            electromagneticTorque = 1.5*obj.Config.polePairs * ...
                (obj.Config.pmFluxWb*iq + ...
                (obj.Config.ldH-obj.Config.lqH)*id*iq);
            copperLossW = 1.5*obj.Config.statorResistanceOhm*(id^2+iq^2);
            fluxD = obj.Config.pmFluxWb + obj.Config.ldH*id;
            fluxQ = obj.Config.lqH*iq;
            ironLossW = obj.Config.ironLossCoefficient*electricalSpeed^2 * ...
                (fluxD^2+fluxQ^2);
            windageLossW = obj.Config.windageCoefficient*abs(omegaMechanical)^3;

            phasePeakA = hypot(id, iq);
            phaseRmsA = phasePeakA/sqrt(2);
            phaseAverageA = 2*phasePeakA/pi;
            conductionLossW = 3*(obj.Inverter.deviceDropV*phaseAverageA + ...
                obj.Inverter.onResistanceOhm*phaseRmsA^2);
            switchingLossW = 6*0.5*usableDcVoltage*phaseAverageA * ...
                obj.Inverter.switchingRiseFallS* ...
                obj.Inverter.switchingFrequencyHz* ...
                obj.Inverter.switchingEnergyScale;
            inverterLossW = conductionLossW + switchingLossW;
            motorLossW = copperLossW + ironLossW + windageLossW;
            mechanicalPowerW = electromagneticTorque*omegaMechanical;
            dcPowerW = mechanicalPowerW + motorLossW + inverterLossW;

            phaseVoltage = obj.dqToAbc([vd; vq], thetaElectrical);
            output.idA = id;
            output.iqA = iq;
            output.idReferenceA = idReference;
            output.iqReferenceA = iqReference;
            output.torqueCommandNm = torqueCommand;
            output.torqueNm = electromagneticTorque;
            output.omegaMechanicalRadS = omegaMechanical;
            output.speedRpm = omegaMechanical*60/(2*pi);
            output.electricalSpeedRadS = electricalSpeed;
            output.voltageDqV = [vd; vq];
            output.phaseCurrentA = phaseCurrent;
            output.phaseVoltageV = phaseVoltage;
            output.voltageUtilization = unsaturatedMagnitude/max(voltageLimit, eps);
            output.fieldWeakening = idReference < -1;
            output.copperLossW = copperLossW;
            output.ironLossW = ironLossW;
            output.windageLossW = windageLossW;
            output.inverterLossW = inverterLossW;
            output.totalHeatW = motorLossW + inverterLossW;
            output.mechanicalPowerW = mechanicalPowerW;
            output.dcPowerW = dcPowerW;
            output.dcCurrentA = dcPowerW/max(usableDcVoltage, eps);
        end

        function abc = dqToAbc(~, dq, thetaElectrical)
            inversePark = [cos(thetaElectrical), -sin(thetaElectrical); ...
                sin(thetaElectrical), cos(thetaElectrical)];
            inverseClarke = [1, 0; -0.5, sqrt(3)/2; -0.5, -sqrt(3)/2];
            abc = inverseClarke*inversePark*dq(:);
        end

        function dq = abcToDq(~, abc, thetaElectrical)
            clarke = (2/3)*[1, -0.5, -0.5; ...
                0, sqrt(3)/2, -sqrt(3)/2];
            park = [cos(thetaElectrical), sin(thetaElectrical); ...
                -sin(thetaElectrical), cos(thetaElectrical)];
            dq = park*clarke*abc(:);
        end
    end
end

