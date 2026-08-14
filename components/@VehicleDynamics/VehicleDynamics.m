classdef VehicleDynamics < DynamicComponent
    %VEHICLEDYNAMICS One-dimensional race-car longitudinal dynamics.

    properties (SetAccess = private)
        Config
        TotalMassKg
        RotorInertiaKgM2
        InertiaMatrix
        EquivalentMassKg
    end

    methods
        function obj = VehicleDynamics(config, rotorInertiaKgM2)
            obj@DynamicComponent('VehicleDynamics');
            obj.Config = config;
            obj.TotalMassKg = config.massKg + config.payloadKg;
            obj.RotorInertiaKgM2 = rotorInertiaKgM2;
            inertiaDiagonal = [obj.TotalMassKg; ...
                repmat(config.wheelInertiaKgM2, config.numberOfWheels, 1); ...
                rotorInertiaKgM2];
            obj.InertiaMatrix = diag(inertiaDiagonal);
            obj.EquivalentMassKg = obj.TotalMassKg + ...
                config.numberOfWheels*config.wheelInertiaKgM2 / ...
                config.wheelRadiusM^2 + rotorInertiaKgM2*config.gearRatio^2 / ...
                config.wheelRadiusM^2;
        end

        function omegaMechanical = motorSpeed(obj, vehicleSpeed)
            omegaMechanical = vehicleSpeed / obj.Config.wheelRadiusM * ...
                obj.Config.gearRatio;
        end

        function wheelTorque = motorToWheelTorque(obj, motorTorque, omegaMechanical)
            mechanicalPower = motorTorque*omegaMechanical;
            if mechanicalPower >= 0
                wheelTorque = motorTorque*obj.Config.gearRatio* ...
                    obj.Config.gearEfficiency;
            else
                wheelTorque = motorTorque*obj.Config.gearRatio / ...
                    obj.Config.gearEfficiency;
            end
        end

        function [speedDerivative, output] = evaluate(obj, vehicleSpeed, ...
                motorTorque, gradeRad)
            wheelTorque = obj.motorToWheelTorque(motorTorque, ...
                obj.motorSpeed(vehicleSpeed));
            tractionForce = wheelTorque / obj.Config.wheelRadiusM;
            dragForce = 0.5*obj.Config.airDensityKgM3* ...
                obj.Config.dragCoefficient*obj.Config.frontalAreaM2* ...
                vehicleSpeed*abs(vehicleSpeed);
            direction = tanh(vehicleSpeed / ...
                obj.Config.rollingVelocitySmoothing);
            rollingForce = obj.TotalMassKg*obj.Config.gravity* ...
                obj.Config.rollingCoefficient*direction*cos(gradeRad);
            gradeForce = obj.TotalMassKg*obj.Config.gravity*sin(gradeRad);
            resistanceForce = dragForce + rollingForce + gradeForce;
            speedDerivative = (tractionForce - resistanceForce) / ...
                obj.EquivalentMassKg;

            output.wheelTorqueNm = wheelTorque;
            output.tractionForceN = tractionForce;
            output.dragForceN = dragForce;
            output.rollingForceN = rollingForce;
            output.gradeForceN = gradeForce;
            output.accelerationMps2 = speedDerivative;
            output.equivalentMassKg = obj.EquivalentMassKg;
        end
    end
end

