classdef DynamicComponent < handle
    %DYNAMICCOMPONENT Lightweight base class shared by physical components.
    %   It provides identity and common finite-value validation while each
    %   subclass owns its equations. Components are injected into the system
    %   orchestrator rather than constructing one another.

    properties (SetAccess = protected)
        Name
    end

    methods
        function obj = DynamicComponent(name)
            obj.Name = char(name);
        end

        function assertFinite(obj, value, label)
            assert(all(isfinite(value(:))), ...
                [obj.Name ':NonFiniteState'], ...
                '%s produced a non-finite value in %s.', obj.Name, label);
        end
    end
end

