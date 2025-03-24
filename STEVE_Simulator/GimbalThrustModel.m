classdef GimbalThrustModel < matlab.System
    %  Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        nozzleOffset = 0.01;
    end

    % Pre-computed constants or internal states
    properties (Access = private)
    end

    methods (Access = protected)
        function setupImpl()
            % Perform one-time calculations, such as computing constants
        end

        function [thrust_x, thrust_z, moment] = stepImpl(thrust, nozzle_angle, CG)
            thrust_x = thrust * cos(nozzle_angle);
            thrust_z = thrust * sin(nozzle_angle);
            moment = CG*thrust_z;
        end

        function resetImpl()
            % Initialize / reset internal properties
        end
    end
end
