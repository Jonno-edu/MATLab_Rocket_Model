classdef GimbalThrustModel < matlab.System
    %  Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        nozzleOffset = -1;
        nozzle_max_angle = (6)*pi/180; % Rad

    end

    % Pre-computed constants or internal states
    properties (Access = private)
    end

    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [thrust_x, thrust_z, moment] = stepImpl(obj,thrust, nozzle_angle)
            thrust_x = thrust * cos(nozzle_angle);
            thrust_z = thrust * sin(nozzle_angle);
            moment = obj.nozzleOffset*thrust_z;
        end

        function resetImpl(obj)
            % Initialize / reset internal properties
        end
    end
end
