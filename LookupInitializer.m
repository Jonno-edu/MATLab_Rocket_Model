classdef LookupInitializer < matlab.System
    % LookupInitializer reads CSV data and returns interpolated CL based on Mach and α.
    
    properties (Nontunable, Access = private)
        MachData    % Vector of Mach values
        AlphaData   % Vector of α values
        CLData      % Vector of CL values
    end
    
    properties (Nontunable)
        InterpMethod = 'linear';  % Interpolation method
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Mark readtable and scatteredInterpolant as extrinsic if running in interpreted mode.
            coder.extrinsic('readtable','scatteredInterpolant');
            % Load CSV data (this occurs once at simulation start)
            T = readtable('aeroData.csv');  % Ensure your CSV file is in your MATLAB path
            obj.MachData  = T.Mach;
            obj.AlphaData = T.Alpha;
            obj.CLData    = T.CL;
        end
        
        function CL = stepImpl(obj, mach_input, alpha_input)
            % Create an interpolant for scattered data
            coder.extrinsic('scatteredInterpolant');
            F = scatteredInterpolant(obj.MachData, obj.AlphaData, obj.CLData, obj.InterpMethod, 'none');
            CL = F(mach_input, alpha_input);
        end
    end
end
