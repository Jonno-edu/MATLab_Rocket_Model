function statdyn = asbPrepDatcom(datcomfile, varargin)
%% Prepare Aerodynamic Coefficients for Simulink(R) Model
% Import Digital Datcom file and prepare data for use in Simulink(R).

% Copyright 2006-2023 The MathWorks, Inc.

if nargin > 1
    matfile = varargin{1}; %#ok<NASGU>
else
    matfile = 'asbSkyHoggData'; %#ok<NASGU>
end

%% Import Digital Datcom Data
%
statdyn = datcomimport(datcomfile);

%% Filling Out Derivative Tables
%
% It can be seen in the Digital DATCOM output file and examining the
% imported data that
%
% $$ C_{Y\beta},$$
% $$ C_{n\beta},$$
% $$ C_{lq},$$ and
% $$ C_{mq}$$
%
% have data only in the first angle of attack value. By default, missing
% data points are set to 99999.  The missing data points are filled with
% the values for the first angle of attack, since these derivatives are
% independent of angle of attack.

for i = statdyn{1}.nalt:-1:1
    statdyn{1}.cyb(:,:,i) = ones(size(statdyn{1}.cyb,1),1)*statdyn{1}.cyb(1,:,i);
    statdyn{1}.cnb(:,:,i) = ones(size(statdyn{1}.cnb,1),1)*statdyn{1}.cnb(1,:,i);
    statdyn{1}.cmq(:,:,i) = ones(size(statdyn{1}.cmq,1),1)*statdyn{1}.cmq(1,:,i);
    statdyn{1}.clq(:,:,i) = ones(size(statdyn{1}.clq,1),1)*statdyn{1}.clq(1,:,i);
end

%%
% No methods available for some values of dCMelevator.  Zero out values for
% time being.

statdyn{1}.dcm_sym(isnan(statdyn{1}.dcm_sym)) = 0;

%% Save Structure of Datcom Data
%

eval(['save ' matfile ' statdyn']);

end
