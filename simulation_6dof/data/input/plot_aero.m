% STEVE_AeroData_MakeAxisymmetricLateralTables.m
% --- Standalone script: normalize names, create CY/CYbeta and Beta grid, and plot sign checks.

% === Settings
% Set or browse for the aero MAT file
aeroMatFilePath = ''; % leave empty to browse
if isempty(aeroMatFilePath)
    [f,p] = uigetfile({'*.mat','MAT-files'}, 'Select CombinedAeroData MAT');
    if isequal(f,0), error('File selection canceled.'); end
    aeroMatFilePath = fullfile(p,f);
end

% === Load
S = load(aeroMatFilePath);
if isfield(S,'CombinedAeroData')
    AeroData = S.CombinedAeroData;
elseif isfield(S,'AeroData')
    AeroData = S.AeroData;
else
    error('No CombinedAeroData or AeroData struct found in MAT.');
end

% === Normalize table names
T = AeroData.Tables;

% 1) Rename CAPower_On -> CA
if isfield(T,'CAPower_On')
    T.CA = T.CAPower_On;
    T = rmfield(T,'CAPower_On');
end

% 2) Normalize CNalpha naming
if isfield(T,'Cnalpha_0_4deg__perRad_')
    T.CNalpha = T.Cnalpha_0_4deg__perRad_;
    T = rmfield(T,'Cnalpha_0_4deg__perRad_');
end
if isfield(T,'Cnalpha') && ~isfield(T,'CNalpha')
    T.CNalpha = T.Cnalpha;
    T = rmfield(T,'Cnalpha');
end

% Sanity checks for required fields
reqNames = {'CN','CP','CA','CNalpha'};
for k = 1:numel(reqNames)
    assert(isfield(T,reqNames{k}), 'Missing required table: %s', reqNames{k});
end

% === Ensure Beta breakpoints exist (axisymmetric -> copy Alpha grid)
BP = AeroData.Breakpoints;
assert(isfield(BP,'Mach') && isfield(BP,'Alpha'), 'Breakpoints must include Mach and Alpha.');
if ~isfield(BP,'Beta')
    BP.Beta = BP.Alpha;
end

% === Create lateral tables by mirroring longitudinal (axisymmetric)
% CY = CN, CYbeta = CNalpha (numeric mirror; sign checked later with plots)
T.CY     = T.CN;
T.CYbeta = T.CNalpha;

% === Save back into struct, add metadata
AeroData.Tables = T;
AeroData.Breakpoints = BP;
if ~isfield(AeroData,'Metadata'), AeroData.Metadata = struct(); end
AeroData.Metadata.Axisymmetry = struct( ...
    'CY_from_CN', true, ...
    'CYbeta_from_CNalpha', true, ...
    'Note', 'Axisymmetric lateral tables mirrored from longitudinal.' ...
);

% === Optional: write updated MAT
[outDir,outBase,~] = fileparts(aeroMatFilePath);
outFile = fullfile(outDir, [outBase '_WithBeta.mat']);
CombinedAeroData = AeroData; %#ok<NASGU>
save(outFile, 'CombinedAeroData');
fprintf('Saved updated aero database to: %s\n', outFile);

% === Plot sign checks (CA, CN, CNalpha vs alpha; CY, CYbeta vs beta; CP vs alpha)
Mgrid = BP.Mach;
Agrid = BP.Alpha;
Bgrid = BP.Beta;

CA_tab  = T.CA;      % [nM x nA]
CN_tab  = T.CN;      % [nM x nA]
CP_tab  = T.CP;      % [nM x nA]
CNa_tab = T.CNalpha; % [nM x nA]
CY_tab  = T.CY;      % [nM x nB]
CYb_tab = T.CYbeta;  % [nM x nB]

% Units (heuristic)
alpha_units = 'deg'; beta_units = 'deg';
if max(abs(Agrid))<=pi, alpha_units='rad'; end
if max(abs(Bgrid))<=pi, beta_units='rad'; end

% Mach slices to inspect (clamped if not present)
mach_slices = [0.3 0.9 1.5 3.0];
idxM = zeros(size(mach_slices));
for i=1:numel(mach_slices)
    [~,idxM(i)] = min(abs(Mgrid - mach_slices(i)));
end
idxM = unique(max(1, min(idxM, numel(Mgrid))));

% --- CA vs Alpha
figure('Name','CA vs Alpha'); hold on; grid on;
for k = 1:numel(idxM)
    plot(Agrid, CA_tab(idxM(k),:), 'DisplayName', sprintf('M=%.2f', Mgrid(idxM(k))));
end
yline(0,'k:'); xlabel(['\alpha [' alpha_units ']']); ylabel('C_A');
legend('show','Location','best'); title('Axial Coefficient C_A vs \alpha');

% --- CN vs Alpha
figure('Name','CN vs Alpha'); hold on; grid on;
for k = 1:numel(idxM)
    plot(Agrid, CN_tab(idxM(k),:), 'DisplayName', sprintf('M=%.2f', Mgrid(idxM(k))));
end
yline(0,'k:'); xlabel(['\alpha [' alpha_units ']']); ylabel('C_N');
legend('show','Location','best'); title('Normal Coefficient C_N vs \alpha');

% --- CNalpha vs Alpha
figure('Name','CNalpha vs Alpha'); hold on; grid on;
for k = 1:numel(idxM)
    plot(Agrid, CNa_tab(idxM(k),:), 'DisplayName', sprintf('M=%.2f', Mgrid(idxM(k))));
end
yline(0,'k:'); xlabel(['\alpha [' alpha_units ']']); ylabel('C_{N_\alpha} [per rad]');
legend('show','Location','best'); title('Slope C_{N_\alpha} vs \alpha');

% --- CY vs Beta
figure('Name','CY vs Beta'); hold on; grid on;
for k = 1:numel(idxM)
    plot(Bgrid, CY_tab(idxM(k),:), 'DisplayName', sprintf('M=%.2f', Mgrid(idxM(k))));
end
yline(0,'k:'); xlabel(['\beta [' beta_units ']']); ylabel('C_Y');
legend('show','Location','best'); title('Side Coefficient C_Y vs \beta');

% --- CYbeta vs Beta
figure('Name','CYbeta vs Beta'); hold on; grid on;
for k = 1:numel(idxM)
    plot(Bgrid, CYb_tab(idxM(k),:), 'DisplayName', sprintf('M=%.2f', Mgrid(idxM(k))));
end
yline(0,'k:'); xlabel(['\beta [' beta_units ']']); ylabel('C_{Y_\beta} [per rad]');
legend('show','Location','best'); title('Slope C_{Y_\beta} vs \beta');

% --- CP vs Alpha
figure('Name','CP vs Alpha'); hold on; grid on;
for k = 1:numel(idxM)
    plot(Agrid, CP_tab(idxM(k),:), 'DisplayName', sprintf('M=%.2f', Mgrid(idxM(k))));
end
xlabel(['\alpha [' alpha_units ']']); ylabel('C_P (position units)');
legend('show','Location','best'); title('Center of Pressure vs \alpha');

% Console summary
fprintf('Grids: Mach=%d, Alpha=%d, Beta=%d\n', numel(Mgrid), numel(Agrid), numel(Bgrid));
fprintf('Tables present: CA, CN, CNalpha, CY, CYbeta, CP\n');
