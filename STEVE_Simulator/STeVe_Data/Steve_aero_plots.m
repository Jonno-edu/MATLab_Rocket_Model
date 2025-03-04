opts = spreadsheetImportOptions("NumVariables", 6);

% Specify sheet and range
opts.Sheet = "Mass Properties";
opts.DataRange = "A2:F12602";

% Specify column names and types
opts.VariableNames = ["Time_s_", "Mass_kg_", "COM_m_", "MOIx_kg_m2_", "MOIy_kg_m2_", "MOIz_kg_m2_"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double"];

% Import the data
STeVeV1NoFins = readtable("/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data/STeVe V1 No Fins.xlsx", opts, "UseExcel", false)
clear opts


opts = spreadsheetImportOptions("NumVariables", 15);

% Specify sheet and range
opts.Sheet = "Aero Properties";
opts.DataRange = "A2:O7501";

% Specify column names and types
opts.VariableNames = ["Mach", "Alpha", "CD", "CDPower_Off", "CDPower_ON", "CAPower_Off", "CAPower_On", "CL", "CN", "CNPotential", "CNViscous", "Cnalpha_0_4deg__perRad_", "CP", "CP_0_4deg_", "Reynolds"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string"];

% Specify variable properties
opts = setvaropts(opts, "Reynolds", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Reynolds", "EmptyFieldRule", "auto");

% Import the data
STeVeV1NoFinsS2 = readtable("/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data/STeVe V1 No Fins.xlsx", opts, "UseExcel", false)


%% Set up the Import Options and import the data
opts = spreadsheetImportOptions("NumVariables", 15);

% Specify sheet and range
opts.Sheet = "Aero Properties 15deg";
opts.DataRange = "A2:O81";

% Specify column names and types
opts.VariableNames = ["Mach", "Alpha", "CD", "CDPower_Off", "CDPower_ON", "CAPower_Off", "CAPower_On", "CL", "CN", "CNPotential", "CNViscous", "Cnalpha_0_4deg__perRad_", "CP", "CP_0_4deg_", "Reynolds"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "categorical"];

% Specify variable properties
opts = setvaropts(opts, "Reynolds", "EmptyFieldRule", "auto");

% Import the data
STeVeV1NoFinsS3 = readtable("/Users/jonno/MATLAB-Drive/Rocket-Model-Simulation/STEVE_Simulator/STeVe_Data/STeVe V1 No Fins.xlsx", opts, "UseExcel", false);

clear opts


plot(STeVeV1NoFins, "Time_s_", "Mass_kg_")
plot(STeVeV1NoFins, "Time_s_", "COM_m_")

% Plot each variable in STeVeV1NoFins against time, all on one plot
figure;
hold on;
plot(STeVeV1NoFins.Time_s_, STeVeV1NoFins.Mass_kg_, 'DisplayName', 'Mass (kg)');
plot(STeVeV1NoFins.Time_s_, STeVeV1NoFins.COM_m_, 'DisplayName', 'COM (m)');

plot(STeVeV1NoFins.Time_s_, STeVeV1NoFins.MOIy_kg_m2_, 'DisplayName', 'MOIy (kg m^2)');
plot(STeVeV1NoFins.Time_s_, STeVeV1NoFins.MOIz_kg_m2_, 'DisplayName', 'MOIz (kg m^2)');
hold off;
xlabel('Time (s)');
ylabel('Values');
title('Variables in STeVeV1NoFins vs Time');
legend;
grid on;

% Plot CD vs Mach vs Alpha for STeVeV1NoFinsS2
figure;
scatter3(STeVeV1NoFinsS2.Mach, STeVeV1NoFinsS2.Alpha, STeVeV1NoFinsS2.CL, 'filled');
xlabel('Mach');
ylabel('Alpha');
zlabel('CD');
title('CD vs Mach vs Alpha (STeVeV1NoFinsS2)');
grid on;
