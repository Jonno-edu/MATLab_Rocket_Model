function success = snapshotModel(mdl)
% snapshotModel Take a screenshot of the current model and display.
% 
% This is a convenience function for the "Mission Analysis with the Orbit
% Propagator Block" example.

% Copyright 2020 The MathWorks, Inc.

success = true;
try
    figure
    print('-dpng', ['-s' char(mdl)], fullfile(tempdir, 'orbprop_tempcapture.png'))
    imshow(fullfile(tempdir, 'orbprop_tempcapture.png'))
catch
    success = false;
end
end