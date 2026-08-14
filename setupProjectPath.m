function projectRoot = setupProjectPath()
%SETUPPROJECTPATH Add every project module parent to the MATLAB path.

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));
end

