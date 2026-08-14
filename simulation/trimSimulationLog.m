function log = trimSimulationLog(log, finalIndex)
%TRIMSIMULATIONLOG Remove any unused tail if a custom duration was supplied.

fields = fieldnames(log);
trimmedValues = cellfun(@(name) log.(name)(1:finalIndex, :), fields, ...
    'UniformOutput', false);
log = cell2struct(trimmedValues, fields, 1);
end

