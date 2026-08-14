function input = sampleDriveCycle(drive, time)
%SAMPLEDRIVECYCLE Fast scalar linear interpolation for the 10 us inner loop.

samplePosition = min(max(time*drive.sampleRate, 0), ...
    numel(drive.time)-1);
lowerZeroIndex = min(floor(samplePosition), numel(drive.time)-2);
lowerIndex = lowerZeroIndex+1;
fraction = samplePosition-lowerZeroIndex;
upperIndex = lowerIndex + 1;

input.targetSpeedMps = drive.speedMps(lowerIndex) + fraction * ...
    (drive.speedMps(upperIndex) - drive.speedMps(lowerIndex));
input.targetAccelerationMps2 = drive.accelerationMps2(lowerIndex) + ...
    fraction * (drive.accelerationMps2(upperIndex) - ...
    drive.accelerationMps2(lowerIndex));
input.gradeRad = drive.gradeRad(lowerIndex) + fraction * ...
    (drive.gradeRad(upperIndex) - drive.gradeRad(lowerIndex));
input.ambientC = drive.ambientC(lowerIndex) + fraction * ...
    (drive.ambientC(upperIndex) - drive.ambientC(lowerIndex));
end
