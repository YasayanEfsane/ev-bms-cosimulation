function drive = generateRaceDriveCycle(cfg)
%GENERATERACEDRIVECYCLE Create a 600 s, 100 Hz high-dynamic race profile.
%   The knots mimic launch, hairpin, chicane and straight sections. PCHIP
%   preserves local monotonicity while avoiding discontinuous speed commands.

time = (0:1/cfg.sampleRate:cfg.duration).';

knotTime = [0 3 8 14 20 27 34 40 48 55 62 70 77 84 92 100 ...
    108 116 124 132 140 148 156 164 172 180 188 196 204 212 ...
    220 228 236 244 252 260 268 276 284 292 300 308 316 324 ...
    332 340 348 356 364 372 380 388 396 404 412 420 428 436 ...
    444 452 460 468 476 484 492 500 508 516 524 532 540 548 ...
    556 564 572 580 588 594 600].';
knotSpeedKph = [0 42 96 145 78 128 55 118 162 88 136 48 112 154 ...
    72 126 38 102 148 65 122 168 82 137 52 116 158 74 131 44 ...
    108 151 61 125 165 79 142 50 114 156 68 129 40 105 147 ...
    58 119 170 84 139 54 110 160 70 127 46 101 152 64 123 ...
    166 76 134 49 115 157 67 128 42 106 149 57 121 163 73 ...
    132 36 82 0].';

speedKph = pchip(knotTime, knotSpeedKph, time);
cornerRipple = 2.8*sin(2*pi*time/13.5) + 1.4*sin(2*pi*time/5.8);
speedKph = speedKph + cornerRipple.*min(max((speedKph-35)/65, 0), 1);
speedKph = min(max(speedKph, 0), cfg.maximumSpeedKph);
speedKph([1, end]) = 0;

speedMps = speedKph/3.6;
accelerationMps2 = gradient(speedMps, 1/cfg.sampleRate);
gradeRad = atan(0.028*sin(2*pi*time/97) + ...
    0.014*sin(2*pi*time/41) + 0.008*sin(2*pi*time/19));
ambientC = cfg.defaultAmbientC + 2.2*sin(2*pi*time/600) ...
    + 0.6*sin(2*pi*time/83);

drive.time = time;
drive.speedMps = speedMps;
drive.speedKph = speedKph;
drive.accelerationMps2 = accelerationMps2;
drive.gradeRad = gradeRad;
drive.ambientC = ambientC;
drive.sampleRate = cfg.sampleRate;
drive.duration = cfg.duration;
end

