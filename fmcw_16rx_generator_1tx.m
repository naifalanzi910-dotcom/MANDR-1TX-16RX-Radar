%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% By ENG.NAIF ABDULKARIM ALANAZI (Electrical Engineer)
% 16RX FMCW RADAR GENERATOR
%
% Commented version. The processing logic is unchanged; comments were added
% to explain configuration values, data flow, and helper functions.
%
% Overview:
% - Defines radar and target parameters.
% - Simulates complex FMCW beat signals at a 4 x 4 RX array.
% - Adds optional two-way free-space path loss, noise, and RCS scintillation.
% - Sends one preamble + frame per RX channel over ZeroMQ.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; close all; clc;

%% Configuration
cfg.c = 3e8;                                % Speed of light used for range and wavelength calculations.
cfg.fc = 77e9;                              % Radar carrier frequency.
cfg.Fs = 32e6;                              % ADC sampling rate for fast-time samples.
cfg.slope = 15e12;                          % FMCW chirp frequency slope.

cfg.numADCSamp = 1024;                      % Number of ADC samples collected per chirp.
cfg.numChirps = 512;                        % Number of chirps in one radar frame.
cfg.numTx = 1;                              % Number of transmitters; this code supports single-TX operation.

cfg.rxRows = 4;                             % Number of receive antenna rows in the virtual array.
cfg.rxCols = 4;                             % Number of receive antenna columns in the virtual array.
cfg.numRx = cfg.rxRows * cfg.rxCols;        % Total number of receive channels.

cfg.outputBasePort = 5550;                  % Base TCP port; RX channels use base+1 through base+numRx.

cfg.enableRealtimePacing = true;            % Throttle frame generation to real-time speed.
cfg.minFramePeriod = 0.10;                  % Minimum streaming period per frame in seconds.

cfg.enableNoise = true;                     % Add complex Gaussian receiver noise when true.
cfg.SNR_dB = 44;                            % Requested signal-to-noise ratio for generated data.

% Free-space propagation loss. The target amplitude column is interpreted
% as received complex amplitude at pathLossReferenceRange_m. For a monostatic
% radar in free space, received power scales approximately as 1/R^4, so the
% complex-voltage/baseband amplitude scales as 1/R^2.
cfg.enableFreeSpacePathLoss = true;            % Apply two-way free-space path loss when true.
cfg.pathLossReferenceRange_m = 100.0;          % Range where target amplitude is unchanged.
cfg.pathLossMinRange_m = 1.0;                  % Clamp range to avoid singular gain at very small R.

cfg.preambleLength = 256;                   % Length of synchronization preamble sent before each frame.
cfg.preambleAmp = 3;                        % Amplitude of the synchronization preamble.

cfg.seed = 7;                               % Random seed for reproducible target phases and noise.
cfg.randomInitialPhase = true;              % Randomize each target initial phase when true.

cfg.enableRcsScintillation = true;          % Enable slow target amplitude fluctuation.
cfg.rcsScintillationHz = 0.20;              % Frequency of target RCS/amplitude fluctuation.

cfg.minDetectRange_m = 1.0;                 % Minimum valid target range.
cfg.maxDetectRange_m = inf;                 % Maximum valid target range; inf is replaced by radar max range.
cfg.azFovDeg = [-90 90];                    % Allowed azimuth field of view in degrees.
cfg.elFovDeg = [0 90];                      % Allowed elevation field of view in degrees.
cfg.requireInFrontOfRadar = true;           % Reject targets/detections behind the radar when true.

cfg.printEveryNFrames = 1;                  % Print target state every N transmitted frames.

% Target rows are [x0, y0, z0, vx, vy, vz, amplitude, rcs_fluct_dB].
targets = [
   -90, 120, 14,   22,  -4,   0, 1e-3 , 3;
    45,  70, 18,    4,  15,   0, 3e-5 , 5;
];

% Make simulation repeatable by fixing MATLAB random stream.
rng(cfg.seed);

validate_config(cfg);
validate_targets(targets);

% Compute derived radar parameters once from cfg.
p = radar_parameters(cfg);

if isinf(cfg.maxDetectRange_m)
    cfg.maxDetectRange_m = p.maxRange;      % Maximum valid target range; inf is replaced by radar max range.
end

streamFrameTime = max(p.frameTime, cfg.minFramePeriod);

% Precompute target, time, antenna, phase, and noise model values.
model = prepare_model(cfg, p, targets);
preamble = make_preamble(cfg);

samplesPerFrame = cfg.numADCSamp * cfg.numChirps;

fprintf('=== 16RX FMCW Generator: Physical-Time 3D Targets ===\n');
fprintf('Ports      : %d to %d\n', cfg.outputBasePort + 1, cfg.outputBasePort + cfg.numRx);
fprintf('Radar frame: %.6f s\n', p.frameTime);
fprintf('Stream dt  : %.6f s\n', streamFrameTime);
fprintf('Range res  : %.6f m\n', p.rangeRes);
fprintf('Vel res    : %.6f m/s\n', p.velRes);
fprintf('Max range  : %.3f m\n', p.maxRange);
fprintf('Max vel    : %.3f m/s\n\n', p.maxVel);

print_targets(model, cfg, 0);

%% ZMQ output
% Create one PUSH socket per RX channel. Each socket carries one channel stream.
zmq = py.importlib.import_module('zmq');
ctx = zmq.Context();
txSocks = cell(cfg.numRx, 1);

for rxIdx = 1:cfg.numRx
    txSocks{rxIdx} = ctx.socket(zmq.PUSH);
    txSocks{rxIdx}.setsockopt(zmq.SNDHWM, int32(1));
    txSocks{rxIdx}.setsockopt(zmq.LINGER, int32(0));

    address = sprintf('tcp://*:%d', cfg.outputBasePort + rxIdx);
    txSocks{rxIdx}.bind(address);

    fprintf('TX RX%d bound to %s\n', rxIdx, address);
end

fprintf('\nStart processor/GNU Radio first, then press ENTER here.\n');
input('', 's');

%% Stream
% Main loop: generate, serialize, send, optionally pace, and print status.
frameCounter = uint64(0);
streamClock = tic;
nextFrameTime = 0;

fprintf('\nStreaming started. Press CTRL+C to stop.\n\n');

while true
    frameCounter = frameCounter + 1;
    frameTimer = tic;

    frameStartTime = double(frameCounter - 1) * streamFrameTime;

    % Simulate all RX channels for this frame start time.
    rxFrame = generate_frame(cfg, p, model, frameStartTime);
    rxFrameStream = reshape(rxFrame, cfg.numRx, samplesPerFrame);

    for rxIdx = 1:cfg.numRx
        % Prefix each RX frame with the known preamble for receiver synchronization.
        txSamples = [preamble; rxFrameStream(rxIdx, :).'];
        txSocks{rxIdx}.send(py.bytes(complex_to_zmq_bytes(txSamples)));
    end

    if cfg.enableRealtimePacing
        nextFrameTime = nextFrameTime + streamFrameTime;
        pause(max(0, nextFrameTime - toc(streamClock)));
    end

    if mod(double(frameCounter), cfg.printEveryNFrames) == 0
        frameElapsed = toc(frameTimer);
        tMid = frameStartTime + p.frameTime / 2;

        fprintf('\nFrame %d sent | %.6f s | %.2f x stream-time\n', ...
            double(frameCounter), frameElapsed, streamFrameTime / max(frameElapsed, eps));

        print_targets(model, cfg, tMid);
    end
end

%% Functions
% Check basic generator settings before streaming.
function validate_config(cfg)
    if cfg.numTx ~= 1
        error('Only numTx = 1 is supported.');
    end

    if cfg.numRx ~= cfg.rxRows * cfg.rxCols
        error('numRx must equal rxRows * rxCols.');
    end
end

% Verify target table shape and values.
function validate_targets(targets)
    if isempty(targets) || size(targets, 2) ~= 8
        error('targets must be [x0, y0, z0, vx, vy, vz, amplitude, rcs_fluct_dB].');
    end

    if any(~isfinite(targets(:))) || any(targets(:, 7) <= 0)
        error('All target values must be finite and amplitude must be positive.');
    end
end

% Derive radar constants such as wavelength, range resolution, and velocity limits.
function p = radar_parameters(cfg)
    p.lambda = cfg.c / cfg.fc;
    p.Tc = cfg.numADCSamp / cfg.Fs;
    p.PRF = 1 / p.Tc;
    p.BW = cfg.slope * p.Tc;

    p.frameTime = cfg.numChirps * p.Tc;
    p.rangeRes = cfg.c / (2 * p.BW);
    p.maxRange = cfg.c * cfg.Fs / (4 * cfg.slope);
    p.velRes = p.lambda / (2 * cfg.numChirps * p.Tc);
    p.maxVel = p.lambda / (4 * p.Tc);
end

% Convert target table into a simulation model and build RX array geometry.
function model = prepare_model(cfg, p, targets)
    model.pos0 = double(targets(:, 1:3).');
    model.vel0 = double(targets(:, 4:6).');
    model.amp = double(targets(:, 7).');
    model.rcsFluct_dB = double(targets(:, 8).');
    model.numTargets = size(targets, 1);
    model.fastTime = (0:cfg.numADCSamp - 1).' / cfg.Fs;

    [col, row] = meshgrid(0:cfg.rxCols - 1, 0:cfg.rxRows - 1);
    col = col - mean(col(:));
    row = row - mean(row(:));

    model.rxX = col(:) * p.lambda / 2;
    model.rxZ = row(:) * p.lambda / 2;

    if cfg.randomInitialPhase
        model.phase0 = 2 * pi * rand(1, model.numTargets);
    else
        model.phase0 = zeros(1, model.numTargets);
    end

    model.rcsPhase = 2 * pi * rand(1, model.numTargets);

    signalPower = sum(model.amp.^2);
    noisePower = signalPower / 10^(cfg.SNR_dB / 10);
    model.noiseStd = sqrt(noisePower / 2);
end

% Generate one full complex baseband frame for every RX channel.
function rxFrame = generate_frame(cfg, p, model, frameStartTime)
    rxFrame = complex(zeros(cfg.numRx, cfg.numADCSamp, cfg.numChirps, 'single'));

    tFast = model.fastTime;
    tSlow = (0:cfg.numChirps - 1) * p.Tc;
    tAbs = frameStartTime + bsxfun(@plus, tFast, tSlow);
    tMid = frameStartTime + p.frameTime / 2;

    for k = 1:model.numTargets
        [x, y, z, vx, vy, vz] = target_state(model, k, tAbs);

        R = sqrt(x.^2 + y.^2 + z.^2);
        az = atan2d(x, y);
        el = asind(limit_unit(z ./ max(R, eps)));

        visible = R >= cfg.minDetectRange_m & R <= cfg.maxDetectRange_m & ...
                  az >= cfg.azFovDeg(1) & az <= cfg.azFovDeg(2) & ...
                  el >= cfg.elFovDeg(1) & el <= cfg.elFovDeg(2);

        if cfg.requireInFrontOfRadar
            visible = visible & y > 0;
        end

        if ~any(visible(:))
            continue;
        end

        ux = x ./ max(R, eps);
        uz = z ./ max(R, eps);

        tau = 2 * R / cfg.c;

        phaseIF = 2 * pi * ( ...
            cfg.fc .* tau + ...
            cfg.slope .* bsxfun(@times, tau, tFast) - ...
            0.5 * cfg.slope .* tau.^2) + model.phase0(k);

        ampNow = model.amp(k);

        if cfg.enableRcsScintillation
            rcs_dB = model.rcsFluct_dB(k) * ...
                sin(2 * pi * cfg.rcsScintillationHz * tMid + model.rcsPhase(k));
            ampNow = ampNow * 10^(rcs_dB / 20);
        end

        pathLossAmp = free_space_path_loss_amplitude(cfg, R);

        targetSamples = ampNow .* pathLossAmp .* exp(1j * phaseIF);
        targetSamples(~visible) = 0;

        for rxIdx = 1:cfg.numRx
            arrayPhase = exp(1j * 2 * pi / p.lambda * ...
                (model.rxX(rxIdx) .* ux + model.rxZ(rxIdx) .* uz));

            rxFrame(rxIdx, :, :) = rxFrame(rxIdx, :, :) + ...
                single(reshape(targetSamples .* arrayPhase, 1, cfg.numADCSamp, cfg.numChirps));
        end
    end

    if cfg.enableNoise
        rxFrame = rxFrame + single(model.noiseStd * complex( ...
            randn(size(rxFrame)), randn(size(rxFrame))));
    end
end

% Return two-way free-space amplitude scaling relative to a reference range.
function gain = free_space_path_loss_amplitude(cfg, R)
    if ~cfg.enableFreeSpacePathLoss
        gain = ones(size(R));
        return;
    end

    Ruse = max(double(R), double(cfg.pathLossMinRange_m));
    Rref = max(double(cfg.pathLossReferenceRange_m), double(cfg.pathLossMinRange_m));

    % Monostatic radar equation: received power ~ 1/R^4, so complex
    % baseband amplitude ~ sqrt(power) ~ 1/R^2.
    gain = (Rref ./ Ruse).^2;
end

% Evaluate target k position and velocity at time t.
function [x, y, z, vx, vy, vz] = target_state(model, k, t)
    x = model.pos0(1, k) + model.vel0(1, k) .* t;
    y = model.pos0(2, k) + model.vel0(2, k) .* t;
    z = model.pos0(3, k) + model.vel0(3, k) .* t;

    vx = model.vel0(1, k) + zeros(size(t));
    vy = model.vel0(2, k) + zeros(size(t));
    vz = model.vel0(3, k) + zeros(size(t));
end

% Build a Zadoff-Chu-like synchronization preamble.
function preamble = make_preamble(cfg)
    n = single((0:cfg.preambleLength - 1).');
    u = single(25);
    phase = -pi * u * n .* (n + 1) / single(cfg.preambleLength);
    preamble = single(cfg.preambleAmp) .* exp(1j * phase);
end

% Pack complex single samples as interleaved float32 I/Q bytes for ZMQ.
function dataBytes = complex_to_zmq_bytes(x)
    x = x(:);
    iq = zeros(2 * numel(x), 1, 'single');
    iq(1:2:end) = real(x);
    iq(2:2:end) = imag(x);
    dataBytes = uint8(typecast(iq, 'uint8'));
end

% Print target position, radial velocity, angles, and visibility at time t.
function print_targets(model, cfg, t)
    fprintf('Targets at t = %.6f s:\n', t);
    fprintf(' ID      X(m)      Y(m)      Z(m)      Range(m)   RadVel(m/s)   Az(deg)   El(deg)   PLamp(dB)   Visible   Vx      Vy      Vz\n');

    for k = 1:model.numTargets
        [x, y, z, vx, vy, vz] = target_state(model, k, t);
        R = sqrt(x.^2 + y.^2 + z.^2);
        az = atan2d(x, y);
        el = asind(limit_unit(z ./ max(R, eps)));
        vr = (x .* vx + y .* vy + z .* vz) ./ max(R, eps);

        visible = R >= cfg.minDetectRange_m & R <= cfg.maxDetectRange_m & ...
                  az >= cfg.azFovDeg(1) & az <= cfg.azFovDeg(2) & ...
                  el >= cfg.elFovDeg(1) & el <= cfg.elFovDeg(2);

        if cfg.requireInFrontOfRadar
            visible = visible & y > 0;
        end

        pathLossAmp = free_space_path_loss_amplitude(cfg, R);
        pathLossAmp_dB = 20 * log10(pathLossAmp + eps);

        fprintf('%3d %9.3f %9.3f %9.3f %11.3f %13.3f %9.3f %9.3f %11.3f %8d %7.2f %7.2f %7.2f\n', ...
            k, x, y, z, R, vr, az, el, pathLossAmp_dB, visible, vx, vy, vz);
    end

    fprintf('\n');
end

% Clamp values into [-1, 1] before inverse trigonometric functions.
function y = limit_unit(x)
    y = max(-1, min(1, x));
end
