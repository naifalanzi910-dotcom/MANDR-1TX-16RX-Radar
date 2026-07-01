%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% By ENG.NAIF ABDULKARIM ALANAZI (Electrical Engineer)
% 16RX FMCW RADAR PROCESSOR
%
% Commented version. The processing logic is unchanged; comments were added
% to explain configuration values, data flow, and helper functions.
%
% Overview:
% - Receives synchronized RX-channel streams over ZeroMQ.
% - Performs range FFT, Doppler FFT, CFAR detection, and MUSIC angle estimation.
% - Filters/merges detections and tracks targets using an EKF.
% - Updates range-Doppler, detection, angle, 3D, and tracking-error plots.
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

% Generator sends 5551..5566.
% Use 5550 for direct generator-to-processor.
% Use 6000 only if GNU Radio relays 555x to 600x.
cfg.inputBasePort = 6000;                   % Base TCP input port; processor listens on base+1 through base+numRx.
cfg.rxPermutation = 1:16;                   % Optional RX channel reordering after frame reception.

cfg.preambleLength = 256;                   % Length of synchronization preamble sent before each frame.
cfg.preambleAmp = 3;                        % Amplitude of the synchronization preamble.
cfg.preambleCorrMin = 0.88;                 % Minimum normalized preamble-correlation score for sync lock.
cfg.maxBufferedFrames = 3;                  % Maximum per-channel FIFO history kept while searching for sync.

cfg.dopplerZPfactor = 2;                    % Doppler FFT zero-padding factor.
cfg.historyLength = 350;                    % Number of past frames retained for tracks/errors.

cfg.azGridDeg = single(-90:1:90);           % Azimuth grid searched by MUSIC.
cfg.elGridDeg = single(0:1:90);             % Elevation grid searched by MUSIC.

cfg.windowType = 'blackman-harris';         % FFT sidelobe-control window type.

cfg.enableAdcDcRemoval = true;              % Remove per-chirp ADC DC offset before range FFT.
cfg.enableStaticClutterRemoval = true;      % Subtract mean across chirps to suppress static clutter.
cfg.enableZeroDopplerNotch = true;          % Zero bins around Doppler DC after FFT.
cfg.zeroDopplerNotchBins = 3;               % Half-width of zero-Doppler notch in bins.

cfg.enableRdBackgroundRemoval = true;       % Remove estimated range/Doppler background floor.
cfg.rdBackgroundScale = 0.92;               % Scale factor applied to the estimated RD background.
cfg.enableRdDisplayCompression = true;      % Apply gamma compression for RD display only.
cfg.rdDisplayGamma = 0.60;                  % Gamma used for display compression.

%% Detection tuning
% These thresholds control how strict CFAR/local-peak filtering is.
cfg.cfarTrainRange = 24;                    % CFAR training cells on each range side.
cfg.cfarTrainDoppler = 30;                  % CFAR training cells on each Doppler side.
cfg.cfarGuardRange = 7;                     % CFAR guard cells on each range side.
cfg.cfarGuardDoppler = 9;                   % CFAR guard cells on each Doppler side.
cfg.cfarPfa = 1e-8;                         % Desired false-alarm probability for CA-CFAR.
cfg.cfarMinThreshold_dB = -80;              % Absolute minimum CFAR threshold in dB.

cfg.minPeakSNR_dB = 14;                     % Minimum peak SNR required for a detection.
cfg.minDetectionPower_dB = -45;             % Minimum normalized detection power in dB.
cfg.minRange_m = 2.0;                       % Ignore very close range bins below this distance.
cfg.minAbsVelocity_mps = 0.20;              % Reject near-zero velocity candidates.

cfg.localMaxRangeBins = 5;                  % Range-window size for local-maximum detection.
cfg.localMaxDopplerBins = 7;                % Doppler-window size for local-maximum detection.

cfg.clusterRangeBins = 7;                   % Range span used to cluster nearby CFAR candidates.
cfg.clusterDopplerBins = 11;                % Doppler span used to cluster nearby CFAR candidates.

cfg.maxRawDetections = 24;                  % Maximum raw detections kept before validation.
cfg.maxValidatedDetections = 10;            % Maximum validated detections passed to tracker.

%% MUSIC
% Parameters used to estimate azimuth/elevation from RX-array covariance.
cfg.musicNumSources = 1;                    % Number of signal sources assumed by MUSIC.
cfg.musicDiagonalLoading = 3e-4;            % Diagonal loading added to covariance for robustness.
cfg.angleRangeSpan = 1;                     % Range-bin neighborhood used for angle covariance.
cfg.angleDopplerSpan = 1;                   % Doppler-bin neighborhood used for angle covariance.
cfg.minAngleQuality_dB = 0.20;              % Minimum MUSIC peak separation/quality.

%% Radar visibility
cfg.minDetectRange_m = 1.0;                 % Minimum valid target range.
cfg.maxDetectRange_m = inf;                 % Maximum valid target range; inf is replaced by radar max range.
cfg.azFovDeg = [-90 90];                    % Allowed azimuth field of view in degrees.
cfg.elFovDeg = [0 90];                      % Allowed elevation field of view in degrees.
cfg.requireInFrontOfRadar = true;           % Reject targets/detections behind the radar when true.

%% Detection NMS and ghost suppression
cfg.nmsRangeGate_m = 5.0;                   % NMS range gate for duplicate suppression.
cfg.nmsVelocityGate_mps = 3.0;              % NMS velocity gate for duplicate suppression.
cfg.nmsAzGate_deg = 12.0;                   % NMS azimuth gate for duplicate suppression.
cfg.nmsElGate_deg = 12.0;                   % NMS elevation gate for duplicate suppression.

cfg.enableGhostSuppression = true;          % Suppress weaker detections that look like same-target ghosts.
cfg.ghostRangeGate_m = 5.0;                 % Ghost-suppression range gate.
cfg.ghostVelocityGate_mps = 3.0;            % Ghost-suppression velocity gate.
cfg.ghostAzGate_deg = 12.0;                 % Ghost-suppression azimuth gate.
cfg.ghostElGate_deg = 12.0;                 % Ghost-suppression elevation gate.
cfg.ghostPowerDrop_dB = 7.0;                % Power drop required before removing a likely ghost.

%% EKF tracker
% Extended Kalman Filter settings for target tracking across frames.
cfg.trackerDt = 0.10;                  % MUST match generator physical frame period

cfg.maxTracks = 20;                         % Maximum simultaneous EKF tracks.
cfg.trackConfirmHits = 4;                   % Hits required within confirmation window.
cfg.trackConfirmWindow = 6;                 % Length of the hit/miss confirmation window.
cfg.trackMaxMisses = 8;                     % Delete tracks after this many missed updates.
cfg.deleteUnconfirmedAfterMisses = 1;       % Delete tentative tracks quickly after misses.

cfg.showTentativeTracks = false;            % Display unconfirmed tracks when true.
cfg.showCoastedTracks = true;               % Display tracks during missed/coasted frames when true.

cfg.processAccelStd_mps2 = 3.0;             % Process-noise acceleration standard deviation.

cfg.trackInitPosStd_m = 2.5;                % Initial track position uncertainty.
cfg.trackInitVelStd_mps = 18.0;             % Initial track velocity uncertainty.

cfg.measRangeStd_m = 0.9;                   % Measurement standard deviation for range.
cfg.measAzStd_deg = 1.8;                    % Measurement standard deviation for azimuth.
cfg.measElStd_deg = 1.8;                    % Measurement standard deviation for elevation.
cfg.measRadialVelStd_mps = 1.3;             % Measurement standard deviation for radial velocity.

cfg.gateNormalizedInnovation2 = 18.47;      % 4D chi-square around 99.9%
cfg.associationMaxCost = 18.47;             % Maximum assignment cost accepted for track association.

cfg.spawnSuppressRange_m = 10.0;            % Do not spawn a new track near an existing track within this range.
cfg.spawnSuppressAz_deg = 12.0;             % Spawn-suppression azimuth gate.
cfg.spawnSuppressEl_deg = 12.0;             % Spawn-suppression elevation gate.
cfg.spawnSuppressVel_mps = 5.0;             % Spawn-suppression velocity gate.

cfg.mergeRange_m = 9.0;                     % Range gate for merging duplicate tracks.
cfg.mergeAz_deg = 12.0;                     % Azimuth gate for merging duplicate tracks.
cfg.mergeEl_deg = 12.0;                     % Elevation gate for merging duplicate tracks.
cfg.mergeVel_mps = 5.0;                     % Velocity gate for merging duplicate tracks.

%% Plotting
% Visualization limits, marker behavior, and debug printing.
cfg.showRadarCoverageVolume = true;         % Show radar FOV/range coverage volume in 3D plot.
cfg.coverageAzSamples = 61;                 % Azimuth mesh resolution for coverage volume.
cfg.coverageElSamples = 31;                 % Elevation mesh resolution for coverage volume.
cfg.coverageFaceAlpha = 0.07;               % Transparency of coverage volume faces.
cfg.coverageEdgeAlpha = 0.10;               % Transparency of coverage volume edges.

cfg.plotAllAngleSpectra = true;             % Plot all MUSIC spectra for visible detections.
cfg.plotAngleMarkers = true;                % Mark estimated MUSIC peaks on angle plots.

cfg.arrowScale = 0.80;                      % Velocity-arrow scale factor in the 3D plot.
cfg.arrowMinLength_m = 6.0;                 % Minimum displayed velocity-arrow length.
cfg.arrowMaxLength_m = 18.0;                % Maximum displayed velocity-arrow length.

cfg.rdDisplayFloor_dB = -90;                % Lower dB floor for range-Doppler display.
cfg.cfarDisplayFloor_dB = -90;              % Lower dB floor for filtered detection display.

cfg.plotXLim = [-180 180];                  % X-axis limits for 3D plot.
cfg.plotYLim = [0 230];                     % Y-axis limits for 3D plot.
cfg.plotZLim = [0 120];                     % Z-axis limits for 3D plot.

cfg.debugPrint = true;                      % Print per-frame diagnostics when true.

% Validate configuration before opening sockets or allocating large arrays.
validate_config(cfg);

% Compute wavelength, chirp period, range resolution, and velocity limits.
p = radar_parameters(cfg);

if isinf(cfg.maxDetectRange_m)
    cfg.maxDetectRange_m = p.maxRange;      % Maximum valid target range; inf is replaced by radar max range.
end

samplesPerFrame = cfg.numADCSamp * cfg.numChirps;

fprintf('=== 16RX FMCW Processor: CFAR + MUSIC + Radar EKF Tracking ===\n');
fprintf('Ports      : %d to %d\n', cfg.inputBasePort + 1, cfg.inputBasePort + cfg.numRx);
fprintf('Radar frame: %.6f s\n', p.frameTime);
fprintf('Tracker dt : %.6f s\n', cfg.trackerDt);
fprintf('Range res  : %.6f m\n', p.rangeRes);
fprintf('Vel res    : %.6f m/s\n', p.velRes);
fprintf('Max range  : %.3f m\n', p.maxRange);
fprintf('Max vel    : %.3f m/s\n\n', p.maxVel);

%% ZMQ input
% Create one PULL socket per RX stream. The generator or GNU Radio relay must already be publishing.
zmq = py.importlib.import_module('zmq');
ctx = zmq.Context();
rxSocks = cell(cfg.numRx, 1);

for rxIdx = 1:cfg.numRx
    rxSocks{rxIdx} = ctx.socket(zmq.PULL);
    rxSocks{rxIdx}.setsockopt(zmq.RCVHWM, int32(1));
    rxSocks{rxIdx}.setsockopt(zmq.LINGER, int32(0));

    address = sprintf('tcp://127.0.0.1:%d', cfg.inputBasePort + rxIdx);
    rxSocks{rxIdx}.connect(address);

    fprintf('RX%d connected to %s\n', rxIdx, address);
end

fprintf('\nWaiting for synchronized stream...\n\n');

%% Processing setup
% Precompute windows, axes, masks, steering vectors, CFAR kernel, and plot handles.
rangeWin = strong_window(cfg.numADCSamp, cfg.windowType);
dopplerWin = strong_window(cfg.numChirps, cfg.windowType);

rangeWin = rangeWin / sqrt(mean(rangeWin.^2));
dopplerWin = dopplerWin / sqrt(mean(dopplerWin.^2));

rangeWin3 = reshape(single(rangeWin.'), 1, [], 1);
dopplerWin3 = reshape(single(dopplerWin), 1, 1, []);

N_doppler = cfg.numChirps * cfg.dopplerZPfactor;

rangeAxis = single((0:cfg.numADCSamp - 1) * p.rangeRes);
rangeMask = rangeAxis <= p.maxRange;

dopplerFreqAxis = single((-N_doppler / 2:N_doppler / 2 - 1) * (p.PRF / N_doppler));
dopplerAxis = single(dopplerFreqAxis * p.lambda / 2);

[angle, A] = angle_setup(cfg, p);
cfar = cfar_setup(cfg, rangeMask, N_doppler);
preamble = make_preamble(cfg);

rxFifo = cell(cfg.numRx, 1);
for rxIdx = 1:cfg.numRx
    rxFifo{rxIdx} = complex(zeros(0, 1, 'single'));
end

tracks = init_track_list();
nextTrackId = 1;
history = init_history(cfg);

h = setup_plots(rangeAxis, dopplerAxis, rangeMask, angle, N_doppler, cfg, p);

%% Stream loop
% Main real-time loop: receive, process, detect, estimate angles, track, and display.
frameCounter = 0;

while true
    frameCounter = frameCounter + 1;

    % Pull one preamble-aligned frame from every RX channel.
    [rxFrameStream, rxFifo, syncMetric] = receive_synced_frame(rxSocks, rxFifo, preamble, cfg, samplesPerFrame);

    frameBuffer = reshape(rxFrameStream, cfg.numRx, cfg.numADCSamp, cfg.numChirps);
    frameBuffer = frameBuffer(cfg.rxPermutation, :, :);

    inputPower = mean(abs(frameBuffer(:)).^2);

    % Convert raw samples into a normalized range-Doppler representation.
    [RDcube, powerRaw, powerRD] = range_doppler_process(frameBuffer, rangeWin3, dopplerWin3, N_doppler, cfg);
    % Find statistically significant peaks using 2D CA-CFAR.
    [cfarMap, thresholdMap, snrMap] = ca_cfar_2d(powerRD, cfar);

    % Convert CFAR candidates into range/velocity detection records.
    rawDetections = detect_objects(powerRD, cfarMap, thresholdMap, snrMap, rangeAxis, dopplerAxis, cfg);

    for i = 1:numel(rawDetections)
        Rxx = local_covariance(RDcube, rawDetections(i).rangeBin, rawDetections(i).dopplerBin, cfg);

        [rawDetections(i).azimuth, rawDetections(i).elevation, ...
         rawDetections(i).azSpec_dB, rawDetections(i).elSpec_dB, ...
         rawDetections(i).angleQuality_dB] = estimate_az_el(Rxx, A, angle, cfg);
    end

    detections = validate_detections(rawDetections, cfg);

    if cfg.enableGhostSuppression
        detections = suppress_same_target_ghosts(detections, cfg);
    end

    detections = detection_nms_4d(detections, cfg);
    detections = sort_detections(detections, cfg.maxValidatedDetections);

    % Update EKF tracks using the validated detections.
    [tracks, nextTrackId] = ekf_track_manager(tracks, nextTrackId, detections, cfg);
    tracks = merge_duplicate_tracks(tracks, cfg);

    displayDetections = tracks_to_detections(tracks, cfg);

    cleanCfarMap = final_detection_map(size(powerRD), detections, cfg);
    history = update_history(history, displayDetections, cfg);

    if cfg.debugPrint
        fprintf('Frame %d | Input %.3e | Sync %.3f | CFAR %d | Raw %d | Valid %d | Tracks shown %d\n', ...
            frameCounter, inputPower, min(syncMetric), nnz(cfarMap), ...
            numel(rawDetections), numel(detections), numel(displayDetections));
        print_detections(displayDetections);
    end

    % Refresh all plots using current detections and tracks.
    h = update_plots(h, frameCounter, powerRD, cleanCfarMap, rangeMask, ...
        detections, displayDetections, tracks, angle, history, cfg);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Check processor settings and RX permutation validity.
function validate_config(cfg)
    if cfg.numTx ~= 1
        error('Only numTx = 1 is supported.');
    end

    if cfg.numRx ~= cfg.rxRows * cfg.rxCols
        error('numRx must equal rxRows * rxCols.');
    end

    if numel(cfg.rxPermutation) ~= cfg.numRx || any(sort(cfg.rxPermutation) ~= 1:cfg.numRx)
        error('rxPermutation must contain all RX indices exactly once.');
    end
end

% Derive radar constants from FMCW configuration.
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

% Generate a high-sidelobe-suppression FFT window.
function w = strong_window(N, typeName)
    n = (0:N - 1).';

    switch lower(typeName)
        case 'blackman-harris'
            w = 0.35875 ...
                - 0.48829 * cos(2 * pi * n / (N - 1)) ...
                + 0.14128 * cos(4 * pi * n / (N - 1)) ...
                - 0.01168 * cos(6 * pi * n / (N - 1));
        case 'nuttall'
            w = 0.355768 ...
                - 0.487396 * cos(2 * pi * n / (N - 1)) ...
                + 0.144232 * cos(4 * pi * n / (N - 1)) ...
                - 0.012604 * cos(6 * pi * n / (N - 1));
        otherwise
            error('Unknown window type.');
    end

    w = single(w(:));
end

% Build the 2D steering-vector dictionary used by MUSIC.
function [angle, A] = angle_setup(cfg, p)
    [col, row] = meshgrid(0:cfg.rxCols - 1, 0:cfg.rxRows - 1);

    col = col - mean(col(:));
    row = row - mean(row(:));

    rxX = single(col(:) * p.lambda / 2);
    rxZ = single(row(:) * p.lambda / 2);

    [AZ, EL] = meshgrid(cfg.azGridDeg, cfg.elGridDeg);

    angle.azGrid = cfg.azGridDeg(:).';
    angle.elGrid = cfg.elGridDeg(:).';

    ux = cosd(EL(:).') .* sind(AZ(:).');
    uz = sind(EL(:).');

    A.full = single(exp(1j * 2 * pi / p.lambda * (rxX * ux + rxZ * uz)));
end

% Prepare CA-CFAR kernel, scaling factor, valid mask, and border sizes.
function cfar = cfar_setup(cfg, rangeMask, N_doppler)
    Tr = cfg.cfarTrainRange;
    Td = cfg.cfarTrainDoppler;
    Gr = cfg.cfarGuardRange;
    Gd = cfg.cfarGuardDoppler;

    kernel = ones(2 * (Tr + Gr) + 1, 2 * (Td + Gd) + 1, 'single');
    kernel(Tr + 1:Tr + 2 * Gr + 1, Td + 1:Td + 2 * Gd + 1) = 0;

    cfar.kernel = kernel;
    cfar.numTrain = sum(kernel(:));
    cfar.alpha = single(cfar.numTrain * (cfg.cfarPfa^(-1 / cfar.numTrain) - 1));
    cfar.minThreshold = single(10^(cfg.cfarMinThreshold_dB / 10));
    cfar.validMask = repmat(rangeMask(:), 1, N_doppler);
    cfar.borderR = Tr + Gr;
    cfar.borderD = Td + Gd;
end

% Build the same synchronization preamble used by the generator.
function preamble = make_preamble(cfg)
    n = single((0:cfg.preambleLength - 1).');
    u = single(25);
    phase = -pi * u * n .* (n + 1) / single(cfg.preambleLength);
    preamble = single(cfg.preambleAmp) .* exp(1j * phase);
end

% Receive one synchronized frame from all RX ZMQ streams.
function [rxFrameStream, rxFifo, syncMetric] = receive_synced_frame(rxSocks, rxFifo, preamble, cfg, samplesPerFrame)
    rxFrameStream = complex(zeros(cfg.numRx, samplesPerFrame, 'single'));
    syncMetric = zeros(cfg.numRx, 1, 'single');

    for rxIdx = 1:cfg.numRx
        while true
            [idx, metric] = find_preamble_index(rxFifo{rxIdx}, preamble, cfg);

            if ~isempty(idx)
                frameStart = idx + numel(preamble);
                frameEnd = frameStart + samplesPerFrame - 1;

                while numel(rxFifo{rxIdx}) < frameEnd
                    rxFifo{rxIdx} = [rxFifo{rxIdx}; zmq_bytes_to_complex(rxSocks{rxIdx}.recv())]; %#ok<AGROW>
                end

                rxFrameStream(rxIdx, :) = rxFifo{rxIdx}(frameStart:frameEnd).';
                rxFifo{rxIdx}(1:frameEnd) = [];
                syncMetric(rxIdx) = metric;
                break;
            end

            rxFifo{rxIdx} = [rxFifo{rxIdx}; zmq_bytes_to_complex(rxSocks{rxIdx}.recv())]; %#ok<AGROW>

            maxKeep = cfg.maxBufferedFrames * (samplesPerFrame + numel(preamble));
            if numel(rxFifo{rxIdx}) > maxKeep
                rxFifo{rxIdx} = rxFifo{rxIdx}(end - maxKeep + 1:end);
            end
        end
    end
end

% Find the start of a frame by normalized preamble correlation.
function [idx, peakMetric] = find_preamble_index(x, preamble, cfg)
    idx = [];
    peakMetric = single(0);
    L = numel(preamble);

    if numel(x) < L
        return;
    end

    c = abs(conv(x, flipud(conj(preamble)), 'valid'));
    preEnergy = sum(abs(preamble).^2);
    xEnergy = conv(abs(x).^2, ones(L, 1, 'single'), 'valid');
    metric = c ./ sqrt(preEnergy .* xEnergy + eps('single'));

    [peakMetric, peakIdx] = max(metric);

    if peakMetric >= cfg.preambleCorrMin
        idx = peakIdx;
    end
end

% Convert time-domain RX data into range-Doppler power maps.
function [RDcube, powerRaw, powerRD] = range_doppler_process(X, rangeWin3, dopplerWin3, N_doppler, cfg)
    if cfg.enableAdcDcRemoval
        X = X - mean(X, 2);
    end

    Xr = fft(X .* rangeWin3, [], 2);

    if cfg.enableStaticClutterRemoval
        Xr = Xr - mean(Xr, 3);
    end

    RDcube = fftshift(fft(Xr .* dopplerWin3, N_doppler, 3), 3);

    if cfg.enableZeroDopplerNotch
        dc = floor(N_doppler / 2) + 1;
        b = cfg.zeroDopplerNotchBins;
        RDcube(:, :, max(1, dc - b):min(N_doppler, dc + b)) = 0;
    end

    powerRaw = squeeze(sum(abs(RDcube).^2, 1));
    powerRaw = powerRaw ./ (max(powerRaw(:)) + eps('single'));

    P = powerRaw;

    if cfg.enableRdBackgroundRemoval
        rangeFloor = median(P, 2);
        dopplerFloor = median(P, 1);
        background = max(rangeFloor, dopplerFloor);
        P = max(P - cfg.rdBackgroundScale .* background, 0);
    end

    powerRD = P ./ (max(P(:)) + eps('single'));
end

% Apply 2D cell-averaging CFAR to the range-Doppler map.
function [cfarMap, thresholdMap, snrMap] = ca_cfar_2d(powerMap, cfar)
    noiseMean = conv2(single(powerMap), cfar.kernel, 'same') ./ max(cfar.numTrain, 1);
    thresholdMap = max(cfar.alpha .* noiseMean, cfar.minThreshold);

    cfarMap = powerMap > thresholdMap;

    cfarMap(1:cfar.borderR, :) = false;
    cfarMap(end - cfar.borderR + 1:end, :) = false;
    cfarMap(:, 1:cfar.borderD) = false;
    cfarMap(:, end - cfar.borderD + 1:end) = false;
    cfarMap(~cfar.validMask) = false;

    snrMap = powerMap ./ (noiseMean + eps('single'));
end

% Turn CFAR peaks into raw range/velocity detections.
function detections = detect_objects(powerMap, cfarMap, thresholdMap, snrMap, rangeAxis, dopplerAxis, cfg)
    minSnrLinear = single(10^(cfg.minPeakSNR_dB / 10));
    minPowerLinear = single(10^(cfg.minDetectionPower_dB / 10));

    candidateMap = cfarMap & ...
        snrMap >= minSnrLinear & ...
        powerMap >= minPowerLinear & ...
        local_maxima_2d(powerMap, cfg.localMaxRangeBins, cfg.localMaxDopplerBins);

    candidateMap(rangeAxis(:) < cfg.minRange_m, :) = false;
    candidateMap(:, abs(double(dopplerAxis)) < cfg.minAbsVelocity_mps) = false;

    mergeKernel = ones(cfg.clusterRangeBins, cfg.clusterDopplerBins, 'single');
    objectMap = conv2(single(candidateMap), mergeKernel, 'same') > 0;

    labels = connected_components_2d(objectMap);
    numLabels = double(max(labels(:)));

    detections = empty_detection_struct();

    for id = 1:numLabels
        region = labels == id;
        validRegion = region & candidateMap;

        if nnz(validRegion) < 1
            continue;
        end

        regionPower = powerMap;
        regionPower(~validRegion) = 0;

        [peakValue, peakIdx] = max(regionPower(:));

        if peakValue <= 0
            continue;
        end

        [rBin, dBin] = ind2sub(size(powerMap), peakIdx);

        if peakValue <= thresholdMap(rBin, dBin)
            continue;
        end

        detections(end + 1) = make_detection(powerMap, snrMap, rangeAxis, dopplerAxis, rBin, dBin, peakValue); %#ok<AGROW>
    end

    detections = sort_detections(detections, cfg.maxRawDetections);
end

% Mark samples that are local maxima within a rectangular neighborhood.
function localMaxMap = local_maxima_2d(M, rWin, dWin)
    [nr, nd] = size(M);
    rRad = floor(rWin / 2);
    dRad = floor(dWin / 2);

    P = -inf(nr + 2 * rRad, nd + 2 * dRad, 'single');
    P(1 + rRad:rRad + nr, 1 + dRad:dRad + nd) = M;

    N = -inf(nr, nd, 'single');

    for rr = 1:2 * rRad + 1
        for dd = 1:2 * dRad + 1
            N = max(N, P(rr:rr + nr - 1, dd:dd + nd - 1));
        end
    end

    localMaxMap = M >= N & M > 0;
end

% Label connected candidate regions using an 8-neighbor flood fill.
function labels = connected_components_2d(binaryMap)
    [numR, numD] = size(binaryMap);
    labels = zeros(numR, numD, 'uint32');
    currentLabel = uint32(0);

    qR = zeros(numel(binaryMap), 1, 'uint32');
    qD = zeros(numel(binaryMap), 1, 'uint32');

    for r0 = 1:numR
        for d0 = 1:numD
            if ~binaryMap(r0, d0) || labels(r0, d0) ~= 0
                continue;
            end

            currentLabel = currentLabel + 1;
            head = 1;
            tail = 1;

            qR(tail) = uint32(r0);
            qD(tail) = uint32(d0);
            labels(r0, d0) = currentLabel;

            while head <= tail
                r = double(qR(head));
                d = double(qD(head));
                head = head + 1;

                for rr = max(1, r - 1):min(numR, r + 1)
                    for dd = max(1, d - 1):min(numD, d + 1)
                        if binaryMap(rr, dd) && labels(rr, dd) == 0
                            tail = tail + 1;
                            qR(tail) = uint32(rr);
                            qD(tail) = uint32(dd);
                            labels(rr, dd) = currentLabel;
                        end
                    end
                end
            end
        end
    end
end

% Estimate local RX covariance around a range-Doppler detection.
function Rxx = local_covariance(RDcube, rBin, dBin, cfg)
    [numRx, numRange, numDoppler] = size(RDcube);

    r1 = max(1, rBin - cfg.angleRangeSpan);
    r2 = min(numRange, rBin + cfg.angleRangeSpan);
    d1 = max(1, dBin - cfg.angleDopplerSpan);
    d2 = min(numDoppler, dBin + cfg.angleDopplerSpan);

    Xloc = reshape(RDcube(:, r1:r2, d1:d2), numRx, []);

    colPower = sum(abs(Xloc).^2, 1);
    keep = colPower > max(colPower) * 1e-3;

    if any(keep)
        Xloc = Xloc(:, keep);
    end

    Rxx = Xloc * Xloc' / max(size(Xloc, 2), 1);
    Rxx = Rxx ./ (real(trace(Rxx)) / numRx + eps('single'));
    Rxx = 0.5 * (Rxx + Rxx');
end

% Estimate azimuth/elevation using the MUSIC pseudospectrum.
function [azEst, elEst, azSpec_dB, elSpec_dB, quality_dB] = estimate_az_el(Rxx, A, angle, cfg)
    numRx = size(Rxx, 1);

    Rxx = 0.5 * (Rxx + Rxx');
    Rxx = Rxx + cfg.musicDiagonalLoading * eye(numRx, 'single');

    [V, D] = eig(double(Rxx), 'vector');
    [~, order] = sort(real(D), 'descend');
    V = V(:, order);

    numSig = max(1, min(round(double(cfg.musicNumSources)), numRx - 1));
    En = V(:, numSig + 1:end);

    Agrid = double(A.full);
    noiseProjection = En' * Agrid;
    denom = sum(abs(noiseProjection).^2, 1);

    spectrum = 1 ./ max(denom, eps);
    spectrum = single(abs(spectrum));
    spectrum = spectrum ./ (max(spectrum) + eps('single'));

    S2 = reshape(spectrum, numel(angle.elGrid), numel(angle.azGrid));

    [~, idx] = max(S2(:));
    [elIdx, azIdx] = ind2sub(size(S2), idx);

    azEst = angle.azGrid(azIdx);
    elEst = angle.elGrid(elIdx);

    azSpec = max(S2, [], 1);
    elSpec = max(S2, [], 2);

    azSpec_dB = 10 * log10(azSpec(:) ./ (max(azSpec) + eps('single')) + eps('single'));
    elSpec_dB = 10 * log10(elSpec(:) ./ (max(elSpec) + eps('single')) + eps('single'));

    sortedSpec = sort(S2(:), 'descend');

    if numel(sortedSpec) >= 2
        quality_dB = 10 * log10((sortedSpec(1) + eps('single')) / (sortedSpec(2) + eps('single')));
    else
        quality_dB = single(0);
    end
end

% Reject detections outside angle/range/FOV and quality limits.
function detections = validate_detections(detections, cfg)
    if isempty(detections)
        return;
    end

    keep = true(1, numel(detections));

    for i = 1:numel(detections)
        d = detections(i);

        if ~isfinite(d.range) || ~isfinite(d.velocity) || ...
           ~isfinite(d.azimuth) || ~isfinite(d.elevation)
            keep(i) = false;
            continue;
        end

        if d.angleQuality_dB < cfg.minAngleQuality_dB
            keep(i) = false;
            continue;
        end

        if d.range < cfg.minDetectRange_m || d.range > cfg.maxDetectRange_m
            keep(i) = false;
            continue;
        end

        if d.azimuth < cfg.azFovDeg(1) || d.azimuth > cfg.azFovDeg(2)
            keep(i) = false;
            continue;
        end

        if d.elevation < cfg.elFovDeg(1) || d.elevation > cfg.elFovDeg(2)
            keep(i) = false;
            continue;
        end

        [~, y, ~] = sph_to_xyz(double(d.range), double(d.azimuth), double(d.elevation));

        if cfg.requireInFrontOfRadar && y <= 0
            keep(i) = false;
        end
    end

    detections = detections(keep);
end

% Remove weaker detections likely caused by the same physical target.
function detections = suppress_same_target_ghosts(detections, cfg)
    if isempty(detections)
        return;
    end

    [~, order] = sort([detections.score], 'descend');
    detections = detections(order);

    keep = true(1, numel(detections));

    for i = 1:numel(detections)
        if ~keep(i)
            continue;
        end

        for j = i + 1:numel(detections)
            sameRange = abs(detections(j).range - detections(i).range) <= cfg.ghostRangeGate_m;
            sameVel = abs(detections(j).velocity - detections(i).velocity) <= cfg.ghostVelocityGate_mps;
            sameAz = abs(wrap_angle_deg(detections(j).azimuth - detections(i).azimuth)) <= cfg.ghostAzGate_deg;
            sameEl = abs(detections(j).elevation - detections(i).elevation) <= cfg.ghostElGate_deg;
            weaker = detections(j).power_dB <= detections(i).power_dB - cfg.ghostPowerDrop_dB;

            if sameRange && sameVel && sameAz && sameEl && weaker
                keep(j) = false;
            end
        end
    end

    detections = detections(keep);
end

% Non-maximum suppression in range, velocity, azimuth, and elevation.
function detections = detection_nms_4d(detections, cfg)
    if isempty(detections)
        return;
    end

    [~, order] = sort([detections.score], 'descend');
    detections = detections(order);

    keep = true(1, numel(detections));

    for i = 1:numel(detections)
        if ~keep(i)
            continue;
        end

        for j = i + 1:numel(detections)
            closeR = abs(detections(i).range - detections(j).range) <= cfg.nmsRangeGate_m;
            closeV = abs(detections(i).velocity - detections(j).velocity) <= cfg.nmsVelocityGate_mps;
            closeA = abs(wrap_angle_deg(detections(i).azimuth - detections(j).azimuth)) <= cfg.nmsAzGate_deg;
            closeE = abs(detections(i).elevation - detections(j).elevation) <= cfg.nmsElGate_deg;

            if closeR && closeV && closeA && closeE
                keep(j) = false;
            end
        end
    end

    detections = detections(keep);
end

% Create one detection struct and refine range/velocity estimates.
function det = make_detection(powerMap, snrMap, rangeAxis, dopplerAxis, rBin, dBin, peakValue)
    det.trackId = 0;
    det.trackAge = 0;
    det.trackMisses = 0;
    det.confirmed = false;
    det.coasted = false;
    det.insideFov = true;

    det.range = refine_axis(powerMap(:, dBin), rangeAxis, rBin);
    det.velocity = refine_axis(powerMap(rBin, :), dopplerAxis, dBin);

    det.azimuth = NaN;
    det.elevation = NaN;

    det.power_dB = 10 * log10(peakValue + eps('single'));
    det.snr_dB = 10 * log10(snrMap(rBin, dBin) + eps('single'));
    det.score = single(det.power_dB + 0.75 * det.snr_dB);

    det.rangeBin = rBin;
    det.dopplerBin = dBin;

    det.azSpec_dB = [];
    det.elSpec_dB = [];
    det.angleQuality_dB = NaN;
end

% Sort detections by score and keep only the strongest maxN.
function detections = sort_detections(detections, maxN)
    if isempty(detections)
        return;
    end

    [~, order] = sort([detections.score], 'descend');
    detections = detections(order(1:min(maxN, numel(order))));
end

% Use parabolic interpolation around a peak for sub-bin accuracy.
function value = refine_axis(powerVector, axisVector, idx)
    value = axisVector(idx);

    if idx <= 1 || idx >= numel(powerVector)
        return;
    end

    y1 = double(powerVector(idx - 1));
    y2 = double(powerVector(idx));
    y3 = double(powerVector(idx + 1));

    denom = y1 - 2 * y2 + y3;

    if abs(denom) < eps
        return;
    end

    delta = 0.5 * (y1 - y3) / denom;
    delta = max(min(delta, 1), -1);

    value = single(double(axisVector(idx)) + delta * double(axisVector(2) - axisVector(1)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EKF tracking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create an empty EKF track array.
function tracks = init_track_list()
    tracks = struct( ...
        'id', {}, ...
        'x', {}, ...
        'P', {}, ...
        'age', {}, ...
        'hits', {}, ...
        'hitHistory', {}, ...
        'misses', {}, ...
        'confirmed', {}, ...
        'score', {}, ...
        'historyXYZ', {}, ...
        'lastDetection', {});
end

% Predict, associate, update, spawn, and delete EKF tracks.
function [tracks, nextTrackId] = ekf_track_manager(tracks, nextTrackId, detections, cfg)
    dt = double(cfg.trackerDt);

    for i = 1:numel(tracks)
        tracks(i) = ekf_predict_track(tracks(i), dt, cfg);
    end

    if isempty(detections)
        for i = 1:numel(tracks)
            tracks(i) = mark_track_missed(tracks(i), cfg);
        end

        tracks = prune_tracks(tracks, cfg);
        return;
    end

    meas = detections_to_radar_measurements(detections, cfg);
    [assignments, unassignedTracks, unassignedDets] = associate_ekf_tracks(tracks, meas, cfg);

    for a = 1:size(assignments, 1)
        ti = assignments(a, 1);
        di = assignments(a, 2);
        tracks(ti) = ekf_update_track(tracks(ti), meas(di), detections(di), cfg);
    end

    for k = 1:numel(unassignedTracks)
        tracks(unassignedTracks(k)) = mark_track_missed(tracks(unassignedTracks(k)), cfg);
    end

    tracks = prune_tracks(tracks, cfg);

    for k = 1:numel(unassignedDets)
        di = unassignedDets(k);

        if numel(tracks) >= cfg.maxTracks
            break;
        end

        if is_detection_near_existing_track(detections(di), tracks, cfg)
            continue;
        end

        tracks(end + 1) = create_track(nextTrackId, meas(di), detections(di), cfg); %#ok<AGROW>
        nextTrackId = nextTrackId + 1;
    end
end

% Constant-velocity EKF prediction step.
function track = ekf_predict_track(track, dt, cfg)
    F = eye(6);
    F(1, 4) = dt;
    F(2, 5) = dt;
    F(3, 6) = dt;

    q = double(cfg.processAccelStd_mps2)^2;
    Q1 = [dt^4 / 4, dt^3 / 2; dt^3 / 2, dt^2] * q;

    Q = zeros(6);
    Q([1 4], [1 4]) = Q1;
    Q([2 5], [2 5]) = Q1;
    Q([3 6], [3 6]) = Q1;

    track.x = F * track.x;
    track.P = F * track.P * F' + Q;
    track.P = 0.5 * (track.P + track.P');
end

% EKF measurement update using range, azimuth, elevation, and radial velocity.
function track = ekf_update_track(track, meas, det, cfg)
    [h, H] = radar_measurement_model(track.x);

    z = meas.z(:);
    R = meas.R;

    innov = z - h;
    innov(2) = wrap_angle_deg(innov(2));

    S = H * track.P * H' + R;
    K = (S \ (H * track.P')).';

    track.x = track.x + K * innov;

    I = eye(6);
    track.P = (I - K * H) * track.P * (I - K * H)' + K * R * K';
    track.P = 0.5 * (track.P + track.P');

    track.age = track.age + 1;
    track.hits = track.hits + 1;
    track.misses = 0;

    track.hitHistory = [track.hitHistory(2:end), true];
    track.confirmed = sum(track.hitHistory) >= cfg.trackConfirmHits;

    track.score = 0.85 * track.score + 0.15 * double(det.score);
    track.lastDetection = det;

    track.historyXYZ = [track.historyXYZ; track.x(1:3).'];

    if size(track.historyXYZ, 1) > cfg.historyLength
        track.historyXYZ = track.historyXYZ(end - cfg.historyLength + 1:end, :);
    end
end

% Age a track when no detection is assigned to it.
function track = mark_track_missed(track, cfg)
    track.age = track.age + 1;
    track.misses = track.misses + 1;
    track.hitHistory = [track.hitHistory(2:end), false];
    track.score = 0.92 * track.score;

    track.historyXYZ = [track.historyXYZ; track.x(1:3).'];

    if size(track.historyXYZ, 1) > cfg.historyLength
        track.historyXYZ = track.historyXYZ(end - cfg.historyLength + 1:end, :);
    end
end

% Initialize a new EKF track from a radar detection.
function track = create_track(trackId, meas, det, cfg)
    R = double(det.range);
    az = double(det.azimuth);
    el = double(det.elevation);
    vr = double(det.velocity);

    [x, y, z] = sph_to_xyz(R, az, el);
    los = [x; y; z] / max(R, eps);

    v0 = vr * los;

    track.id = trackId;
    track.x = [x; y; z; v0];

    track.P = diag([ ...
        cfg.trackInitPosStd_m^2, cfg.trackInitPosStd_m^2, cfg.trackInitPosStd_m^2, ...
        cfg.trackInitVelStd_mps^2, cfg.trackInitVelStd_mps^2, cfg.trackInitVelStd_mps^2]);

    track.age = 1;
    track.hits = 1;
    track.hitHistory = false(1, cfg.trackConfirmWindow);
    track.hitHistory(end) = true;
    track.misses = 0;
    track.confirmed = cfg.trackConfirmHits <= 1;
    track.score = double(det.score);
    track.historyXYZ = [x, y, z];
    track.lastDetection = det;
end

% Delete stale or unconfirmed tracks according to miss counters.
function tracks = prune_tracks(tracks, cfg)
    if isempty(tracks)
        return;
    end

    keep = true(1, numel(tracks));

    for i = 1:numel(tracks)
        if tracks(i).misses > cfg.trackMaxMisses
            keep(i) = false;
        end

        if ~tracks(i).confirmed && tracks(i).misses > cfg.deleteUnconfirmedAfterMisses
            keep(i) = false;
        end
    end

    tracks = tracks(keep);
end

% Convert detection structs to EKF measurement vectors/covariances.
function meas = detections_to_radar_measurements(detections, cfg)
    meas = struct('z', {}, 'R', {});

    for i = 1:numel(detections)
        z = [ ...
            double(detections(i).range); ...
            double(detections(i).azimuth); ...
            double(detections(i).elevation); ...
            double(detections(i).velocity)];

        R = diag([ ...
            cfg.measRangeStd_m^2, ...
            cfg.measAzStd_deg^2, ...
            cfg.measElStd_deg^2, ...
            cfg.measRadialVelStd_mps^2]);

        meas(i).z = z; %#ok<AGROW>
        meas(i).R = R;
    end
end

% Greedily associate detections to predicted tracks using innovation cost.
function [assignments, unassignedTracks, unassignedDets] = associate_ekf_tracks(tracks, meas, cfg)
    Nt = numel(tracks);
    Nd = numel(meas);

    assignments = zeros(0, 2);
    unassignedTracks = 1:Nt;
    unassignedDets = 1:Nd;

    if Nt == 0 || Nd == 0
        return;
    end

    C = inf(Nt, Nd);

    for i = 1:Nt
        [h, H] = radar_measurement_model(tracks(i).x);

        for j = 1:Nd
            innov = meas(j).z(:) - h;
            innov(2) = wrap_angle_deg(innov(2));

            S = H * tracks(i).P * H' + meas(j).R;
            d2 = real(innov' * (S \ innov));

            if d2 <= cfg.gateNormalizedInnovation2 && d2 <= cfg.associationMaxCost
                C(i, j) = d2;
            end
        end
    end

    while true
        [bestCost, idx] = min(C(:));

        if ~isfinite(bestCost)
            break;
        end

        [ti, di] = ind2sub(size(C), idx);
        assignments(end + 1, :) = [ti, di]; %#ok<AGROW>

        C(ti, :) = inf;
        C(:, di) = inf;
    end

    if ~isempty(assignments)
        unassignedTracks = setdiff(1:Nt, assignments(:, 1).');
        unassignedDets = setdiff(1:Nd, assignments(:, 2).');
    end
end

% Prevent duplicate track spawning near an existing track.
function near = is_detection_near_existing_track(det, tracks, cfg)
    near = false;

    for i = 1:numel(tracks)
        [h, ~] = radar_measurement_model(tracks(i).x);

        dR = abs(double(det.range) - h(1));
        dA = abs(wrap_angle_deg(double(det.azimuth) - h(2)));
        dE = abs(double(det.elevation) - h(3));
        dV = abs(double(det.velocity) - h(4));

        if dR <= cfg.spawnSuppressRange_m && ...
           dA <= cfg.spawnSuppressAz_deg && ...
           dE <= cfg.spawnSuppressEl_deg && ...
           dV <= cfg.spawnSuppressVel_mps
            near = true;
            return;
        end
    end
end

% Merge tracks that represent the same physical target.
function tracks = merge_duplicate_tracks(tracks, cfg)
    if numel(tracks) < 2
        return;
    end

    keep = true(1, numel(tracks));

    for i = 1:numel(tracks)
        if ~keep(i)
            continue;
        end

        [hi, ~] = radar_measurement_model(tracks(i).x);

        for j = i + 1:numel(tracks)
            if ~keep(j)
                continue;
            end

            [hj, ~] = radar_measurement_model(tracks(j).x);

            dR = abs(hi(1) - hj(1));
            dA = abs(wrap_angle_deg(hi(2) - hj(2)));
            dE = abs(hi(3) - hj(3));
            dV = abs(hi(4) - hj(4));

            if dR <= cfg.mergeRange_m && ...
               dA <= cfg.mergeAz_deg && ...
               dE <= cfg.mergeEl_deg && ...
               dV <= cfg.mergeVel_mps

                scoreI = tracks(i).score + 5 * tracks(i).confirmed + tracks(i).hits - tracks(i).misses;
                scoreJ = tracks(j).score + 5 * tracks(j).confirmed + tracks(j).hits - tracks(j).misses;

                if scoreI >= scoreJ
                    tracks(i) = absorb_track(tracks(i), tracks(j), cfg);
                    keep(j) = false;
                else
                    tracks(j) = absorb_track(tracks(j), tracks(i), cfg);
                    keep(i) = false;
                    break;
                end
            end
        end
    end

    tracks = tracks(keep);
end

% Combine history and quality fields when one track absorbs another.
function a = absorb_track(a, b, cfg)
    if b.hits > a.hits
        a.lastDetection = b.lastDetection;
    end

    a.hits = max(a.hits, b.hits);
    a.misses = min(a.misses, b.misses);
    a.confirmed = a.confirmed || b.confirmed;
    a.score = max(a.score, b.score);

    a.historyXYZ = [a.historyXYZ; b.historyXYZ];

    if size(a.historyXYZ, 1) > cfg.historyLength
        a.historyXYZ = a.historyXYZ(end - cfg.historyLength + 1:end, :);
    end
end

% Nonlinear radar measurement model and Jacobian for the EKF.
function [h, H] = radar_measurement_model(x)
    px = x(1);
    py = x(2);
    pz = x(3);
    vx = x(4);
    vy = x(5);
    vz = x(6);

    r2 = px^2 + py^2 + pz^2;
    r = sqrt(max(r2, eps));

    rho2 = px^2 + py^2;
    rho = sqrt(max(rho2, eps));

    az = atan2d(px, py);
    el = asind(max(-1, min(1, pz / r)));
    vr = (px * vx + py * vy + pz * vz) / r;

    h = [r; az; el; vr];

    H = zeros(4, 6);

    H(1, 1) = px / r;
    H(1, 2) = py / r;
    H(1, 3) = pz / r;

    rad2deg = 180 / pi;

    H(2, 1) = rad2deg * py / max(rho2, eps);
    H(2, 2) = -rad2deg * px / max(rho2, eps);

    H(3, 1) = -rad2deg * px * pz / max(r^2 * rho, eps);
    H(3, 2) = -rad2deg * py * pz / max(r^2 * rho, eps);
    H(3, 3) = rad2deg * rho / max(r^2, eps);

    dotpv = px * vx + py * vy + pz * vz;

    H(4, 1) = vx / r - dotpv * px / max(r^3, eps);
    H(4, 2) = vy / r - dotpv * py / max(r^3, eps);
    H(4, 3) = vz / r - dotpv * pz / max(r^3, eps);
    H(4, 4) = px / r;
    H(4, 5) = py / r;
    H(4, 6) = pz / r;
end

% Convert EKF tracks back into display-friendly detection structs.
function detections = tracks_to_detections(tracks, cfg)
    detections = empty_detection_struct();

    for i = 1:numel(tracks)
        if ~cfg.showTentativeTracks && ~tracks(i).confirmed
            continue;
        end

        if tracks(i).misses > 0 && ~cfg.showCoastedTracks
            continue;
        end

        [h, ~] = radar_measurement_model(tracks(i).x);

        det = tracks(i).lastDetection;
        det.trackId = tracks(i).id;
        det.trackAge = tracks(i).age;
        det.trackMisses = tracks(i).misses;
        det.confirmed = tracks(i).confirmed;
        det.coasted = tracks(i).misses > 0;
        det.insideFov = is_state_inside_radar_fov(tracks(i).x(1:3), cfg);
        det.range = single(h(1));
        det.azimuth = single(h(2));
        det.elevation = single(h(3));
        det.velocity = single(h(4));
        det.score = single(tracks(i).score);

        detections(end + 1) = det; %#ok<AGROW>
    end
end

% Check whether a Cartesian track state is inside radar FOV.
function inside = is_state_inside_radar_fov(pos, cfg)
    x = double(pos(1));
    y = double(pos(2));
    z = double(pos(3));

    R = sqrt(x^2 + y^2 + z^2);

    if R <= 0 || ~isfinite(R)
        inside = false;
        return;
    end

    az = atan2d(x, y);
    el = asind(max(-1, min(1, z / R)));

    inside = R >= cfg.minDetectRange_m && R <= cfg.maxDetectRange_m && ...
             az >= cfg.azFovDeg(1) && az <= cfg.azFovDeg(2) && ...
             el >= cfg.elFovDeg(1) && el <= cfg.elFovDeg(2);

    if cfg.requireInFrontOfRadar
        inside = inside && y > 0;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% History, plotting, and display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Allocate rolling history used for frame-to-frame error plots.
function history = init_history(cfg)
    history.timeIndex = 0;
    history.errRange = nan(cfg.historyLength, cfg.maxTracks, 'single');
    history.errVelocity = nan(cfg.historyLength, cfg.maxTracks, 'single');
    history.errAzimuth = nan(cfg.historyLength, cfg.maxTracks, 'single');
    history.errElevation = nan(cfg.historyLength, cfg.maxTracks, 'single');
    history.prevByTrackId = containers.Map('KeyType', 'double', 'ValueType', 'any');
end

% Update per-track changes in range, velocity, azimuth, and elevation.
function history = update_history(history, detections, cfg)
    history.timeIndex = history.timeIndex + 1;
    idx = mod(history.timeIndex - 1, cfg.historyLength) + 1;

    history.errRange(idx, :) = nan;
    history.errVelocity(idx, :) = nan;
    history.errAzimuth(idx, :) = nan;
    history.errElevation(idx, :) = nan;

    for k = 1:min(numel(detections), cfg.maxTracks)
        id = double(detections(k).trackId);

        current = [ ...
            double(detections(k).range), ...
            double(detections(k).velocity), ...
            double(detections(k).azimuth), ...
            double(detections(k).elevation)];

        if isKey(history.prevByTrackId, id)
            prev = history.prevByTrackId(id);

            history.errRange(idx, k) = single(current(1) - prev(1));
            history.errVelocity(idx, k) = single(current(2) - prev(2));
            history.errAzimuth(idx, k) = single(wrap_angle_deg(current(3) - prev(3)));
            history.errElevation(idx, k) = single(current(4) - prev(4));
        end

        history.prevByTrackId(id) = current;
    end
end

% Create and initialize all MATLAB figures/graphics handles.
function h = setup_plots(rangeAxis, dopplerAxis, rangeMask, angle, N_doppler, cfg, p)
    h.figRDM = figure('Name', 'Range-Doppler Map', 'Color', 'w');
    h.axRDM = axes(h.figRDM);
    h.rdmImage = imagesc(h.axRDM, dopplerAxis, rangeAxis(rangeMask), zeros(sum(rangeMask), N_doppler, 'single'));
    axis(h.axRDM, 'xy');
    grid(h.axRDM, 'on');
    colorbar(h.axRDM);
    caxis(h.axRDM, [cfg.rdDisplayFloor_dB 0]);
    xlabel(h.axRDM, 'Velocity (m/s)');
    ylabel(h.axRDM, 'Range (m)');
    hold(h.axRDM, 'on');
    h.rdmRaw = plot(h.axRDM, nan, nan, 'wo', 'LineWidth', 1.5, 'MarkerSize', 8);
    h.rdmTrack = plot(h.axRDM, nan, nan, 'rx', 'LineWidth', 2.5, 'MarkerSize', 10);

    h.figCFAR = figure('Name', 'Filtered Detection Map', 'Color', 'w');
    h.axCFAR = axes(h.figCFAR);
    h.cfarImage = imagesc(h.axCFAR, dopplerAxis, rangeAxis(rangeMask), nan(sum(rangeMask), N_doppler, 'single'));
    axis(h.axCFAR, 'xy');
    grid(h.axCFAR, 'on');
    colorbar(h.axCFAR);
    caxis(h.axCFAR, [cfg.cfarDisplayFloor_dB 0]);
    xlabel(h.axCFAR, 'Velocity (m/s)');
    ylabel(h.axCFAR, 'Range (m)');
    hold(h.axCFAR, 'on');
    h.cfarRaw = plot(h.axCFAR, nan, nan, 'ro', 'LineWidth', 1.5, 'MarkerSize', 8);

    h.figAz = figure('Name', 'MUSIC Azimuth Spectra', 'Color', 'w');
    h.axAz = axes(h.figAz);
    hold(h.axAz, 'on');
    grid(h.axAz, 'on');
    xlim(h.axAz, [min(angle.azGrid) max(angle.azGrid)]);
    ylim(h.axAz, [-70 5]);
    xlabel(h.axAz, 'Azimuth (deg)');
    ylabel(h.axAz, 'MUSIC spectrum (dB)');

    h.figEl = figure('Name', 'MUSIC Elevation Spectra', 'Color', 'w');
    h.axEl = axes(h.figEl);
    hold(h.axEl, 'on');
    grid(h.axEl, 'on');
    xlim(h.axEl, [min(angle.elGrid) max(angle.elGrid)]);
    ylim(h.axEl, [-70 5]);
    xlabel(h.axEl, 'Elevation (deg)');
    ylabel(h.axEl, 'MUSIC spectrum (dB)');

    h.fig3D = figure('Name', '3D Detection and Tracking', 'Color', 'w');
    h.ax3D = axes(h.fig3D);
    hold(h.ax3D, 'on');
    grid(h.ax3D, 'on');
    view(h.ax3D, 3);
    xlabel(h.ax3D, 'X-range (m)');
    ylabel(h.ax3D, 'Y-range (m)');
    zlabel(h.ax3D, 'Z-height (m)');
    xlim(h.ax3D, cfg.plotXLim);
    ylim(h.ax3D, cfg.plotYLim);
    zlim(h.ax3D, cfg.plotZLim);
    daspect(h.ax3D, [1 1 1]);

    [groundX, groundY] = meshgrid(cfg.plotXLim, cfg.plotYLim);
    groundZ = zeros(size(groundX), 'single');
    surf(h.ax3D, groundX, groundY, groundZ, ...
        'FaceAlpha', 0.08, ...
        'EdgeAlpha', 0.12, ...
        'FaceColor', [0.45 0.45 0.45], ...
        'EdgeColor', [0.25 0.25 0.25], ...
        'HandleVisibility', 'off');

    plot3(h.ax3D, 0, 0, 0, 'k*', ...
        'MarkerSize', 18, ...
        'LineWidth', 2.5, ...
        'DisplayName', 'Radar');

    if cfg.showRadarCoverageVolume
        h.coverageVolume = draw_radar_coverage_volume(h.ax3D, cfg);
    else
        h.coverageVolume = gobjects(0);
    end

    colormap(h.ax3D, turbo);
    caxis(h.ax3D, [-p.maxVel p.maxVel]);
    h.velColorbar = colorbar(h.ax3D);
    ylabel(h.velColorbar, 'Radial velocity (m/s)');

    h.det3D = scatter3(h.ax3D, nan, nan, nan, ...
        120, nan, 'filled', ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 0.8);

    h.figErr = figure('Name', 'Frame-to-Frame Track Errors', 'Color', 'w');
    h.axErr(1) = subplot(4, 1, 1, 'Parent', h.figErr);
    ylabel(h.axErr(1), '\Delta Range (m)');
    h.axErr(2) = subplot(4, 1, 2, 'Parent', h.figErr);
    ylabel(h.axErr(2), '\Delta Velocity (m/s)');
    h.axErr(3) = subplot(4, 1, 3, 'Parent', h.figErr);
    ylabel(h.axErr(3), '\Delta Azimuth (deg)');
    h.axErr(4) = subplot(4, 1, 4, 'Parent', h.figErr);
    ylabel(h.axErr(4), '\Delta Elevation (deg)');
    xlabel(h.axErr(4), 'Frame');

    for a = 1:4
        hold(h.axErr(a), 'on');
        grid(h.axErr(a), 'on');
    end

    h.errLines = gobjects(4, cfg.maxTracks);
    for a = 1:4
        for k = 1:cfg.maxTracks
            h.errLines(a, k) = plot(h.axErr(a), nan, nan, 'LineWidth', 1.1);
        end
    end
end

% Refresh range-Doppler, CFAR, MUSIC, 3D, and error plots.
% Draw the radar coverage volume from configured range, azimuth, and elevation FOV.
function hCov = draw_radar_coverage_volume(ax, cfg)
    Rmax = cfg.maxDetectRange_m;

    if ~isfinite(Rmax)
        Rmax = max(cfg.plotYLim);
    end

    Rmin = max(cfg.minDetectRange_m, 0);

    az = linspace(cfg.azFovDeg(1), cfg.azFovDeg(2), cfg.coverageAzSamples);
    el = linspace(cfg.elFovDeg(1), cfg.elFovDeg(2), cfg.coverageElSamples);

    [AZ, EL] = meshgrid(az, el);

    [Xmax, Ymax, Zmax] = sph_to_xyz(Rmax, AZ, EL);
    [Xmin, Ymin, Zmin] = sph_to_xyz(Rmin, AZ, EL);

    hold(ax, 'on');
    hCov = gobjects(0);

    faceColor = [0.2 0.7 1.0];
    edgeColor = [0.1 0.4 0.8];

    % Far curved surface.
    hCov(end + 1) = surf(ax, Xmax, Ymax, Zmax, ...
        'FaceColor', faceColor, ...
        'FaceAlpha', cfg.coverageFaceAlpha, ...
        'EdgeColor', edgeColor, ...
        'EdgeAlpha', cfg.coverageEdgeAlpha, ...
        'DisplayName', 'Radar coverage');

    % Near curved surface, only visible if the minimum range is nonzero.
    if Rmin > 0
        hCov(end + 1) = surf(ax, Xmin, Ymin, Zmin, ...
            'FaceColor', faceColor, ...
            'FaceAlpha', cfg.coverageFaceAlpha, ...
            'EdgeColor', edgeColor, ...
            'EdgeAlpha', cfg.coverageEdgeAlpha, ...
            'HandleVisibility', 'off');
    end

    % Azimuth boundary surfaces.
    for iaz = [1, numel(az)]
        Xside = [Xmin(:, iaz), Xmax(:, iaz)];
        Yside = [Ymin(:, iaz), Ymax(:, iaz)];
        Zside = [Zmin(:, iaz), Zmax(:, iaz)];

        hCov(end + 1) = surf(ax, Xside, Yside, Zside, ...
            'FaceColor', faceColor, ...
            'FaceAlpha', cfg.coverageFaceAlpha, ...
            'EdgeColor', edgeColor, ...
            'EdgeAlpha', cfg.coverageEdgeAlpha, ...
            'HandleVisibility', 'off');
    end

    % Elevation boundary surfaces.
    for iel = [1, numel(el)]
        Xside = [Xmin(iel, :).', Xmax(iel, :).'];
        Yside = [Ymin(iel, :).', Ymax(iel, :).'];
        Zside = [Zmin(iel, :).', Zmax(iel, :).'];

        hCov(end + 1) = surf(ax, Xside, Yside, Zside, ...
            'FaceColor', faceColor, ...
            'FaceAlpha', cfg.coverageFaceAlpha, ...
            'EdgeColor', edgeColor, ...
            'EdgeAlpha', cfg.coverageEdgeAlpha, ...
            'HandleVisibility', 'off');
    end
end

function h = update_plots(h, frameCounter, powerRD, cleanCfarMap, rangeMask, rawDetections, displayDetections, tracks, angle, history, cfg)
    rdPower = powerRD;

    if cfg.enableRdDisplayCompression
        rdPower = rdPower .^ cfg.rdDisplayGamma;
        rdPower = rdPower ./ (max(rdPower(:)) + eps('single'));
    end

    rdDisplay = 10 * log10(rdPower(rangeMask, :) + eps('single'));
    rdDisplay(rdDisplay < cfg.rdDisplayFloor_dB) = cfg.rdDisplayFloor_dB;

    set(h.rdmImage, 'CData', rdDisplay);
    title(h.axRDM, sprintf('Range-Doppler Map | Frame %d', frameCounter));

    cfarDisplay = nan(size(cleanCfarMap), 'single');
    cfarDisplay(cleanCfarMap) = 10 * log10(powerRD(cleanCfarMap) + eps('single'));
    cfarDisplay(cfarDisplay < cfg.cfarDisplayFloor_dB) = cfg.cfarDisplayFloor_dB;

    set(h.cfarImage, 'CData', cfarDisplay(rangeMask, :));
    title(h.axCFAR, sprintf('Filtered Detection Map | Frame %d', frameCounter));

    if isempty(rawDetections)
        set(h.rdmRaw, 'XData', nan, 'YData', nan);
        set(h.cfarRaw, 'XData', nan, 'YData', nan);
    else
        set(h.rdmRaw, 'XData', [rawDetections.velocity], 'YData', [rawDetections.range]);
        set(h.cfarRaw, 'XData', [rawDetections.velocity], 'YData', [rawDetections.range]);
    end

    if isempty(displayDetections)
        set(h.rdmTrack, 'XData', nan, 'YData', nan);
        set(h.det3D, 'XData', nan, 'YData', nan, 'ZData', nan, 'CData', nan);
    else
        set(h.rdmTrack, 'XData', [displayDetections.velocity], 'YData', [displayDetections.range]);

        [x, y, z] = sph_to_xyz([displayDetections.range].', [displayDetections.azimuth].', [displayDetections.elevation].');
        v = [displayDetections.velocity].';

        set(h.det3D, ...
            'XData', x, ...
            'YData', y, ...
            'ZData', z, ...
            'CData', v);
    end

    update_music_plots(h, rawDetections, angle, cfg);
    update_track_paths_and_arrows(h, tracks, cfg);
    update_error_plots(h, history, cfg);

    drawnow limitrate;
end

% Draw MUSIC azimuth and elevation spectra for detections.
function update_music_plots(h, detections, angle, cfg)
    delete(findall(h.axAz, 'Tag', 'musicLine'));
    delete(findall(h.axAz, 'Tag', 'musicPeak'));
    delete(findall(h.axEl, 'Tag', 'musicLine'));
    delete(findall(h.axEl, 'Tag', 'musicPeak'));

    for k = 1:numel(detections)
        if cfg.plotAllAngleSpectra && ~isempty(detections(k).azSpec_dB)
            plot(h.axAz, angle.azGrid, detections(k).azSpec_dB, ...
                'LineWidth', 1.2, ...
                'Tag', 'musicLine');
        end

        if cfg.plotAllAngleSpectra && ~isempty(detections(k).elSpec_dB)
            plot(h.axEl, angle.elGrid, detections(k).elSpec_dB, ...
                'LineWidth', 1.2, ...
                'Tag', 'musicLine');
        end

        if cfg.plotAngleMarkers && isfinite(detections(k).azimuth)
            plot(h.axAz, detections(k).azimuth, 0, 'o', ...
                'MarkerSize', 8, ...
                'LineWidth', 2, ...
                'Tag', 'musicPeak');
        end

        if cfg.plotAngleMarkers && isfinite(detections(k).elevation)
            plot(h.axEl, detections(k).elevation, 0, 'o', ...
                'MarkerSize', 8, ...
                'LineWidth', 2, ...
                'Tag', 'musicPeak');
        end
    end

    title(h.axAz, 'MUSIC Azimuth Spectra');
    title(h.axEl, 'MUSIC Elevation Spectra');
end

% Draw track histories and velocity arrows in the 3D plot.
function update_track_paths_and_arrows(h, tracks, cfg)
    delete(findall(h.ax3D, 'Tag', 'trackPath'));
    delete(findall(h.ax3D, 'Tag', 'trackLabel'));
    delete(findall(h.ax3D, 'Tag', 'trackArrow'));

    for i = 1:numel(tracks)
        if ~tracks(i).confirmed
            continue;
        end

        xyz = tracks(i).historyXYZ;

        if size(xyz, 1) >= 2
            plot3(h.ax3D, xyz(:, 1), xyz(:, 2), xyz(:, 3), ...
                '-', ...
                'LineWidth', 2.0, ...
                'Tag', 'trackPath');
        end

        pos = tracks(i).x(1:3);
        vel = tracks(i).x(4:6);

        speed = norm(vel);

        if speed > 0
            arrowLen = min(max(speed * cfg.arrowScale, cfg.arrowMinLength_m), cfg.arrowMaxLength_m);
            dir = vel / speed * arrowLen;

            quiver3(h.ax3D, pos(1), pos(2), pos(3), ...
                dir(1), dir(2), dir(3), 0, ...
                'LineWidth', 2.2, ...
                'MaxHeadSize', 1.4, ...
                'Tag', 'trackArrow');
        end

        text(h.ax3D, pos(1), pos(2), pos(3), ...
            sprintf('ID %d', tracks(i).id), ...
            'FontWeight', 'bold', ...
            'Tag', 'trackLabel');
    end
end

% Refresh frame-to-frame track-error history plots.
function update_error_plots(h, history, cfg)
    idxCount = min(history.timeIndex, cfg.historyLength);

    if idxCount <= 0
        return;
    end

    if history.timeIndex > cfg.historyLength
        startIdx = mod(history.timeIndex - idxCount, cfg.historyLength) + 1;
        order = mod((startIdx:startIdx + idxCount - 1) - 1, cfg.historyLength) + 1;
    else
        order = 1:idxCount;
    end

    x = 1:idxCount;
    E = {history.errRange, history.errVelocity, history.errAzimuth, history.errElevation};

    for a = 1:4
        for k = 1:cfg.maxTracks
            set(h.errLines(a, k), 'XData', x, 'YData', E{a}(order, k));
        end
        autoscale_error_axis(h.axErr(a));
    end

    title(h.axErr(1), sprintf('Frame-to-Frame Track Errors | Frame %d', history.timeIndex));
end

% Auto-scale one error subplot while handling missing data.
function autoscale_error_axis(ax)
    lines = findobj(ax, 'Type', 'line');
    y = [];

    for i = 1:numel(lines)
        yi = get(lines(i), 'YData');
        y = [y, yi(isfinite(yi))]; %#ok<AGROW>
    end

    if ~isempty(y)
        m = max(abs(y));
        ylim(ax, [-max(m * 1.2, 0.1), max(m * 1.2, 0.1)]);
    end
end

% Build a display mask centered on final validated detections.
function cleanMap = final_detection_map(mapSize, detections, cfg)
    cleanMap = false(mapSize);

    if isempty(detections)
        return;
    end

    rRadius = max(1, round(cfg.clusterRangeBins / 2));
    dRadius = max(1, round(cfg.clusterDopplerBins / 2));

    for k = 1:numel(detections)
        r = detections(k).rangeBin;
        d = detections(k).dopplerBin;

        r1 = max(1, r - rRadius);
        r2 = min(mapSize(1), r + rRadius);
        d1 = max(1, d - dRadius);
        d2 = min(mapSize(2), d + dRadius);

        cleanMap(r1:r2, d1:d2) = true;
    end
end

% Return an empty detection struct with all expected fields.
function detections = empty_detection_struct()
    detections = struct( ...
        'trackId', {}, ...
        'trackAge', {}, ...
        'trackMisses', {}, ...
        'confirmed', {}, ...
        'coasted', {}, ...
        'insideFov', {}, ...
        'range', {}, ...
        'velocity', {}, ...
        'azimuth', {}, ...
        'elevation', {}, ...
        'power_dB', {}, ...
        'snr_dB', {}, ...
        'score', {}, ...
        'rangeBin', {}, ...
        'dopplerBin', {}, ...
        'azSpec_dB', {}, ...
        'elSpec_dB', {}, ...
        'angleQuality_dB', {});
end

% Print a compact table of current displayed detections/tracks.
function print_detections(detections)
    if isempty(detections)
        fprintf('  No confirmed tracks\n');
        return;
    end

    fprintf(' ID Trk Age Miss Cnf Coast FOV   Range(m)   RadVel(m/s)   Az(deg)   El(deg)   Power(dB)  Qang(dB)\n');

    for k = 1:numel(detections)
        fprintf('%3d %3d %3d %4d %3d %5d %3d %10.3f %13.3f %9.3f %9.3f %10.3f %9.3f\n', ...
            k, detections(k).trackId, detections(k).trackAge, detections(k).trackMisses, ...
            detections(k).confirmed, detections(k).coasted, detections(k).insideFov, ...
            detections(k).range, detections(k).velocity, detections(k).azimuth, detections(k).elevation, ...
            detections(k).power_dB, detections(k).angleQuality_dB);
    end
end

% Convert radar spherical coordinates to Cartesian coordinates.
function [x, y, z] = sph_to_xyz(r, az, el)
    x = r .* cosd(el) .* sind(az);
    y = r .* cosd(el) .* cosd(az);
    z = r .* sind(el);
end

% Wrap an angle in degrees to the interval [-180, 180).
function a = wrap_angle_deg(a)
    a = mod(a + 180, 360) - 180;
end

% Unpack interleaved float32 I/Q ZMQ payload into complex samples.
function x = zmq_bytes_to_complex(msg)
    b = uint8(py.array.array('B', msg));
    f = typecast(b, 'single');

    if mod(numel(f), 2) ~= 0
        error('Odd number of float32 values. ZMQ stream must be interleaved float32 IQ.');
    end

    x = complex(f(1:2:end), f(2:2:end));
    x = x(:);
end
