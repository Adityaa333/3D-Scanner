function processScanDistance(inputFile, outputStlFile)
%PROCESSSCANDISTANCE  Turn a raw scanner text file into an STL mesh.
%
%   processScanDistance(INPUTFILE, OUTPUTSTLFILE) reads the raw distance
%   readings written by scannerCode.ino (one value per line, with a
%   delimiter value of 9999 marking the end of each Z slice), cleans and
%   filters the data, converts the polar (angle, radius) readings for
%   each slice into XYZ points, smooths/resamples the resulting mesh,
%   plots it, and writes it out as an STL file using surf2stl.
%
%   Example:
%       processScanDistance('farmer.txt', 'farmer.stl');
%
%   If called with no arguments, defaults to 'farmer.txt' -> 'farmer.stl'.

    if nargin < 1 || isempty(inputFile)
        inputFile = 'farmer.txt';
    end
    if nargin < 2 || isempty(outputStlFile)
        outputStlFile = 'farmer.stl';
    end

    % ---- Configuration --------------------------------------------------
    delimiterValue    = 9999;   % marks the end of a Z slice in the raw file
    zStepHeightMm      = 2.0;   % physical height advanced between slices
    outlierWindow      = 5;     % window (samples) for median filtering
    outlierThreshold   = 15;    % mm; readings further from local median are rejected
    resampleAngles     = 180;   % number of angular samples per slice after resampling
    smoothSpan         = 0.05;  % fraction of points used for mesh smoothing

    % ---- 1. Read raw readings and split into per-slice cell array -------
    rawValues = readRawScan(inputFile);
    slices = splitIntoSlices(rawValues, delimiterValue);

    numSlices = numel(slices);
    fprintf('Loaded %d slices from %s\n', numSlices, inputFile);

    % ---- 2. Clean/filter each slice's radius readings --------------------
    for s = 1:numSlices
        slices{s} = cleanSlice(slices{s}, outlierWindow, outlierThreshold);
    end

    % ---- 3. Convert polar (theta, radius) -> XYZ for each slice ---------
    [X, Y, Z] = polarSlicesToXYZ(slices, zStepHeightMm, resampleAngles);

    % ---- 4. Smooth / resample the resulting mesh --------------------------
    [Xs, Ys, Zs] = smoothMesh(X, Y, Z, smoothSpan);

    % ---- 5. Plot the result ------------------------------------------------
    figure('Name', 'Scanned mesh');
    surf(Xs, Ys, Zs, 'EdgeColor', 'none');
    axis equal;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('Reconstructed 3D scan');
    camlight; lighting gouraud;
    colormap(gray);
    view(45, 25);

    % ---- 6. Export STL --------------------------------------------------
    surf2stl(outputStlFile, Xs, Ys, Zs, 'binary');
    fprintf('Wrote mesh to %s\n', outputStlFile);
end

% =========================================================================
function rawValues = readRawScan(inputFile)
% Read the raw scan text file: one numeric value per line.

    fid = fopen(inputFile, 'r');
    if fid == -1
        error('processScanDistance:fileOpen', 'Could not open %s.', inputFile);
    end

    rawValues = fscanf(fid, '%f');
    fclose(fid);
end

% =========================================================================
function slices = splitIntoSlices(rawValues, delimiterValue)
% Split a flat vector of readings into a cell array, one entry per Z
% slice, breaking on the delimiter value.

    slices = {};
    current = [];

    for i = 1:numel(rawValues)
        v = rawValues(i);
        if v == delimiterValue
            if ~isempty(current)
                slices{end+1} = current; %#ok<AGROW>
                current = [];
            end
        else
            current(end+1) = v; %#ok<AGROW>
        end
    end

    % Catch a trailing slice with no closing delimiter
    if ~isempty(current)
        slices{end+1} = current;
    end
end

% =========================================================================
function cleaned = cleanSlice(radii, windowSize, threshold)
% Remove spurious readings (sensor glitches, out-of-range values) using
% a local median filter, then linearly interpolate over the gaps.

    radii = radii(:)';
    n = numel(radii);

    medFiltered = medfilt1Simple(radii, windowSize);
    isOutlier = abs(radii - medFiltered) > threshold | radii <= 0;

    goodIdx = find(~isOutlier);
    if numel(goodIdx) < 2
        cleaned = medFiltered; % not enough good points, fall back
        return;
    end

    allIdx = 1:n;
    cleaned = interp1(goodIdx, radii(goodIdx), allIdx, 'linear', 'extrap');
end

% =========================================================================
function y = medfilt1Simple(x, windowSize)
% Simple centered median filter (avoids requiring the Signal Processing
% Toolbox's medfilt1).

    n = numel(x);
    halfWin = floor(windowSize / 2);
    y = zeros(1, n);

    for i = 1:n
        lo = max(1, i - halfWin);
        hi = min(n, i + halfWin);
        y(i) = median(x(lo:hi));
    end
end

% =========================================================================
function [X, Y, Z] = polarSlicesToXYZ(slices, zStepHeightMm, resampleAngles)
% Convert each slice's (angle, radius) readings into XYZ points on a
% regular grid, resampling each slice to a common number of angular
% samples so the slices can be stacked into a surface.

    numSlices = numel(slices);
    thetaOut = linspace(0, 2*pi, resampleAngles + 1);
    thetaOut(end) = []; % drop duplicate 0/2pi sample

    X = zeros(numSlices, resampleAngles);
    Y = zeros(numSlices, resampleAngles);
    Z = zeros(numSlices, resampleAngles);

    for s = 1:numSlices
        radii = slices{s};
        nPts = numel(radii);

        thetaIn = linspace(0, 2*pi, nPts + 1);
        thetaIn(end) = [];

        % Resample onto the common angular grid (wrap-around interpolation)
        radiiExt = [radii, radii(1)];
        thetaInExt = [thetaIn, 2*pi];
        radiiResampled = interp1(thetaInExt, radiiExt, thetaOut, 'linear');

        X(s, :) = radiiResampled .* cos(thetaOut);
        Y(s, :) = radiiResampled .* sin(thetaOut);
        Z(s, :) = (s - 1) * zStepHeightMm;
    end
end

% =========================================================================
function [Xs, Ys, Zs] = smoothMesh(X, Y, Z, span)
% Light smoothing along both mesh directions to reduce sensor noise
% while preserving the overall shape.

    [rows, cols] = size(Z);
    Zs = Z;

    windowRows = max(3, round(span * rows));
    windowCols = max(3, round(span * cols));

    % Smooth along columns (down each angular ray, across Z slices)
    for c = 1:cols
        Zs(:, c) = movingAverage(Zs(:, c), windowRows);
    end

    % Smooth along rows (around each slice's angular profile, wrapping)
    for r = 1:rows
        Zs(r, :) = movingAverageCircular(Zs(r, :), windowCols);
    end

    Xs = X;
    Ys = Y;
end

% =========================================================================
function y = movingAverage(x, windowSize)
    n = numel(x);
    halfWin = floor(windowSize / 2);
    y = x;
    for i = 1:n
        lo = max(1, i - halfWin);
        hi = min(n, i + halfWin);
        y(i) = mean(x(lo:hi));
    end
end

% =========================================================================
function y = movingAverageCircular(x, windowSize)
% Moving average that wraps around, appropriate for angular data.

    n = numel(x);
    halfWin = floor(windowSize / 2);
    xExt = [x(end-halfWin+1:end), x, x(1:halfWin)];
    y = zeros(1, n);
    for i = 1:n
        idx = i:(i + 2*halfWin);
        y(i) = mean(xExt(idx));
    end
end
