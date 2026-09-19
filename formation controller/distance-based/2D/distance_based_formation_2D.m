%% Distance-Based Formation Control in 2D
% Simulate N agents converging to a regular polygon using distance errors.
% The script evaluates complete, minimally globally rigid, minimally rigid,
% and non-rigid graph topologies and overwrites the 2D result directory.

clc; clear; close all;

%% 1) User configuration
nAgents = 5;                 % Number of polygon vertices/agents (N >= 3)
polygonRadius = 1.0;        % Radius of the desired regular polygon
Tfinal = 20.0;               % Simulation duration in seconds
dt = 0.01;                  % Euler integration step in seconds
kGain = 1.5;                % Distance-controller gain
convergenceThreshold = 1e-3; % Stop exported transients after sustained convergence
videoFrameStride = 5;        % Export every fifth simulation sample
animationPlaybackRate = 0.6; % Video time advances at 60% of simulation time
videoQuality = 65;           % Compact H.264 output for GitHub and PowerPoint

% Set this to [] to generate a reproducible default initialization. Otherwise,
% provide an nAgents-by-2 matrix, with one [x y] row per agent.
initialPositions = [];

visualization.lineWidth = 2.5;
visualization.graphLineWidth = 2.0;
visualization.fontSize = 14;
visualization.titleFontSize = 16;
visualization.annotationFontSize = 13;
visualization.markerSize = 9;
visualization.legendFontSize = 12;

%% 2) Project and formation setup
scriptFolder = fileparts(mfilename('fullpath'));
resultFolder = fullfile(scriptFolder, 'result');
resetResultFolder(resultFolder);

desiredPositions = regularPolygon(nAgents, polygonRadius);
if isempty(initialPositions)
    initialPositions = defaultInitialPositions(desiredPositions);
end
validatePositions(initialPositions, nAgents, 2);

desiredDistanceSquared = pairwiseDistanceSquared(desiredPositions);
graphNames = {'complete', 'minimally_globally_rigid', 'minimally_rigid', 'non_rigid'};

fprintf('\n2D distance-based formation control\n');
fprintf('Agents: %d | Duration: %.2f s | dt: %.3f s\n', nAgents, Tfinal, dt);

%% 3) Run every graph topology
for graphIndex = 1:numel(graphNames)
    graphName = graphNames{graphIndex};
    adjacency = buildGraph(graphName, nAgents);
    expectedEdgeCount = graphEdgeCount(graphName, nAgents, 2);
    actualEdgeCount = nnz(triu(adjacency));
    if actualEdgeCount ~= expectedEdgeCount
        error('%s graph has %d edges; expected %d.', graphName, actualEdgeCount, expectedEdgeCount);
    end
    fprintf('%-25s edges: %d\n', graphName, actualEdgeCount);
    caseFolder = fullfile(resultFolder, graphName);
    if ~isfolder(caseFolder)
        mkdir(caseFolder);
    end

    [trajectory, edgeErrorHistory, edgePairs] = simulateFormation( ...
        initialPositions, desiredDistanceSquared, adjacency, Tfinal, dt, kGain);

    convergenceIndex = sustainedConvergenceIndex(edgeErrorHistory, convergenceThreshold);
    outputTrajectory = trajectory(1:convergenceIndex,:,:);
    outputErrorHistory = edgeErrorHistory(1:convergenceIndex,:);
    convergenceTime = (convergenceIndex-1)*dt;

    distanceError = sqrt(mean(edgeErrorHistory(end,:).^2));
    fprintf('%-25s final RMS edge error: %.6f\n', graphName, distanceError);
    fprintf('%-25s export cutoff: %.2f s (|error| <= %.4g)\n', ...
        graphName, convergenceTime, convergenceThreshold);

    saveDesiredGraphFigure(caseFolder, desiredPositions, adjacency, graphName, visualization);
    saveTrajectoryFigure(caseFolder, trajectory, adjacency, graphName, visualization);
    saveDistanceErrorFigure(caseFolder, outputErrorHistory, edgePairs, dt, graphName, visualization);
    saveAnimation(caseFolder, outputTrajectory, desiredPositions, adjacency, graphName, ...
        dt, videoFrameStride, animationPlaybackRate, videoQuality, visualization);
end

fprintf('Results written to: %s\n', resultFolder);

%% Local functions
function positions = regularPolygon(nAgents, radius)
    angles = (0:nAgents-1)' * (2*pi/nAgents) + pi/2;
    positions = radius * [cos(angles), sin(angles)];
end

function positions = defaultInitialPositions(desiredPositions)
    nAgents = size(desiredPositions, 1);
    offsets = [linspace(-0.9, 0.9, nAgents)', 0.65*cos((1:nAgents)'*1.7)];
    positions = desiredPositions + offsets;
end

function validatePositions(positions, nAgents, dimension)
    if ~isequal(size(positions), [nAgents, dimension])
        error('initialPositions must be an nAgents-by-%d matrix.', dimension);
    end
    if any(~isfinite(positions), 'all')
        error('initialPositions must contain only finite values.');
    end
end

function distanceSquared = pairwiseDistanceSquared(positions)
    distanceSquared = zeros(size(positions, 1));
    for i = 1:size(positions, 1)
        for j = i+1:size(positions, 1)
            distanceSquared(i, j) = sum((positions(i,:) - positions(j,:)).^2);
            distanceSquared(j, i) = distanceSquared(i, j);
        end
    end
end

function adjacency = buildGraph(graphName, nAgents)
    minimumEdges = 2*nAgents - 3;
    globalEdges = 3*nAgents - 6;
    switch graphName
        case 'complete'
            edgeCount = nAgents*(nAgents-1)/2;
        case 'minimally_globally_rigid'
            edgeCount = globalEdges;
        case 'minimally_rigid'
            edgeCount = minimumEdges;
        case 'non_rigid'
            edgeCount = minimumEdges - 1;
        otherwise
            error('Unknown graph type: %s', graphName);
    end
    if nAgents < 3 || edgeCount < 0 || edgeCount > nAgents*(nAgents-1)/2
        error('nAgents must be at least 3 for the requested 2D graph counts.');
    end
    adjacency = countedGraph(nAgents, edgeCount);
end

function edgeCount = graphEdgeCount(graphName, nAgents, dimension)
    switch graphName
        case 'complete'
            edgeCount = nAgents*(nAgents-1)/2;
        case 'minimally_globally_rigid'
            edgeCount = (dimension+1)*nAgents - (dimension+2)*(dimension+1)/2;
        case 'minimally_rigid'
            edgeCount = dimension*nAgents - (dimension+1)*dimension/2;
        case 'non_rigid'
            edgeCount = dimension*nAgents - (dimension+1)*dimension/2 - 1;
        otherwise
            error('Unknown graph type: %s', graphName);
    end
end

function adjacency = countedGraph(nAgents, edgeCount)
    adjacency = zeros(nAgents);
    edgeIndex = 0;
    for i = 1:nAgents
        for j = i+1:nAgents
            edgeIndex = edgeIndex + 1;
            if edgeIndex > edgeCount
                return;
            end
            adjacency(i,j) = 1;
            adjacency(j,i) = 1;
        end
    end
end

function [trajectory, edgeErrorHistory, edgePairs] = simulateFormation(initialPositions, desiredDistanceSquared, adjacency, Tfinal, dt, kGain)
    nSteps = round(Tfinal/dt);
    nAgents = size(initialPositions, 1);
    [edgeRows, edgeColumns] = find(triu(adjacency, 1));
    edgePairs = [edgeRows, edgeColumns];
    positions = initialPositions;
    trajectory = zeros(nSteps+1, nAgents, 2);
    edgeErrorHistory = zeros(nSteps+1, size(edgePairs, 1));
    trajectory(1,:,:) = positions;
    edgeErrorHistory(1,:) = calculateEdgeErrors(positions, desiredDistanceSquared, edgePairs);
    for step = 1:nSteps
        velocity = zeros(size(positions));
        for i = 1:nAgents
            for j = i+1:nAgents
                if adjacency(i,j) == 0
                    continue;
                end
                relativePosition = positions(i,:) - positions(j,:);
                errorSquared = sum(relativePosition.^2) - desiredDistanceSquared(i,j);
                contribution = -kGain * errorSquared * relativePosition;
                velocity(i,:) = velocity(i,:) + contribution;
                velocity(j,:) = velocity(j,:) - contribution;
            end
        end
        positions = positions + dt * velocity;
        trajectory(step+1,:,:) = positions;
        edgeErrorHistory(step+1,:) = calculateEdgeErrors(positions, desiredDistanceSquared, edgePairs);
    end
end

function edgeErrors = calculateEdgeErrors(positions, desiredDistanceSquared, edgePairs)
    edgeErrors = zeros(1, size(edgePairs, 1));
    for edgeIndex = 1:size(edgePairs, 1)
        i = edgePairs(edgeIndex, 1);
        j = edgePairs(edgeIndex, 2);
        edgeErrors(edgeIndex) = sum((positions(i,:) - positions(j,:)).^2) ...
            - desiredDistanceSquared(i,j);
    end
end

function convergenceIndex = sustainedConvergenceIndex(edgeErrorHistory, threshold)
    maximumError = max(abs(edgeErrorHistory), [], 2);
    maximumFutureError = flip(cummax(flip(maximumError)));
    convergenceIndex = find(maximumFutureError <= threshold, 1, 'first');
    if isempty(convergenceIndex)
        convergenceIndex = size(edgeErrorHistory, 1);
    end
end

function saveDesiredGraphFigure(caseFolder, desiredPositions, adjacency, graphName, visualization)
    figureHandle = figure('Visible','off','Color','w');
    hold on; grid on; axis equal; box on;
    drawGraph(desiredPositions, adjacency, true, visualization);
    title(sprintf('Desired regular polygon and %s graph', strrep(graphName,'_',' ')), ...
        'FontSize', visualization.titleFontSize);
    xlabel('x', 'FontSize', visualization.fontSize); ylabel('y', 'FontSize', visualization.fontSize);
    set(gca, 'FontSize', visualization.fontSize, 'LineWidth', visualization.graphLineWidth);
    savefig(figureHandle, fullfile(caseFolder, 'desired_shape.fig'));
    exportgraphics(figureHandle, fullfile(caseFolder, 'desired_shape.png'), 'Resolution', 150);
    close(figureHandle);
end

function saveTrajectoryFigure(caseFolder, trajectory, adjacency, graphName, visualization)
    nAgents = size(trajectory, 2);
    colors = lines(nAgents);
    finalPositions = squeeze(trajectory(end,:,:));
    figureHandle = figure('Visible','off','Color','w');
    hold on; grid on; axis equal; box on;
    for i = 1:nAgents
        trace = squeeze(trajectory(:,i,:));
        plot(trace(:,1), trace(:,2), 'Color', colors(i,:), 'LineWidth', visualization.lineWidth, ...
            'DisplayName', sprintf('Agent %d', i));
    end
    drawGraph(finalPositions, adjacency, true, visualization);
    title(sprintf('Agent trajectories and final graph: %s', strrep(graphName,'_',' ')), ...
        'FontSize', visualization.titleFontSize);
    xlabel('x', 'FontSize', visualization.fontSize); ylabel('y', 'FontSize', visualization.fontSize);
    legend('Location','eastoutside', 'FontSize', visualization.legendFontSize);
    set(gca, 'FontSize', visualization.fontSize, 'LineWidth', visualization.graphLineWidth);
    savefig(figureHandle, fullfile(caseFolder, 'trajectories.fig'));
    exportgraphics(figureHandle, fullfile(caseFolder, 'trajectories.png'), 'Resolution', 150);
    close(figureHandle);
end

function saveDistanceErrorFigure(caseFolder, edgeErrorHistory, edgePairs, dt, graphName, visualization)
    figureHandle = figure('Visible','off','Color','w');
    hold on; grid on; box on;
    time = (0:size(edgeErrorHistory,1)-1)' * dt;
    colors = lines(size(edgePairs,1));
    for edgeIndex = 1:size(edgePairs,1)
        plot(time, edgeErrorHistory(:,edgeIndex), 'Color', colors(edgeIndex,:), ...
            'LineWidth', visualization.lineWidth, ...
            'DisplayName', sprintf('e_{%d,%d}', edgePairs(edgeIndex,1), edgePairs(edgeIndex,2)));
    end
    yline(0, 'k--', 'LineWidth', visualization.graphLineWidth, 'HandleVisibility','off');
    title(sprintf('Squared-distance error convergence: %s', strrep(graphName,'_',' ')), ...
        'FontSize', visualization.titleFontSize);
    xlabel('Time (s)', 'FontSize', visualization.fontSize);
    ylabel('||p_i-p_j||^2 - d_{ij}^2', 'FontSize', visualization.fontSize);
    legend('Location','eastoutside', 'FontSize', visualization.legendFontSize);
    set(gca, 'FontSize', visualization.fontSize, 'LineWidth', visualization.graphLineWidth);
    savefig(figureHandle, fullfile(caseFolder, 'distance_error_convergence.fig'));
    exportgraphics(figureHandle, fullfile(caseFolder, 'distance_error_convergence.png'), 'Resolution', 150);
    close(figureHandle);
end

function saveAnimation(caseFolder, trajectory, desiredPositions, adjacency, graphName, dt, frameStride, playbackRate, videoQuality, visualization)
    video = VideoWriter(fullfile(caseFolder, 'animation.mp4'), 'MPEG-4');
    video.FrameRate = playbackRate/(dt*frameStride);
    video.Quality = videoQuality;
    open(video);
    gifPath = fullfile(caseFolder, 'animation.gif');
    gifDelay = dt*frameStride/playbackRate;
    figureHandle = figure('Visible','off','Color','w','Position',[100 100 960 540]);
    allPositions = [reshape(trajectory, [], 2); desiredPositions];
    limits = axisLimits(allPositions);
    frameIndices = unique([1:frameStride:size(trajectory,1), size(trajectory,1)]);
    for step = frameIndices
        clf(figureHandle); hold on; grid on; axis equal; box on;
        axis(limits);
        currentPositions = squeeze(trajectory(step,:,:));
        colors = lines(size(trajectory,2));
        trajectoryHandles = gobjects(size(trajectory,2), 1);
        for agentIndex = 1:size(trajectory,2)
            % Preserve a time-by-coordinate matrix even on the first frame.
            trace = reshape(trajectory(1:step,agentIndex,:), step, 2);
            trajectoryHandles(agentIndex) = plot(trace(:,1), trace(:,2), ...
                'Color', colors(agentIndex,:), ...
                'LineWidth', visualization.lineWidth, ...
                'DisplayName', sprintf('Agent %d', agentIndex));
        end
        drawGraph(currentPositions, adjacency, true, visualization);
        title(sprintf('%s graph, t = %.2f s', strrep(graphName,'_',' '), (step-1)*dt), ...
            'FontSize', visualization.titleFontSize);
        xlabel('x', 'FontSize', visualization.fontSize); ylabel('y', 'FontSize', visualization.fontSize);
        legend(trajectoryHandles, arrayfun(@(agentIndex) sprintf('Agent %d', agentIndex), ...
            1:size(trajectory,2), 'UniformOutput', false), 'Location','eastoutside', ...
            'FontSize', visualization.legendFontSize);
        set(gca, 'FontSize', visualization.fontSize, 'LineWidth', visualization.graphLineWidth);
        frame = getframe(figureHandle);
        writeVideo(video, frame);
        [indexedFrame, colorMap] = rgb2ind(frame2im(frame), 128);
        if step == frameIndices(1)
            imwrite(indexedFrame, colorMap, gifPath, 'gif', 'LoopCount', Inf, ...
                'DelayTime', gifDelay);
        else
            imwrite(indexedFrame, colorMap, gifPath, 'gif', 'WriteMode', 'append', ...
                'DelayTime', gifDelay);
        end
    end
    close(video); close(figureHandle);
end

function drawGraph(positions, adjacency, annotate, visualization)
    for i = 1:size(positions,1)
        for j = i+1:size(positions,1)
            if adjacency(i,j) ~= 0
                plot([positions(i,1), positions(j,1)], [positions(i,2), positions(j,2)], ...
                    'Color',[0.35 0.35 0.35], 'LineWidth', visualization.graphLineWidth, 'HandleVisibility','off');
            end
        end
    end
    plot(positions(:,1), positions(:,2), 'ko', 'MarkerFaceColor','w', ...
        'MarkerSize', visualization.markerSize, 'HandleVisibility','off');
    if annotate
        for i = 1:size(positions,1)
            text(positions(i,1)+0.04, positions(i,2)+0.04, sprintf('v%d',i), ...
                'FontWeight','bold', 'FontSize', visualization.annotationFontSize, 'HandleVisibility','off');
        end
    end
end

function limits = axisLimits(positions)
    minimum = min(positions, [], 1); maximum = max(positions, [], 1);
    padding = max(0.5, 0.15*max(maximum-minimum));
    limits = [minimum(1)-padding, maximum(1)+padding, minimum(2)-padding, maximum(2)+padding];
end

function resetResultFolder(resultFolder)
    if isfolder(resultFolder)
        generatedFiles = [dir(fullfile(resultFolder, '**', '*.fig')); ...
            dir(fullfile(resultFolder, '**', '*.png')); ...
            dir(fullfile(resultFolder, '**', '*.mp4')); ...
            dir(fullfile(resultFolder, '**', '*.gif')); ...
            dir(fullfile(resultFolder, '**', '*.avi'))];
        for fileIndex = 1:numel(generatedFiles)
            delete(fullfile(generatedFiles(fileIndex).folder, generatedFiles(fileIndex).name));
        end
    else
        mkdir(resultFolder);
    end
end
