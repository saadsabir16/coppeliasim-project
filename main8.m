clear all; clc; close all;
addpath(genpath(cd));

%% Connect to CoppeliaSim
vrep = remApi('remoteApi');
vrep.simxFinish(-1);  % close all open connections
id = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if id < 0
    disp('Failed to connect MATLAB to CoppeliaSim');
    vrep.delete();
    return;
else
    fprintf('Connection %d to remote API server is open.\n', id);
end

%% Get Handles
fprintf('\n=== Getting Object Handles ===\n');
[err, camhandle]   = vrep.simxGetObjectHandle(id, 'Vision_sensor', vrep.simx_opmode_oneshot_wait);
[err, basehandle]  = vrep.simxGetObjectHandle(id, 'UR5',           vrep.simx_opmode_oneshot_wait);
[err, goalhandle]  = vrep.simxGetObjectHandle(id, 'goalPose',      vrep.simx_opmode_oneshot_wait);
[err, tiphandle]   = vrep.simxGetObjectHandle(id, 'tip',           vrep.simx_opmode_oneshot_wait);

% Box handles
[err, blueBoxHandle]  = vrep.simxGetObjectHandle(id, 'blueBox',  vrep.simx_opmode_oneshot_wait);
[err, redBoxHandle]   = vrep.simxGetObjectHandle(id, 'redBox',   vrep.simx_opmode_oneshot_wait);
[err, greenBoxHandle] = vrep.simxGetObjectHandle(id, 'greenBox', vrep.simx_opmode_oneshot_wait);

%% Get Fixed Positions
fprintf('\n=== Getting Fixed Positions ===\n');

[err, basePos]  = vrep.simxGetObjectPosition(id, basehandle, -1, vrep.simx_opmode_oneshot_wait);
fprintf('Base Position (WORLD): [%.3f, %.3f, %.3f]\n', basePos(1), basePos(2), basePos(3));

[err, cam2base] = vrep.simxGetObjectPosition(id, camhandle, basehandle, vrep.simx_opmode_oneshot_wait);
fprintf('Camera to Base (BASE frame): [%.3f, %.3f, %.3f]\n', cam2base(1), cam2base(2), cam2base(3));

% Box positions in WORLD coordinates
[err, blueBoxPos]  = vrep.simxGetObjectPosition(id, blueBoxHandle,  -1, vrep.simx_opmode_oneshot_wait);
[err, redBoxPos]   = vrep.simxGetObjectPosition(id, redBoxHandle,   -1, vrep.simx_opmode_oneshot_wait);
[err, greenBoxPos] = vrep.simxGetObjectPosition(id, greenBoxHandle, -1, vrep.simx_opmode_oneshot_wait);

% Set drop height ABOVE boxes (not inside)
dropHeight = 0.20;  % 20 cm above box (tune if needed)
blueBoxPos(3)  = dropHeight;
redBoxPos(3)   = dropHeight;
greenBoxPos(3) = dropHeight;

fprintf('Blue Box Drop Position : [%.3f, %.3f, %.3f]\n', blueBoxPos(1),  blueBoxPos(2),  blueBoxPos(3));
fprintf('Red Box Drop Position  : [%.3f, %.3f, %.3f]\n', redBoxPos(1),   redBoxPos(2),   redBoxPos(3));
fprintf('Green Box Drop Position: [%.3f, %.3f, %.3f]\n', greenBoxPos(1), greenBoxPos(2), greenBoxPos(3));

%% Get Image and Detect Cubes
fprintf('\n=== Capturing Image ===\n');
[err, resolution, img] = vrep.simxGetVisionSensorImage2(id, camhandle, 0, vrep.simx_opmode_oneshot_wait);

if err == 0
    figure('Name', 'Vision Sensor Image');
    imshow(img);
    title('Vision Sensor - Detected Cubes');
    pause(1);
else
    error('Failed to get image from vision sensor');
end

% resolution from CoppeliaSim: [width, height]
imgWidth  = double(resolution(1));
imgHeight = double(resolution(2));

% Camera FOV and workspace width/height at cube level
% Assuming a 60° vertical FOV (pi/3); half-angle is pi/6
camHeight   = cam2base(3);                % camera height above base XY plane (approx.)
realWidth   = 2 * tan(pi/6) * camHeight;  % physical width at cube plane
realHeight  = realWidth;                  % assuming square FOV (adjust if needed)

%% Color Detection
fprintf('\n=== Detecting Colored Cubes ===\n');
cubeData = detectColoredCubes(img);

if isempty(cubeData)
    error('No cubes detected!');
end

cubePositions = [];
cubeColors    = {};

figure('Name', 'Detected Cubes');
imshow(img);
hold on;

% Approximate cube center height in BASE frame (tune if needed)
zcube2base = 0.025;  % 2.5 cm above base XY plane

for i = 1:size(cubeData, 1)
    xpix      = cubeData(i, 1);  % column index
    ypix      = cubeData(i, 2);  % row index
    colorCode = cubeData(i, 3);

    % Image center
    cx = imgWidth  / 2;
    cy = imgHeight / 2;

    % Pixel -> camera frame (meters)
    % X: right positive, using width
    xcam = (xpix - cx) * (realWidth  / imgWidth);

    % Y: image Y increases DOWN, so invert relative to center
    ycam = (cy - ypix) * (realHeight / imgHeight);

    % Camera frame -> base frame (assuming aligned axes)
    xcube2base = cam2base(1) + xcam;
    ycube2base = cam2base(2) + ycam;

    % Base frame -> world frame
    xcube_world = basePos(1) + xcube2base;
    ycube_world = basePos(2) + ycube2base;
    zcube_world = basePos(3) + zcube2base;

    cubePositions = [cubePositions; xcube_world, ycube_world, zcube_world];

    switch colorCode
        case 1
            colorName = 'blue';
        case 2
            colorName = 'red';
        case 3
            colorName = 'green';
        otherwise
            colorName = 'unknown';
    end
    cubeColors{end+1} = colorName;

    % Plot detections on image
    plot(xpix, ypix, 'o', 'MarkerSize', 15, 'LineWidth', 3, ...
        'Color', getColorRGB(colorName));
    text(xpix, ypix - 20, colorName, 'Color', 'white', 'FontSize', 12, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
        'BackgroundColor', 'black');

    fprintf('Detected %s cube at WORLD: [%.3f, %.3f, %.3f]\n', ...
        colorName, xcube_world, ycube_world, zcube_world);
end

hold off;
pause(2);

%% DIRECT Pick and Place for Each Cube
fprintf('\n=== Starting Direct Pick and Place ===\n');

for i = 1:size(cubePositions, 1)
    fprintf('\n━━━ Cube %d/%d: %s ━━━\n', i, size(cubePositions, 1), upper(cubeColors{i}));

    pickPos = cubePositions(i, :);

    % Choose drop position based on color
    switch cubeColors{i}
        case 'blue'
            dropPos = blueBoxPos;
        case 'red'
            dropPos = redBoxPos;
        case 'green'
            dropPos = greenBoxPos;
        otherwise
            fprintf('Unknown color, skipping cube.\n');
            continue;
    end

    success = directPickAndPlace(vrep, id, goalhandle, pickPos, dropPos);

    if success
        fprintf('✓ %s cube sorted successfully!\n', upper(cubeColors{i}));
    else
        fprintf('✗ Failed to sort %s cube\n', cubeColors{i});
    end

    fprintf('Waiting 3 seconds before next cube...\n');
    pause(3);
end

fprintf('\n═══ ALL CUBES SORTED! ═══\n');

%% Cleanup
vrep.simxFinish(id);
vrep.delete();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cubeData = detectColoredCubes(img)
    cubeData = [];
    imgHSV = rgb2hsv(img);

    % Basic HSV thresholds for blue, red, green (tune to your scene!)
    blueMask = (imgHSV(:,:,1) > 0.5 & imgHSV(:,:,1) < 0.7) & ...
               (imgHSV(:,:,2) > 0.3 & imgHSV(:,:,3) > 0.3);

    redMask  = ((imgHSV(:,:,1) < 0.05 | imgHSV(:,:,1) > 0.95) & ...
               (imgHSV(:,:,2) > 0.3 & imgHSV(:,:,3) > 0.3));

    greenMask = (imgHSV(:,:,1) > 0.25 & imgHSV(:,:,1) < 0.45) & ...
                (imgHSV(:,:,2) > 0.3 & imgHSV(:,:,3) > 0.3);

    colors = {blueMask, redMask, greenMask};

    for c = 1:3
        mask = colors{c};
        mask = bwareaopen(mask, 50);
        mask = imfill(mask, 'holes');
        stats = regionprops(mask, 'Centroid', 'Area');

        for k = 1:length(stats)
            if stats(k).Area > 100
                centroid = stats(k).Centroid;  % [x, y] = [col, row]
                cubeData = [cubeData; centroid(1), centroid(2), c];
            end
        end
    end
end

function success = directPickAndPlace(vrep, id, goalhandle, pickPos, dropPos)
    success = false;

    try
        % Clear any old movementComplete signal
        vrep.simxClearIntegerSignal(id, 'movementComplete', vrep.simx_opmode_oneshot_wait);

        %% STEP 1: Move ABOVE cube
        aboveCube      = pickPos;
        aboveCube(3)   = pickPos(3) + 0.10;  % 10 cm above cube center

        fprintf('  → Moving above cube [%.3f, %.3f, %.3f]...\n', ...
            aboveCube(1), aboveCube(2), aboveCube(3));
        vrep.simxSetObjectPosition(id, goalhandle, -1, aboveCube, vrep.simx_opmode_oneshot_wait);
        vrep.simxSetIntegerSignal(id, 'moveCommand', 1, vrep.simx_opmode_oneshot_wait);

        if ~waitForMovementComplete(vrep, id, 30)
            warning('Failed to reach above cube');
            return;
        end
        pause(1.0);

        %% STEP 2: Descend to cube
        fprintf('  → Descending to pick...\n');
        vrep.simxSetObjectPosition(id, goalhandle, -1, pickPos, vrep.simx_opmode_oneshot_wait);
        vrep.simxSetIntegerSignal(id, 'moveCommand', 2, vrep.simx_opmode_oneshot_wait);

        if ~waitForMovementComplete(vrep, id, 30)
            warning('Failed to reach pick position');
            return;
        end
        pause(0.5);

        %% STEP 3: Close gripper
        fprintf('  → Gripping cube...\n');
        vrep.simxSetIntegerSignal(id, 'gripperCommand', 1, vrep.simx_opmode_oneshot_wait);
        pause(2.0);

        %% STEP 4: Lift
        liftPos    = pickPos;
        liftPos(3) = pickPos(3) + 0.15;

        fprintf('  → Lifting cube...\n');
        vrep.simxSetObjectPosition(id, goalhandle, -1, liftPos, vrep.simx_opmode_oneshot_wait);
        vrep.simxSetIntegerSignal(id, 'moveCommand', 3, vrep.simx_opmode_oneshot_wait);

        if ~waitForMovementComplete(vrep, id, 30)
            warning('Failed to lift');
            return;
        end
        pause(1.0);

        %% STEP 5: Move to drop position (above box)
        fprintf('  → Moving to drop position [%.3f, %.3f, %.3f]...\n', ...
            dropPos(1), dropPos(2), dropPos(3));
        vrep.simxSetObjectPosition(id, goalhandle, -1, dropPos, vrep.simx_opmode_oneshot_wait);
        vrep.simxSetIntegerSignal(id, 'moveCommand', 4, vrep.simx_opmode_oneshot_wait);

        if ~waitForMovementComplete(vrep, id, 30)
            warning('Failed to reach drop position');
            return;
        end
        pause(1.0);

        %% STEP 6: Open gripper to release cube
        fprintf('  → Releasing cube...\n');
        vrep.simxSetIntegerSignal(id, 'gripperCommand', 0, vrep.simx_opmode_oneshot_wait);
        pause(2.0);

        success = true;
        fprintf('  ✓ Pick and place complete!\n');

    catch ME
        fprintf('  ✗ Error: %s\n', ME.message);
        success = false;
    end
end

function success = waitForMovementComplete(vrep, id, timeout)
    startTime = tic;
    success   = false;

    while toc(startTime) < timeout
        [err, signal] = vrep.simxGetIntegerSignal(id, 'movementComplete', vrep.simx_opmode_oneshot_wait);

        if err == 0 && signal == 1
            vrep.simxClearIntegerSignal(id, 'movementComplete', vrep.simx_opmode_oneshot_wait);
            pause(0.5);
            success = true;
            return;
        end

        pause(0.2);
    end

    if ~success
        warning('Movement timeout after %d seconds!', timeout);
    end
end

function rgb = getColorRGB(colorName)
    switch colorName
        case 'blue',  rgb = [0, 0, 1];
        case 'red',   rgb = [1, 0, 0];
        case 'green', rgb = [0, 1, 0];
        otherwise,    rgb = [1, 1, 1];
    end
end
