function ax = plotTransforms(translations, rotations, varargin)
%plotTransforms Plot 3D transforms described by translations and rotations
%   AX = PLOTTRANSFORMS(TRANSLATIONS, ROTATIONS) draws transform frames
%   according to the given TRANSLATIONS and ROTATIONS relative to the
%   inertial frame. The z-axis of the plot always points upward
%   regardless the Z direction of the inertial frame. TRANSLATIONS is
%   an Nx3 matrix representing xyz-position of the transform relative
%   to the inertial frame. ROTATIONS is an Nx1 quaternion vector or Nx4
%   numeric matrix representing the rotations that transforms points
%   from the transform frame to the inertial frame.
%
%   AX = PLOTTRANSFORMS(___, Name, Value) provides additional options
%   specified by Name-Value pair arguments. Available argument names:
%
%      'FrameSize'            - Positive numeric that determines the
%                               size of the plotted transform frame and
%                               the mesh attached to the transform.
%
%      'InertialZDirection'    - A string that indicates whether the
%                               z-axis of the inertial frame points
%                               upward or downward. Must be either
%                               "up" or "down". The default value is
%                               "up"
%
%      'MeshColor'            - Either a string or an RGB triplet that
%                               describes the color of the plotted
%                               mesh. Default is [1 0 0] or "red".
%
%      'MeshFilePath'         - File path to the mesh file to be
%                               attached to the transform frames. The
%                               file path can be absolute path,
%                               relative path or on the MATLAB path.
%
%      'View'                 - A string or 3-element vector indicating the
%                               desired plot view. Valid options are "2D",
%                               "3D", or the 3-element vector [x y z] that
%                               sets the view angle in Cartesian
%                               coordinates. The magnitude of vector Z, Y,
%                               Z is ignored. The default value is "3D".
%
%      'Parent'               - A handle to an axis upon which this
%                               plot would be drawn.
%   Example:
%      % plot three multirotor UAV with different poses
%      plotTransforms(eye(3), [eye(3), zeros(3,1)], 'MeshFilePath', 'multirotor.stl')
%      light
%
%      % plot three fixed-wing UAV with different poses
%      plotTransforms(eye(3), [eye(3), zeros(3,1)], 'MeshFilePath', 'fixedwing.stl')
%      light
%
%      % plot three mobile robots with different poses
%      plotTransforms(eye(3), [eye(3), zeros(3,1)], 'MeshFilePath', 'groundvehicle.stl')
%      light
%
%   See also: roboticsAddons

%   Copyright 2018-2019 The MathWorks, Inc.

    narginchk(2, 14);

    % validate translations
    validateattributes(translations, {'numeric'}, {'2d', 'ncols', 3, 'nonempty'}, 'plotTransforms', 'translations');

    % validate rotation using internal quaternion validation utility
    robotics.internal.validation.validateQuaternion(rotations, 'plotTransforms', 'rotations');

    if (isa(rotations, 'quaternion') && size(rotations, 2) ~= 1)
        error(message('shared_robotics:robotcore:plotTransforms:RotationMustBeColumn'));
    end

    if (size(translations, 1)~=size(rotations, 1))
        error(message('shared_robotics:robotcore:plotTransforms:MismatchTranslationRotation'));
    end

    % convert orientations to quaternions if necessary
    if ~isa(rotations, 'quaternion')
        rotations = quaternion(rotations);
    end

    % setup input parser
    p = inputParser;

    % optional inputs
    addParameter(p, 'FrameSize', 1, ...
                 @(x)validateattributes(x, {'numeric'}, {'scalar','positive'}, 'plotTransforms', 'FrameSize'));
    addParameter(p, 'Parent', [], ...
                 @(x)validateattributes(x, {'matlab.graphics.axis.Axes'}, {'nonempty', 'scalar'}, 'plotTransforms', 'Parent'));
    
    % postpone full validation for view, string, file path, and mesh color
    addParameter(p, 'View', [], @(x)validateattributes(x, {'string', 'char', 'numeric'}, {'nonempty'}, 'plotTransforms', 'View'));
    addParameter(p, 'MeshFilePath', '', @(x)validateattributes(x, {'string', 'char'}, {'scalartext'}, 'plotTransforms', 'MeshFilePath'));
    addParameter(p, 'InertialZDirection', 'up', @(x)validateattributes(x, {'string', 'char'}, {'scalartext'}, 'plotTransforms', 'InertialZDirection'));
    addParameter(p, 'MeshColor', [1 0 0]);

    % parse inputs
    parse(p, varargin{:})

    % validate mesh file path if it is not empty
    if ~isempty(p.Results.MeshFilePath)
        meshPath = robotics.internal.validation.findFilePath(convertStringsToChars(p.Results.MeshFilePath), 'plotTransforms', 'MeshFilePath');
    else
        meshPath = '';
    end

    % validate inertial z direction
    inertialZDirection = validatestring(p.Results.InertialZDirection, {'up', 'down'}, 'plotTransforms', 'InertialZDirection');

    % prepare ax for plotting
    if isempty(p.Results.Parent)
        parentAx = newplot;
    else
        parentAx = p.Results.Parent;
    end

    % prepare transform painter
    painter = robotics.core.internal.visualization.TransformPainter(parentAx, meshPath, false);
    painter.Color = p.Results.MeshColor;
    painter.Size = p.Results.FrameSize;
    painter.InertialZDownward = strcmp(inertialZDirection, 'down');

    % paint transform at given translations and orientations
    for i = 1:size(translations, 1)
        painter.paintAt(translations(i, :), rotations(i));
    end

    % optionally output the plot ax
    if nargout == 1
        ax = parentAx;
    end
    
    % Optionally change the view if the user has provided that input
    if ~isempty(p.Results.View)
        viewAzEl = validateViewInput(p.Results.View);
        view(parentAx, viewAzEl(1), viewAzEl(2));
    end
end
    
function selectedView = validateViewInput(inputViewValue)
%validateViewInput Validate the input the the "View" Name/Value pair
%   The earlier check verifies that the user-specified input is nonempty,
%   but the default value (no input provided) is empty. This function
%   converts the input options to a 2-element vector [AZ EL] containing the
%   azimuth and elevation angles that are passed to the view command
%   downstream.

    if isnumeric(inputViewValue) && ~isempty(inputViewValue)
        % If the input is numeric, extract azimuth and elevation angles
        validateattributes(inputViewValue, {'numeric'}, {'vector', 'numel', 2, 'finite'}, 'plotTransforms', 'View');
        selectedView = inputViewValue;
    else
        % If the input is a string, it should specify 2D or 3D. Convert
        % these two azimuth and elevation angles
        viewStr = validatestring(inputViewValue, {'2D', '3D'}, 'plotTransforms', 'View');
        if strcmp(viewStr, '2D')
            [az, el] = view(2);
        else
            [az, el] = view(3);
        end
        selectedView = [az el];
    end

end