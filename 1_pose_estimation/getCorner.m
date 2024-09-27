function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
%% Input Parameter Description
% id = List of all the AprilTag ids detected in the current image(data)

%% Output Parameter Description
% tags = Structure array containing the coordinates of the center and corners of each detected AprilTag

% Define constants
tag_size = 0.152; % Size of each AprilTag square in centimeters
tag_spacing = 0.152; % Default spacing between tags in centimeters
mat_tags = [0, 12, 24, 36, 48, 60, 72, 84,  96;
    1, 13, 25, 37, 49, 61, 73, 85,  97;
    2, 14, 26, 38, 50, 62, 74, 86,  98;
    3, 15, 27, 39, 51, 63, 75, 87,  99;
    4, 16, 28, 40, 52, 64, 76, 88, 100;
    5, 17, 29, 41, 53, 65, 77, 89, 101;
    6, 18, 30, 42, 54, 66, 78, 90, 102;
    7, 19, 31, 43, 55, 67, 79, 91, 103;
    8, 20, 32, 44, 56, 68, 80, 92, 104;
    9, 21, 33, 45, 57, 69, 81, 93, 105;
    10, 22, 34, 46, 58, 70, 82, 94, 106;
    11, 23, 35, 47, 59, 71, 83, 95, 107];

% Initialize structure array
%res = struct('id', {}, 'center', {}, 'corners', {});
res=[];
% Iterate over each detected tag ID
for k = 1:length(id)
    % Get current tag ID
    current_id = id(k);

    % Compute row and column of current tag based on tag ID
    [row,column] = find(mat_tags == current_id);

    % Compute coordinates of top left corner of current tag
    x_tl = (row - 1) * (tag_size + tag_spacing);
    if(column<=3)
        y_tl = (column - 1) * (tag_size + tag_spacing);
    end
    if(column>=4 || column<=6)
        y_tl = 0.938 + (column - 4) * (tag_size + tag_spacing);
    end
    if(column>=7)
        y_tl = 1.876 + (column - 7) * (tag_size + tag_spacing);
    end

    % Compute coordinates of other corners of current tag
    x_tr = x_tl;
    y_tr = y_tl + tag_size;
    x_bl = x_tl + tag_size;
    y_bl = y_tl;
    x_br = x_tl + tag_size;
    y_br = y_tl + tag_size;

    % Compute center coordinates of current tag
    center_x = (x_tl + x_tr + x_bl + x_br) / 4;
    center_y = (y_tl + y_tr + y_bl + y_br) / 4;

    % Store coordinates in structure array
    %res(k).id = current_id;
    %res(k).center = [center_x, center_y]; % Center coordinates
    %res(k).corners = {[center_x, center_y], [x_bl, y_bl], [x_br, y_br], [x_tr, y_tr], [x_tl, y_tl]}; % Center coordinates, bottom left, bottom right, top right, top left
    
    %Store Coordinates in a 2x60 double
    curtag_corners = [[center_x; center_y], [x_bl;y_bl], [x_br;y_br], [x_tr;y_tr], [x_tl;y_tl]];
    res = [res, curtag_corners];
end
end
