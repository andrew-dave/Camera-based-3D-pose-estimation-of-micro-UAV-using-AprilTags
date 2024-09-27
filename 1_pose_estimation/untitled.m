% Define constants
tag_size = 0.152; % Size of each AprilTag square in meters
tag_spacing = 0.152; % Spacing between tags in meters
column_spacing_exceptions = [3, 6]; % Columns with different spacing

% Initialize matrix to store corner coordinates of all tags
all_tag_corners = cell(12, 9);

% Compute location of every corner of every tag in the world frame
for i = 1:12 % Rows
    for j = 1:9 % Columns
        % Compute coordinates of top left corner of current tag
        x_tl = (j - 1) * (tag_size + tag_spacing);
        y_tl = (i - 1) * (tag_size + tag_spacing);
        
        % Adjust spacing for columns with exceptions
        if ismember(j, column_spacing_exceptions)
            x_tl = x_tl + (tag_spacing - 0.026); % Adjust spacing
        end
        
        % Compute coordinates of other corners of current tag
        x_tr = x_tl + tag_size;
        y_tr = y_tl;
        x_bl = x_tl;
        y_bl = y_tl + tag_size;
        x_br = x_tl + tag_size;
        y_br = y_tl + tag_size;
        
        % Store corner coordinates of current tag in world frame
        all_tag_corners{i, j} = [x_tl, y_tl; x_tr, y_tr; x_bl, y_bl; x_br, y_br];
    end
end

% Example: Display corner coordinates of tag in row 3, column 5
disp(all_tag_corners{3, 5});