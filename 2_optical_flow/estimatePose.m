function [position, orientation, R_c2w] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function

%% Input Parameter Defination
% data = the entire data loaded in the current dataset
% t = index of the current data in the dataset

%Camera intrinsic matrix
k = [311.0520        0 201.8724;
    0 311.3885 113.6210;
    0        0        1];
%Res to get corners and center of apriltags in the given mat
res = getCorner(data(t).id);

%Retrieving the center and tagcorners from data in current timestamp
img_corners= [];
for j = 1:length(data(t).id)
    itag_corners = [data(t).p0(:,j), data(t).p1(:, j), data(t).p2(:, j), data(t).p3(:, j), data(t).p4(:, j);];
    img_corners = [img_corners, itag_corners];
end

% Construct A matrix for current timestamp
num_points = size(res, 2);
A = zeros(2*num_points, 9);

for i = 1:num_points
    x_w = res(1, i);
    y_w = res(2, i);
    x_i = img_corners(1, i);
    y_i = img_corners(2, i);

    A(2*i-1, :) = [x_w, y_w, 1, 0, 0, 0, -x_i*x_w, -x_i*y_w, -x_i];
    A(2*i, :) = [0, 0, 0, x_w, y_w, 1, -y_i*x_w, -y_i*y_w, -y_i];
end

% Perform SVD on A matrix
[~, ~, V] = svd(A);

% Extract transformation matrix
h = reshape(V(:, end), 3, 3)';
h = h/V(9,9);
t_mat = inv(k)*h;

%Obtaining the Rotations in each axes
R1 = t_mat(:,1);
R2 = t_mat(:,2);
R3 = cross(R1,R2);

%Calculating translation
T = t_mat(:,3)/ norm(t_mat(:,1));

%Calculating solution for rotation
[U, ~, V] = svd([R1 R2 R3]);
D = [1 0 0; 0 1 0; 0 0 det(U*V')];
R = U * D * V';
H_cam = [R T; zeros(1,3) 1];

%Tranforming from camera frame to drone(body) frame
%d2c_rot = rotz(-45)*rotx(180);
%d2c_trans = d2c_rot*[0.04; 0; -0.03];
%T_drone = [d2c_rot d2c_trans; zeros(1,3) 1];
T_drone = [0.7071, -0.7071, 0, -0.04 ; -0.7071, -0.7071, 0, 0 ; 0, 0, -1, 0.03 ; 0, 0, 0, 1];
H_drone = inv(H_cam)*inv(T_drone);

%Obtaining Position and Orientation
% position = translation vector representing the position of the
% drone(body) in the world frame in the current time, in the order XYZ
position = H_drone(1:3,4);
% orientation = euler angles representing the orientation of the
% drone(body) in the world frame in the current time, in the order XYZ
orientation = rotm2eul(H_drone(1:3,1:3),'ZYX');
%R_c2w = Rotation which defines camera to world frame
R_c2w = R;   
   
end