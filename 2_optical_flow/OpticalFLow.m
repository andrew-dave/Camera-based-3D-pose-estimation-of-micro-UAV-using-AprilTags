%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 4;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
% Camera Intrinsic matrix k
k = [311.0520, 0, 201.8724; 0, 311.3885, 113.6210;0, 0, 1];

% Body to camera homogenous transformation Matrix
H_B2C = [0.7071, -0.7071, 0, 0.0283; -0.7071, -0.7071, 0, -0.0283; 0, 0, -1, 0.03; 0, 0, 0, 1];

% Extract camera to body rotation matrix
R_B2C = H_B2C(1:3,1:3);

% Initialise the skew symmetric matric for the camera to body translation
SkewT_C2B=[0,0.03,0;-0.03,0,0.04;0,-0.04,0];

% Passing time values through a LPF to filter noise 
t = sgolayfilt([sampledData.t],1,101);

% Selecting the number of most relevant/strongest points
samplepts = 100;

% RANSAC Flag
ransacFlg = 1;

%% Computing Optical flow, Velocity and Omega
for n = 2:length(sampledData)
    % Load the previous and current images
    Ip = sampledData(n-1).img;
    Ic = sampledData(n).img;
    % Calculate dt
    d_t = t(n) - t(n-1);
    
    % Detect good points from the previous image
    Ip_detected = detectMinEigenFeatures(Ip).selectStrongest(samplepts).Location;
    
    % Initalize the tracker to the last frame
    pointTracker = vision.PointTracker('MaxBidirectionalError',1);
    initialize(pointTracker, Ip_detected, Ip);
    
    % Find the location of the next points;
    [Ic_tracked, ~, ~] = pointTracker(Ic);
    
    % normalisation with respect to camera intrinsic
    Ip_detected = k \ [Ip_detected, ones(samplepts,1)]';
    Ip_detected = [Ip_detected(1,:); Ip_detected(2,:)];
    
    Ic_tracked = k \ [Ic_tracked, ones(samplepts,1)]';
    Ic_tracked = [Ic_tracked(1,:); Ic_tracked(2,:)];
    
    %% Compute and initialise the prerequisites for calculating linear and angular velocity
    p_dot = [];
    [position, orientation, R_c2w] = estimatePose(sampledData, n);
    
    R_B2W = R_B2C'*R_c2w(1:3,1:3);
    Trans = [R_B2W', zeros(3); zeros(3) R_B2W'] * [R_B2C -R_B2C*SkewT_C2B;zeros(3,3) R_B2C];
    
    % Calculate the value of Z
    P = -1 * (H_B2C(1:3, 1:3) * position);
    Z = P(3);
   
    H = [];
    VW = [];
    
    % Change ransacFlg to 1 if Outlier rejection must be performed 
    if ransacFlg == 1
        % Calculating Pdot for current image set
        for i = 1:samplepts
            p_dot_i = [(Ic_tracked(1,i) - Ip_detected(1,i)); (Ic_tracked(2,i) - Ip_detected(2,i))]./d_t;
            p_dot = vertcat(p_dot, p_dot_i);
        end
        %Initialise RANSAC hyperparameter
        e= 0.8;
        % Transform the computed velocites from camera to body frame 
        VW = Trans * velocityRANSAC(p_dot, Ip_detected, Z, R_c2w, e);

    % Change ransacFlg to 0 if no outliers need to be rejected 
    else
        for i = 1:samplepts
            x = Ip_detected(1,i);
            y = Ip_detected(2,i);
            p_dot_i = [(Ic_tracked(1,i) - Ip_detected(1,i)); (Ic_tracked(2,i) - Ip_detected(2,i))]./d_t;
            
            A_p = [-1/Z, 0, x/Z; 0, -1/Z, y/Z];
            B_p = [(x*y), -(1 + (x^2)), y; (1 + (y^2)), (-x*y), -x];
            H = vertcat(H, [((1/Z) .* A_p), B_p]);
            p_dot = vertcat(p_dot, p_dot_i);
        end
        % Transform the computed velocites from camera to body frame 
        VW = Trans * (pinv(H) * p_dot);
    end
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV 
    estimatedV(:,n) = VW; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

%estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 11);
estimatedV=sgolayfilt(double(estimatedV'),1,41)';

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
