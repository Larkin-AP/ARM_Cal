% test_IK
% Given matrix T_08
T_08 = [0, 0, 1, 9.000;
        -1, 0, 0, -668.0000;
         0, -1.0000, 0, 158.3000;
         0, 0, 0, 1.0000];

% Initial positions
px0 = T_08(1,4);
py0 = T_08(2,4);
pz0 = T_08(3,4);

% Trajectory parameters
t = 0:0.1:10;
px = px0-10 + 10 * sin(t);
py = py0+20 + 10 * sin(2 * t);
pz = pz0-10 + 10 * cos(t);

phi = 0; % Initial phi angle

% Initialize theta output storage
theta_all = zeros(length(t), 7); % Assuming theta has 7 angles

% Compute inverse kinematics for each point on the trajectory
for i = 1:length(t)
    % Construct transformation matrix for current trajectory point
    T = T_08;
    T(1,4) = px(i);
    T(2,4) = py(i);
    T(3,4) = pz(i);
    
    % Call inverse kinematics function
    theta = ARM_IK_CAL(T, phi);
    
    % Store the result
    theta_all(i, :) = theta;
end

% Plot the end-effector trajectory
figure;
plot3(px, py, pz, 'LineWidth', 1.5);
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('End-Effector Trajectory');
% legend('Trajectory');
axis equal;

% Plot joint angles
figure;
for j = 1:7
    subplot(7, 1, j);
    plot(t, theta_all(:, j));
    title(['Joint ' num2str(j) ' Angle']);
    xlabel('Time (s)');
    ylabel(['Theta ' num2str(j) ' (rad)']);
    grid on;
end

% Adjust layout
sgtitle('Joint Angles Over Time');

%%
% T_08 = [0, 0, 1, 9.000;
%         -1, 0, 0, -668.0000;
%          0, -1.0000, 0, 158.3000;
%          0, 0, 0, 1.0000];

T_08=[[ 0.72385667 -0.43704675 -0.5338742   0.60321589]
 [ 0.42081406 -0.33353447  0.84360553  0.15010233]
 [-0.5467605  -0.83531125 -0.05751573 -0.02606367]
 [ 0.          0.          0.          1.        ]];

