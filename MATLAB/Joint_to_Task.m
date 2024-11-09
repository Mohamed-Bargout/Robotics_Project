clear
clc
close all

syms q1 q2 q3 q4;

q0 = [30,0,0,60];
qf = [90,20,90,0];
qdot0 = [0,0,0,0];
qdotf = [0,0,0,0];
Tf = 7;
Ts = 0.1;
t = 0:Ts:Tf;

Joint_space = Joint_traj(q0,qf,qdot0,qdotf,Tf,Ts);

qq1 = Joint_space(1,:);
qq2 = Joint_space(2,:);
qq3 = Joint_space(3,:);
qq4 = Joint_space(4,:);

Task = zeros(3,71);

X = Forward_kinematics_func();

for i = 1:71
    
    Task(:,i) = eval(subs(X,{q1,q2,q3,q4},{-qq1(i),-qq2(i),qq3(i),qq4(i)}));

end

%%
%Visualization

% Assuming Task is a 3x71 matrix
% Task(1, :) contains X values
% Task(2, :) contains Y values
% Task(3, :) contains Z values

% Create a 3D plot
figure;
plot3(Task(1, 1:10), Task(2, 1:10), Task(3, 1:10), 'LineWidth', 2);

% Add labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Plot of Task');

% Adjust the view if needed
view(3);

% Optionally, add grid or other settings
grid on;