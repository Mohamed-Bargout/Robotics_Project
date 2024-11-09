function q = Inverse_kinematics_func(q0,X_desired)
%q0: Initial quess as a column vector
%X_desired : desired postions as column vector
syms q1 q2 q3 q4;
q = [q1;q2;q3;q4];

%Define the Tolerance 
x_error = 0.01;
y_error = 0.01;
z_error = 0.01;
error_tol = [x_error;y_error;z_error];


%Define Desired Location of the end effector
X = Forward_kinematics_func();
F = X - X_desired;

%Define the Jacobian Matrix
J = Jacobian_matrix(q);

%Define the Maximum number of iterations
max_iter = 100;
q_curr = q0;
for i = 1:max_iter
    J_evaluated = eval(subs(J, {q1, q2, q3, q4}, {q_curr(1), q_curr(2), q_curr(3), q_curr(4)}));
    F_evaluated = eval(subs(F, {q1, q2, q3, q4}, {q_curr(1), q_curr(2), q_curr(3), q_curr(4)}));
 
    q_increment = pinv(J_evaluated) * F_evaluated;
    
    % Update q_curr
    q_curr = q_curr - q_increment; 

    % Check if the error is below the tolerances
        if all(abs(F_evaluated) < error_tol)
            disp('Converged to desired accuracy.');
            disp(i)
            break;  % Exit the loop if converged
        end
end

q = wrapTo2Pi(q_curr);
end



