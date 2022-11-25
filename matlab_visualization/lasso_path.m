clc
clear all
close all




% Create an matrix using the eigenvectors and eigenvalues

eigvec1 = [-1; -5];
eigvec2 = [eigvec1(2); -eigvec1(1)];
% eigvec2 = [eigvec1(1)+0.0001; eigvec1(2)];

eigval1 = 3;

eigval2 = 1;

A = eigvec1*eigvec1'*eigval1 + eigvec2*eigvec2'*eigval2;


% Plot the contour lines of the matrix

x1 = linspace(0,1500);
x2 = linspace(0,1500);

[X1,X2] = meshgrid(x1,x2);

Z = zeros(size(x1));

for i = 1:length(x1)
    for j = 1:length(x2)
        Z(i,j) = norm(A*[X1(i,j);X2(i,j)])^2;
    end
end

figure
contour(X1,X2,Z,100);






% Now we will plot the contour lines of the full problem

% Choose the center of the problem

x_sol = [1000;1000];

for i = 1:length(x1)
    for j = 1:length(x2)
        Z(i,j) = norm(A*[X1(i,j);X2(i,j)]-A*x_sol)^2;
    end
end

figure
contour(X1,X2,Z,100);




% Now define it as a function and test it out

loss = @(x) norm(A*[x(1);x(2)]-A*x_sol)^2;
loss_z = zeros(size(x1));

for i = 1:length(x1)
    for j = 1:length(x2)
        loss_z(i,j) = loss([X1(i,j);X2(i,j)]);
    end
end

figure
contour(X1,X2,loss_z,100);





% Now use some optimization technique to minimize the loss function
x0 = [0;0];
x_solution = fminsearch(loss,x0)


% Now we can use the loss function to go ahead and test out the lasso
% solution
lambda = 2.^[-15:30];
solution = zeros(length(lambda),2);

for i = 1:length(lambda)
    LASSO = @(x) norm(A*[x(1);x(2)]-A*x_sol)^2 + lambda(i)*norm(x,1);
    
    lambda(i)
    solution(i,:) = fminsearch(LASSO,x0)
    loss([solution(i,1);solution(i,2)])
    hold on
    plot(solution(i,1),solution(i,2),'X')
%     pause
    
    
end


hold on
plot(solution(:,1),solution(:,2),'-o')
title('L1 Normalization (LASSO)');

figure
plot(log(lambda),solution(:,1))
hold on
plot(log(lambda),solution(:,2))
title('L1 Normalization (LASSO)');
xlabel('log lambda')
ylabel('coefficients')









% Now we want to test out out version

figure
contour(X1,X2,loss_z,100);


% Now we can use the loss function to go ahead and test out the lasso
% solution
lambda = 2.^[-10:50];
solution = zeros(length(lambda),2);

p = 1;
q = 2;

for i = 1:length(lambda)
    %LASSO_PLUS = @(x) norm(A*[x(1);x(2)]-A*x_sol)^2 + lambda(i)*norm(x,1)/norm(x,2);
    
    LASSO_PLUS = @(x) norm(A*[x(1);x(2)]-A*x_sol)^2 + lambda(i)*sum(abs(x).^p)^(1/p)/sum(abs(x).^q)^(1/q)
    
    lambda(i);
    solution(i,:) = fminsearch(LASSO_PLUS,x0)
    loss([solution(i,1);solution(i,2)])
    hold on
    plot(solution(i,1),solution(i,2),'X')
%     pause
    
end


hold on
plot(solution(:,1),solution(:,2),'-o')
title('L1/L2 normalization (OSAMA&PAUL)');





figure
plot(log(lambda),solution(:,1))
hold on
plot(log(lambda),solution(:,2))
title('L1/L2 normalization (OSAMA&PAUL)');
xlabel('log lambda')
ylabel('coefficients')

