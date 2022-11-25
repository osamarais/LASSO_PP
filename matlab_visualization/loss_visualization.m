clc
clear all
close all


N = 2;

p = 3;
q = 4;



l1_penalty = @(x) norm(x,1);

l1_l2_penalty = @(x) ((sum(abs(x).^p)/N)^(1/p))/((sum(abs(x).^q)/N)^(1/q));


% r = 0.1;
% other_penalty = @(x) sum(abs(x).^r)^(1/r);


x1 = linspace(-10,10);
x2 = linspace(-10,10);

[X1,X2] = meshgrid(x1,x2);


l1_z = zeros(size(x1));
l1_l2_z = zeros(size(x1));
% other_z = zeros(size(x1));


for i = 1:length(x1)
    for j = 1:length(x2)
        l1_z(i,j) = l1_penalty([X1(i,j);X2(i,j)]);
        l1_l2_z(i,j) = l1_l2_penalty([X1(i,j);X2(i,j)]);
%         other_z(i,j) = other_penalty([X1(i,j);X2(i,j)]);

    end
end

tiledlayout(2,2)
nexttile
contour(X1,X2,l1_z,10);
title('L1')

nexttile
contour(X1,X2,l1_l2_z,10);
title('L1/L2')

% figure
% contour(X1,X2,other_z,10);



nexttile
surf(X1,X2,l1_z);

nexttile
surf(X1,X2,l1_l2_z);

% figure
% surf(X1,X2,other_z);
