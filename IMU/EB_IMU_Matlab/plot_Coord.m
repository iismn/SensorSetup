function plot_Coord(H,scale)
%% CE481 - HW#02 - Plot Coordination
%
% figure = plot_Coord(SE(3)Matrix, MetricScale)
%
% KAIST CEE.
% 20203454 / Lee Sang Min

%Figure = plot_Coord(~SE(3) Transformation Matrix)
U_x = [scale*1 0 0];
V_y = [0 scale*1 0];
W_z = [0 0 scale*1];
Origin = [0 0 0]';

R = H(1:3,1:3);
d = H(1:3,end);

Origin = Origin + d;
U_x = [R*U_x'];
V_y = [R*V_y'];
W_z= [R*W_z'];

quiver3(Origin(1), Origin(2), Origin(3), U_x(1), U_x(2), U_x(3), 'Color', 'red', 'LineWidth', 2); hold on;
quiver3(Origin(1), Origin(2), Origin(3), V_y(1), V_y(2), V_y(3), 'Color', 'green', 'LineWidth', 2);
quiver3(Origin(1), Origin(2), Origin(3), W_z(1), W_z(2), W_z(3), 'Color', 'blue', 'LineWidth', 2);
xlabel('x');
ylabel('y');
zlabel('z');
axis equal;
hold off;
end

