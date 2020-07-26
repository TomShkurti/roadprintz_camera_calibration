% Matlab setup for visualizing and coarsely calibrating the camera or
% establishing good seed values to begin calibration with. In the future,
% most of this will need to be automated as currently all there is to it is
% a bunch of values you can tweak to change the graphs when it is rerun.

%%
%%Record and preprocess.
big_data = csvread('intrinsic_calibration_points.csv');
%TODO: Make this automatically read from where the file is dropped.

u = big_data(:, 33);
v = big_data(:, 34);

h_x = big_data(:, 13);
h_y = big_data(:, 14);
h_z = big_data(:, 15);

t_x = big_data(:, 29);
t_y = big_data(:, 30);
t_z = big_data(:, 31);

p_x = h_x + t_x;
p_y = h_y + t_y;
p_z = h_z + t_z;

%%
%%Dimensions check. Align the two plots so that their axes point the same
%%direction, and fiddle with the "AXIS ASSIGNMENTS" in
%%intrinsic_detect.cpp, negating them and reassigning them until the shapes
%%in the two plots look more or less identical (not rotated or flipped).
n_targets = 100;

figure('Name', 'Target Scatter - Unadjusted');
scatter3(p_x(1 : n_targets, :), p_y(1 : n_targets, :), p_z(1 : n_targets, :));

hold on;
quiver3(0, 0, 0, 0, 0, 0.1, 'b-');
quiver3(0, 0, 0, 0.1, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 0.1, 0, 'g-');
hold off;

figure('Name', 'Image Scatter - Unadjusted');
scatter(u(1 : n_targets, :), v(1 : n_targets, :));
axis equal;

hold on;
quiver3(0, 0, 0, 100, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 100, 0, 'g-');
hold off;

%%
%%Offset check: set adjust_offset_x, adjust_offset_y, and adjust_offset_z
%%to 0, then look at the distance from the axes in the center to the
%%centroid of the point cloud and set those three values to put the
%%centroid over the axes. These values are then CAM_to_BASE_t in
%%INTRINSIC_CAL.cpp
%TODO: Should this just be automated? It's not like centroids are hard to
%caluclate or anything.
adjust_offset_x = 0.125125;
adjust_offset_y = -0.06;
adjust_offset_z = 0.4;

p_x_adjusted = p_x + adjust_offset_x;
p_y_adjusted = p_y + adjust_offset_y;
p_z_adjusted = p_z + adjust_offset_z;

figure('Name', 'Target Scatter - Adjusted');
scatter3(p_x_adjusted, p_y_adjusted, p_z_adjusted);

hold on;
quiver3(0, 0, 0, 0, 0, 0.1, 'b-');
quiver3(0, 0, 0, 0.1, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 0.1, 0, 'g-');
hold off;

%%
%%Z check: Make sure the squares closest to the camera are larger (points
%%spaced farther apart) than the ones father away, so that you know the Z
%%value in "AXIS ASSIGNMENTS" is not negated. The Z-planes are color-coded
%%for visibility in the flat image.
n_targets = 1500;
figure('Name', '3D Z Check');
scatter3(p_x(1 : n_targets, :), p_y(1 : n_targets, :), p_z(1 : n_targets, :), [], p_z(1 : n_targets));

hold on;
quiver3(0, 0, 0, 0, 0, 0.1, 'b-');
quiver3(0, 0, 0, 0.1, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 0.1, 0, 'g-');
hold off;

figure('Name', '2D Z check');
scatter(u(1 : n_targets, :), v(1 : n_targets, :), [], p_z(1 : n_targets));
axis equal;

hold on;
quiver3(0, 0, 0, 100, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 100, 0, 'g-');
hold off;

%%
%%Target alignment: Make sure that the direction of the color gradient on
%%the 2D and 3D targets matches (when the axes in both plots are aligned).
%%If they do not, transpose the i_circle and j_circle values indices in
%%intrinsic_detect.cpp's "WRITE TO FILE" section.
figure('Name', 'Target Scatter - Color-coded');
scatter3(p_x(1 : 25, :), p_y(1 : 25, :), p_z(1 : 25, :), [], 1 : 25);
hold on;
quiver3(0, 0, 0, 0, 0, 0.1, 'b-');
quiver3(0, 0, 0, 0.1, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 0.1, 0, 'g-');
hold off;

figure('Name', 'Image Scatter - Color-Coded');
scatter(u(1 : 25, :), v(1 : 25, :), [], 1 : 25);
axis equal;

hold on;
quiver3(0, 0, 0, 100, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 100, 0, 'g-');
hold off;

%%
%%Simple intrinsic calculation: Tweak the values in the projection matrix
%%below until the real image (red) and projected one (blue) more or less
%%line up. These values are then fx, fy, cx, and cy in INTRINSIC_CAL.cpp
projection_matrix = [
	1633	0		1222	0;
	0		1635	953		0;
	0		0		1		0
];

u_p = nan(size(p_x));
v_p = nan(size(p_x));
for i = 1 : size(p_x, 1)
	%Project
	projected_point = projection_matrix * [p_x_adjusted(i); p_y_adjusted(i); p_z_adjusted(i); 1];
	u_p(i) = projected_point(1) / projected_point(3);
	v_p(i) = projected_point(2) / projected_point(3);
end

figure('Name', 'Image Match');
scatter(u, v, 'rx');
hold on;
scatter(u_p, v_p, 'bo');
hold on;
quiver3(0, 0, 0, 100, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 100, 0, 'g-');
hold off;