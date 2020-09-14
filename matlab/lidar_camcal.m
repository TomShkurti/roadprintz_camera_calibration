detection_points = csvread('detections.csv');
lidar_points_raw = csvread('pointcloud_points.csv');

lidar_points_table = nan(8, 4, 3);

for i = 1 : size(lidar_points_raw, 1)
	lidar_points_table(lidar_points_raw(i, 1) + 1, lidar_points_raw(i, 2) + 1, :) =...
		lidar_points_raw(i, 3 : 5);
end

positions = nan(size(detection_points, 1), 16);
points = ones(size(detection_points, 1), 4);
us = nan(size(detection_points, 1), 1);
vs = nan(size(detection_points, 1), 1);

for i = 1 : size(detection_points, 1)
	positions(i, :) = detection_points(i, 1 : 16);
	points(i, 1 : 3) = lidar_points_table(...
		detection_points(i, 17) + 1,...
		detection_points(i, 18) + 1, ...
		:...
	);
	us(i) = detection_points(i, 19);
	vs(i) = detection_points(i, 20);
end

figure('Name', 'Raw Lidar Detections');
scatter3(lidar_points_raw(:, 3), lidar_points_raw(:, 4), lidar_points_raw(:, 5));
axis equal;
hold on;
quiver3(0, 0, 0, 0, 0, 1.0, 'b-');
quiver3(0, 0, 0, 1.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');
hold off;

figure('Name', 'Processed Lidar Detections');
scatter3(points(:, 1), points(:, 2), points(:, 3));
axis equal;
hold on;
quiver3(0, 0, 0, 0, 0, 1.0, 'b-');
quiver3(0, 0, 0, 1.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');

vp_size = 0.2;
qvec_x = [vp_size; 0; 0; 0];
qvec_y = [0; vp_size; 0; 0];
qvec_z = [0; 0; vp_size; 0];

n_poses = 27;
for i = 1 : n_poses
	BASE_to_FOREARM = reshape(positions(i, :), 4, 4)';
	q_o_t = [BASE_to_FOREARM(1, 4), BASE_to_FOREARM(2, 4), BASE_to_FOREARM(3, 4)];
	q_x_t = BASE_to_FOREARM * qvec_x;
	q_y_t = BASE_to_FOREARM * qvec_y;
	q_z_t = BASE_to_FOREARM * qvec_z;
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_z_t(1), q_z_t(2), q_z_t(3), 'b-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_x_t(1), q_x_t(2), q_x_t(3), 'r-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_y_t(1), q_y_t(2), q_y_t(3), 'g-');
end


FOREARM_to_CAMERA = [
	0   1 0 1.062;
	-1  0 0 -0.105;
	0   0 1 -0.067;
	0   0 0 1
];

figure('Name', 'Adjusted Views');
scatter3(points(:, 1), points(:, 2), points(:, 3));
axis equal;
hold on;
quiver3(0, 0, 0, 0, 0, 1.0, 'b-');
quiver3(0, 0, 0, 1.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');

CAMERA_to_BASE_ALL = nan(size(positions, 1), 4, 4);

for i = 1 : n_poses
	BASE_to_FOREARM = reshape(positions(i, :), 4, 4)';
	BASE_to_CAMERA = BASE_to_FOREARM * FOREARM_to_CAMERA;
	CAMERA_to_BASE_ALL(i, :, :) = inv(BASE_to_CAMERA);
	q_o_t = [BASE_to_CAMERA(1, 4), BASE_to_CAMERA(2, 4), BASE_to_CAMERA(3, 4)];
	q_x_t = BASE_to_CAMERA * qvec_x;
	q_y_t = BASE_to_CAMERA * qvec_y;
	q_z_t = BASE_to_CAMERA * qvec_z;
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_z_t(1), q_z_t(2), q_z_t(3), 'b-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_x_t(1), q_x_t(2), q_x_t(3), 'r-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_y_t(1), q_y_t(2), q_y_t(3), 'g-');
end
hold off;

projection_matrix = [
	1637.367343	0			1313.144667	0
	0			1638.139550	774.029343	0
	0			0			1			0
];

figure('Name', 'Plotted Points');
hold on;
for i = 1 : n_poses
	projected_point = projection_matrix *...
		squeeze(CAMERA_to_BASE_ALL(i, :, :)) * points(i, :)';
	scatter(...
		projected_point(1) / projected_point(3),...
		projected_point(2) / projected_point(3),...
		'rs'...
	);
	scatter(us(i), vs(i), 'bx');
end