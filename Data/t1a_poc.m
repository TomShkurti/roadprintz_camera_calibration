gt_resolution_x = 320.0;
gt_resolution_y = 320.0;
gt_fov_x = deg2rad(60.0);
gt_fov_y = deg2rad(60.0);

%gt_distortion = [0 0 0 0 0];
gt_distortion = [0.1 0 0 0 0];
%gt_distortion = [0.4 0.4 0 0 0];
%gt_distortion = [0.1 0.1 0.1 -0.1 -0.1];
% gt_camera_to_sled = [
% 	1,  0,  0	0
% 	0,  1,  0	0
% 	0,  0,  1	0
% 	0	0	0	1
% ];
gt_camera_to_sled = [
	1,  0,  0	0.3
	0,  1,  0	0.2
	0,  0,  1	0.1
	0	0	0	1
];
% gt_camera_to_sled = [
% 	0.4119822,  0.0587266,  0.9092974	0.3
% 	-0.6812427, -0.6428728,  0.3501755	0.2
% 	0.6051273, -0.7637183, -0.2248451	0.1
% 	0	0	0	1
% ];

% gt_sled_to_target = [
% 	1,  0,  0	0
% 	0,  1,  0	0
% 	0,  0,  1	0
% 	0	0	0	1
% ];
gt_sled_to_target = [
	 0.9362934, -0.2896295,  0.1986693	0
	0.3129918,  0.9447025, -0.0978434	0
	-0.1593451,  0.1537920,  0.9751703	0
	0	0	0	1
];

focal_x = gt_resolution_x / tan(0.5 * gt_fov_x);
focal_y = gt_resolution_y / tan(0.5 * gt_fov_y);
projection = [
	focal_x,	0,			gt_resolution_x,	0;
	0,			focal_y,	gt_resolution_y,	0;
	0,			0,			1,				0
];


target_motion_vector_xy = -0.1 : 0.05 : 0.1;
target_motion_vector_z = 0.3 : 0.01 : 0.4;
target_size_vector = [3, 4];
target_scale_vector = [0.01, 0.01];


[tp_x, tp_y, tp_z] = meshgrid(...
	target_motion_vector_xy,...
	target_motion_vector_xy,...
	target_motion_vector_z);
target_center_points = affinize_points(tp_x, tp_y, tp_z, gt_camera_to_sled);

scatter3(target_center_points(:, 1, 4), target_center_points(:, 2, 4), target_center_points(:, 3, 4));
figure();

[sled_affine, target_affine, point_affine] = points_to_target(...
	target_center_points,...
	target_size_vector,...
	target_scale_vector,...
	gt_sled_to_target...
);

sled_points = [sled_affine(:, 1, 4), sled_affine(:, 2, 4), sled_affine(:, 3, 4)];
target_points = [target_affine(:, 1, 4), target_affine(:, 2, 4)];
point_points = [point_affine(:, 1, 4), point_affine(:, 2, 4), point_affine(:, 3, 4)];

scatter3(point_points(:, 1), point_points(:, 2), point_points(:, 3));
figure();

projected_points = nan(size(point_points, 1), 2);
for i = 1 : size(point_points, 1)
	homogeneous_3 = [point_points(i, 1); point_points(i, 2); point_points(i, 3); 1];
	homogeneous_2 = projection * homogeneous_3;
	
	homogeneous_2(1) = homogeneous_2(1) / homogeneous_2(3);
	homogeneous_2(2) = homogeneous_2(2) / homogeneous_2(3);
	projected_points(i, 1) = homogeneous_2(1);
	projected_points(i, 2) = homogeneous_2(2);
end
scatter(projected_points(:, 1), projected_points(:, 2));
hold on;
projected_points(:, 1) = projected_points(:, 1) - gt_resolution_x;
projected_points(:, 2) = projected_points(:, 2) - gt_resolution_y;

pp_shrunk = [projected_points(:, 1) / gt_resolution_x...
	projected_points(:, 2) / gt_resolution_y];
distorted_points = distort(pp_shrunk, gt_distortion);
distorted_points(:, 1) = distorted_points(:, 1) * gt_resolution_x;
distorted_points(:, 2) = distorted_points(:, 2) * gt_resolution_y;
distorted_points(:, 1) = distorted_points(:, 1) + gt_resolution_x;
distorted_points(:, 2) = distorted_points(:, 2) + gt_resolution_y;
projected_points(:, 1) = projected_points(:, 1) + gt_resolution_x;
projected_points(:, 2) = projected_points(:, 2) + gt_resolution_y;

scatter(distorted_points(:, 1), distorted_points(:, 2));

data_block = [sled_points target_points distorted_points];
csvwrite('trans_rot_1.csv', data_block);