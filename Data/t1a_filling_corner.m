%Generate stuff
gt_resolution_x = 320.0;
gt_resolution_y = 320.0;
gt_fov_x = deg2rad(60.0);
gt_fov_y = deg2rad(60.0);

gt_distortion = [0.1 0 0 0 0];

focal_x = gt_resolution_x / tan(0.5 * gt_fov_x);
focal_y = gt_resolution_y / tan(0.5 * gt_fov_y);
projection = [
	focal_x,	0,			gt_resolution_x,	0;
	0,			focal_y,	gt_resolution_y,	0;
	0,			0,			1,				0
];

gt_camera_to_sled = [
	1,  0,  0	0
	0,  1,  0	0
	0,  0,  1	0
	0	0	0	1
];
gt_sled_to_target = [
	1,  0,  0	0
	0,  1,  0	0
	0,  0,  1	0
	0	0	0	1
];


target_size_vector = [3, 4];
target_scale_vector = [0.01, 0.01];

for s = 0.1 : 0.1 : 1.0
	
	target_motion_vector_xy = ((-0.1 : 0.05 : 0.1) * s);
	target_motion_vector_xy = target_motion_vector_xy + 0.1 - range(target_motion_vector_xy) / 2.0;
	target_motion_vector_z = 0.3 : 0.01 : 0.4;

	rng(12345);
	
	[tp_x, tp_y, tp_z] = meshgrid(...
	target_motion_vector_xy,...
	target_motion_vector_xy,...
	target_motion_vector_z);
	target_center_points = affinize_points(tp_x, tp_y, tp_z, gt_camera_to_sled);

	[sled_affine, target_affine, point_affine] = points_to_target(...
		target_center_points,...
		target_size_vector,...
		target_scale_vector,...
		gt_sled_to_target...
	);

	sled_points = [sled_affine(:, 1, 4), sled_affine(:, 2, 4), sled_affine(:, 3, 4)];
	target_points = [target_affine(:, 1, 4), target_affine(:, 2, 4)];
	point_points = [point_affine(:, 1, 4), point_affine(:, 2, 4), point_affine(:, 3, 4)];

	projected_points = nan(size(point_points, 1), 2);
	for i = 1 : size(point_points, 1)
		homogeneous_3 = [point_points(i, 1); point_points(i, 2); point_points(i, 3); 1];
		homogeneous_2 = projection * homogeneous_3;
	
		homogeneous_2(1) = homogeneous_2(1) / homogeneous_2(3);
		homogeneous_2(2) = homogeneous_2(2) / homogeneous_2(3);
		projected_points(i, 1) = homogeneous_2(1);
		projected_points(i, 2) = homogeneous_2(2);
	end
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
	
	%figure();
	%scatter(projected_points(:, 1), projected_points(:, 2));
	
	perturb = random('normal',0,0.5,size(projected_points));
	distorted_points = distorted_points + perturb;
	projected_points = projected_points + perturb;

	d_sweep = 1 : size(projected_points, 1);
	for d = 0 : 5 : 95
		d_select = mod(d_sweep, 100) > (d - 1);
		distorted_decimated = distorted_points(d_select, :);
		projected_decimated = projected_points(d_select, :);
		sled_decimated = sled_points(d_select, :);
		targ_decimated = target_points(d_select, :);
		
		data_block_d = [sled_decimated targ_decimated distorted_decimated];
		csvwrite(sprintf('filling_corner/distorted_%d_%d.csv', floor(s*10), d), data_block_d);
		data_block_u = [sled_decimated targ_decimated projected_decimated];
		csvwrite(sprintf('filling_corner/undistorted_%d_%d.csv', floor(s*10), d), data_block_u);
	end
end