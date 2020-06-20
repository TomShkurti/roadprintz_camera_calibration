[sprays_x, sprays_y] = meshgrid(2.6 : 0.2 : 4.0, -1.0 : 0.2 : 1.0);
sprays_x = reshape(sprays_x, [], 1);
sprays_y = reshape(sprays_y, [], 1);
% scatter(sprays_x, sprays_y);
% figure();

gt_proj = [
	434.8289178042981	0					500.5	0;
	0					434.8289178042981	330.5	0;
	0					0					1		0
];
gt_camera = [
	-1 0 0 0;
	0 0 1 0;
	0 1 0 0;
	0 0 0 1
]';
gt_sprayhead = [
	1 0 0 0;
	0 1 0 0;
	0 0 1 0;
	0 0 0 1
];

take_poses = csvread('arm_positions.csv');
take_poses = take_poses(:, 1 : end - 1);
take_poses = reshape(take_poses, [], 4, 4);

camera_poses = nan(size(take_poses));
for i = 1 : size(camera_poses, 1)
	take_poses(i, :, :) = squeeze(take_poses(i, :, :))';
	camera_poses(i, :, :) = gt_camera * squeeze(take_poses(i, :, :));
end

scatter3(sprays_x, sprays_y, zeros(size(sprays_x)));
hold on;
scatter3(camera_poses(:, 1, 4), camera_poses(:, 2, 4), camera_poses(:, 3, 4));
hold off;
figure();

data_lines = nan(size(take_poses, 1) * size(sprays_x, 1), 34);
i = 1;
for t = 1 : size(take_poses, 1)
	take_pose = squeeze(take_poses(t, :, :));
	take_flat = reshape(take_pose, [], 1);
	camera_pose = squeeze(camera_poses(t, :, :));
	
	for p = 1 : size(sprays_x, 1)
		spray_matrix = [
			-1 0  0 sprays_x(p);
			0  1  0 sprays_y(p);
			0  0 -1 0.2;
			0  0  0 1
		];
		spraymat_flat = reshape(spray_matrix, [], 1);
		
		point = [sprays_x(p); sprays_y(p); 0; 1];
		
		spraypoint_transformed = camera_pose * point;
		
		spraypoint_projected = gt_proj * spraypoint_transformed;
		
		data_lines(i, :) = [
			take_flat' spraymat_flat' (spraypoint_projected(1) / spraypoint_projected(3)) (spraypoint_projected(2) / spraypoint_projected(3))
		];
		i = i + 1;
	end
end

scatter(data_lines(:, 33), data_lines(:, 34));

csvwrite('synthetic_data.csv', data_lines);