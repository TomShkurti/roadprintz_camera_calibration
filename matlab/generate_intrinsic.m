%Generate projection stuff.
gt_proj = [
	434.8289178042981	0					500.5	0;
	0					434.8289178042981	330.5	0;
	0					0					1		0
];
gt_dist = [
	1 1 50 0.1 0.1
];

%Generate transforms.
gt_CAM_to_BASE = [
	1 0 0 -0.3;
	0 1 0 -0.2;
	0 0 1 0.5;
	0 0 0 1
];
gt_BASE_to_TIP_all = nan(96, 4, 4);
i = 1;
for x = 0 : 0.1 : 0.5
	for y = 0 : 0.1 : 0.3
		for z = 0 : 0.1 : 0.3
			gt_BASE_to_TIP_all(i, :, :) = [
				1 0 0 x;
				0 1 0 y;
				0 0 1 z;
				0 0 0 1
			];
			i = i + 1;
		end
	end
end
gt_TIP_to_TARGET = [
	1 0 0 0;
	0 1 0 0;
	0 0 1 0;
	0 0 0 1
];
gt_TARGET_to_POINT_all = nan(30, 4, 4);
i = 1;
for u = 0 : 4
	for v = 0 : 5
		gt_TARGET_to_POINT_all(i, :, :) = [
			1 0 0 u * 0.01;
			0 1 0 v * 0.01;
			0 0 1 0.01;
			0 0 0 1
		];
		i = i + 1;
	end
end

%Project
projection_results = nan(size(gt_BASE_to_TIP_all, 1) * size(gt_TARGET_to_POINT_all, 1), 34);
i = 1;
figure();
hold on;
for tip = 1 : size(gt_BASE_to_TIP_all, 1)
	BASE_to_TIP = squeeze(gt_BASE_to_TIP_all(tip, :, :));
	for dot = 1 : size(gt_TARGET_to_POINT_all, 1)
		TARGET_to_POINT = squeeze(gt_TARGET_to_POINT_all(dot, :, :));
		
		%Position
		CAM_to_POINT = gt_CAM_to_BASE * BASE_to_TIP * gt_TIP_to_TARGET * TARGET_to_POINT;
		%scatter3(CAM_to_POINT(1, 4), CAM_to_POINT(2, 4), CAM_to_POINT(3, 4), [], 'rx');
		
		%Project
		projected_point = gt_proj * [CAM_to_POINT(1, 4); CAM_to_POINT(2, 4); CAM_to_POINT(3, 4); 1];
		u = projected_point(1) / projected_point(3);
		v = projected_point(2) / projected_point(3);
		
		%Distort
		[u_d, v_d] = distort(u, v, gt_dist, gt_proj);
		
		%Round
		%TODO
		
		%Record
		projection_results(i, :) = [
			squeeze(reshape(BASE_to_TIP, [], 1))' squeeze(reshape(TARGET_to_POINT, [], 1))' u_d v_d
		]';
		i = i + 1;
	end
end

quiver3(0, 0, 0, 0, 0, 1, '-');
hold off;

figure();
scatter(projection_results(:, 33), projection_results(:, 34));

csvwrite("idata_1-1-50-01-01.csv", projection_results);