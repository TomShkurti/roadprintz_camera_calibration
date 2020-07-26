%Generate projection stuff.
gt_proj = [
	1200	0		1152	0;
	0		1200	648		0;
	0		0		1		0
];
gt_dist = [
	1 1 40 0.1 0.1
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
for x = 0.0 : 0.1 : 0.5
	for y = 0.0 : 0.1 : 0.3
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
gt_TARGET_to_POINT_all = nan(25, 4, 4);
i = 1;
for u = 0 : 4
	for v = 0 : 4
		gt_TARGET_to_POINT_all(i, :, :) = [
			1 0 0 u * 0.03;
			0 1 0 v * 0.03;
			0 0 1 0.01;
			0 0 0 1
		];
		i = i + 1;
	end
end

%Project
projection_results = nan(size(gt_BASE_to_TIP_all, 1) * size(gt_TARGET_to_POINT_all, 1) / 2, 34);
i = 1;
j = 1;
figure();
hold on;
for tip = 1 : size(gt_BASE_to_TIP_all, 1)
	BASE_to_TIP = squeeze(gt_BASE_to_TIP_all(tip, :, :));
	for dot = 1 : size(gt_TARGET_to_POINT_all, 1)
		TARGET_to_POINT = squeeze(gt_TARGET_to_POINT_all(dot, :, :));
		
		%Position
		CAM_to_POINT = gt_CAM_to_BASE * BASE_to_TIP * gt_TIP_to_TARGET * TARGET_to_POINT;
		
		if(i == 1)
			TARGET_to_POINT
			TIP_to_POINT = gt_TIP_to_TARGET * TARGET_to_POINT
			BASE_to_POINT = BASE_to_TIP * gt_TIP_to_TARGET * TARGET_to_POINT
			CAM_to_POINT
		end
		
		scatter3(CAM_to_POINT(1, 4), CAM_to_POINT(2, 4), CAM_to_POINT(3, 4), [], 'rx');
		
		%Project
		projected_point = gt_proj * [CAM_to_POINT(1, 4); CAM_to_POINT(2, 4); CAM_to_POINT(3, 4); 1];
		u = projected_point(1) / projected_point(3);
		v = projected_point(2) / projected_point(3);
		
		%Distort
		[u_d, v_d] = distort(u, v, gt_dist, gt_proj);
		
		%Round
		u_d = u_d - mod(u_d, 1.0);
		v_d = v_d - mod(v_d, 1.0);
		
		%Record
		if(mod(i, 2) == 0)
			projection_results(j, :) = [
				squeeze(reshape(BASE_to_TIP, [], 1))' squeeze(reshape(TARGET_to_POINT, [], 1))' u_d v_d
			]';
			j = j + 1;
		end
		i = i + 1;
	end
end

quiver3(0, 0, 0, 0, 0, 1, 'b-');
quiver3(0, 0, 0, 1, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1, 0, 'g-');
axis equal;
hold off;

figure();
scatter(projection_results(:, 33), projection_results(:, 34));
axis equal;

csvwrite("rdata_1-1-40-01-01_r1d2.csv", projection_results);