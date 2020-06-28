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

camera_premod = [
	0 0  1 0;
	0 -1 0 0;
	1 0  0 0;
	0 0 0 1
];

gt_CAM_to_FOREARM = [
	1 0 0 0;
	0 1 0 0;
	0 0 1 0;
	0 0 0 1
];
gt_FLANGE_to_HEAD = [
	1 0 0 0;
	0 1 0 0;
	0 0 1 0;
	0 0 0 1
];

FOREARM_to_BASE_all = csvread('arm_positions.csv');
FOREARM_to_BASE_all = FOREARM_to_BASE_all(:, 1 : end - 1);
FOREARM_to_BASE_all = reshape(FOREARM_to_BASE_all, [], 4, 4);
for i = 1 : size(FOREARM_to_BASE_all, 1)
	%They get read in transposed for some reason.
	FOREARM_to_BASE_all(i, :, :) = camera_premod * squeeze(FOREARM_to_BASE_all(i, :, :))';
end

BASE_to_FLANGE_all = nan(size(sprays_x, 1), 4, 4);
for i = 1 : size(sprays_x, 1)
	BASE_to_FLANGE_all(i, :, :) = [
		1  0  0 sprays_x(i);
		0 -1  0 sprays_y(i);
		0  0 -1 0.2
		0  0  0 1
	];
end

HEAD_to_POINT = [
	1 0 0 0;
	0 1 0 0;
	0 0 1 0.2;
	0 0 0 1
];

data_lines = nan(size(BASE_to_FLANGE_all, 1) * size(FOREARM_to_BASE_all, 1), 34);

btps = nan(size(BASE_to_FLANGE_all, 1), 3);
btfs = nan(size(data_lines, 1), 3);

i = 1;
for s = 1 : size(BASE_to_FLANGE_all, 1)
	BASE_to_FLANGE = squeeze(BASE_to_FLANGE_all(s, :, :));
	BASE_to_POINT = BASE_to_FLANGE * gt_FLANGE_to_HEAD * HEAD_to_POINT;
	
	btps(s, 1) = BASE_to_POINT(1, 4);
	btps(s, 2) = BASE_to_POINT(2, 4);
	btps(s, 3) = BASE_to_POINT(3, 4);
	
	for v = 1 : size(FOREARM_to_BASE_all, 1)
		FOREARM_to_BASE = squeeze(FOREARM_to_BASE_all(v, :, :));
		CAM_to_POINT = gt_CAM_to_FOREARM * FOREARM_to_BASE * BASE_to_POINT;
		
		point = [CAM_to_POINT(1, 4); CAM_to_POINT(2, 4); CAM_to_POINT(3, 4); 1];
		projected_point = gt_proj * point;
		
		data_lines(i, :) = [
			reshape(BASE_to_FLANGE, 1, []) reshape(FOREARM_to_BASE, 1, []) 0 0
		];
		data_lines(i, 33) = projected_point(1) / projected_point(3);
		data_lines(i, 34) = projected_point(2) / projected_point(3);
		%data_lines(i, 33) = point(1);
		%data_lines(i, 34) = point(2);
		
		btfs(i, 1) = FOREARM_to_BASE(1, 4);
		btfs(i, 2) = FOREARM_to_BASE(2, 4);
		btfs(i, 3) = FOREARM_to_BASE(3, 4);
		
		i = i + 1;
	end
end

squeeze(BASE_to_FLANGE_all(1, :, :))

scatter3(btps(:, 1), btps(:, 2), btps(:, 3));
hold on;
scatter3(btfs(:, 1), btfs(:, 2), btfs(:, 3));
hold off;

figure();
scatter(data_lines(:, 33), data_lines(:, 34));

csvwrite('synthetic_data.csv', data_lines);