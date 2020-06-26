[sprays_x, sprays_y] = meshgrid(2.6 : 0.2 : 4.0, -1.0 : 0.2 : 1.0);
sprays_x = reshape(sprays_x, [], 1);
sprays_y = reshape(sprays_y, [], 1);

gt_FLANGE_to_HEAD = [
	1 0 0 0;
	0 1 0 0;
	0 0 1 0;
	0 0 0 1
];

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

data_lines = nan(size(BASE_to_FLANGE_all, 1), 34);

i = 1;
for s = 1 : size(BASE_to_FLANGE_all, 1)
	BASE_to_FLANGE = squeeze(BASE_to_FLANGE_all(s, :, :));
	BASE_to_POINT = BASE_to_FLANGE * gt_FLANGE_to_HEAD * HEAD_to_POINT;
	
	data_lines(i, :) = [
		reshape(BASE_to_FLANGE, 1, []) reshape(BASE_to_POINT, 1, []) 0 0
	];

	i = i + 1;
end

csvwrite('synthetic_data_head.csv', data_lines);