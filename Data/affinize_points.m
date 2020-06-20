function LIST_OUT = affinize_points(XS, YS, ZS, TRANSFORM)
	x = reshape(XS, [], 1);
	y = reshape(YS, [], 1);
	z = reshape(ZS, [], 1);

	LIST_OUT = nan(size(x, 1), 4, 4);
	for i = 1 : size(x, 1)
		LIST_OUT(i, :, :) = [
			1	0	0	x(i);
			0	1	0	y(i);
			0	0	1	z(i);
			0	0	0	1
		];
		LIST_OUT(i, :, :) = squeeze(LIST_OUT(i, :, :)) * TRANSFORM;
	end
end