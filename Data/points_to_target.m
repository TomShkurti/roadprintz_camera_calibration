function [SLED_POINTS, TARGET_POINTS, COMBINED_POINTS] =...
points_to_target(TARGET_POSITIONS, TARGET_SIZE, TARGET_SCALE, TARGET_TRANSFORM)

	target_vec_x = 0 : TARGET_SCALE(1) : TARGET_SIZE(1) * TARGET_SCALE(1);
	target_vec_y = 0 : TARGET_SCALE(2) : TARGET_SIZE(2) * TARGET_SCALE(2);
	
	[tp_x, tp_y] = meshgrid(target_vec_x, target_vec_y);
	tp_x = reshape(tp_x, [], 1);
	tp_y = reshape(tp_y, [], 1);
	
	SLED_POINTS = nan(size(tp_x, 1) * size(TARGET_POSITIONS, 1), 4, 4);
	TARGET_POINTS = nan(size(tp_x, 1) * size(TARGET_POSITIONS, 1), 4, 4);
	COMBINED_POINTS = nan(size(tp_x, 1) * size(TARGET_POSITIONS, 1), 4, 4);
	for s = 1 : size(TARGET_POSITIONS, 1)
		for t = 1 : size(tp_x, 1)
			SLED_POINTS(t + (s-1) * size(tp_x, 1), :, :)=...
				squeeze(TARGET_POSITIONS(s, :, :));
			
			TARGET_POINTS(t + (s-1) * size(tp_x, 1), :, :)= [
				1	0	0	tp_x(t);
				0	1	0	tp_y(t);
				0	0	1	0;
				0	0	0	1
			];
			COMBINED_POINTS(t + (s-1) * size(tp_x, 1), :, :)=...
				squeeze(TARGET_POINTS(t + (s-1) * size(tp_x, 1), :, :)) * ...
				TARGET_TRANSFORM * ...
				squeeze(SLED_POINTS(t + (s-1) * size(tp_x, 1), :, :));
		end
	end
end