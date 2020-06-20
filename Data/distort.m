function DISTORTED_POINTS = distort(POINTS, D_PARAMS)
	r_squared = POINTS(:, 1) .^ 2 + POINTS(:, 2) .^ 2;
	r_squared = [r_squared r_squared];
	
	dist_1 = POINTS .* (...
		1 + D_PARAMS(1) .* r_squared +...
		D_PARAMS(2) .* (r_squared .^2) +...
		D_PARAMS(3) .* (r_squared .^3)...
	);

	x_tangential = dist_1(:, 1) +...
		D_PARAMS(5) .* (r_squared(:, 1) + 2 * POINTS(:, 1) .^ 2) +...
		D_PARAMS(4) .*  POINTS(:, 1) .*  POINTS(:, 2) * 2 ...
	;
	y_tangential = dist_1(:, 2) +...
		D_PARAMS(4) .* (r_squared(:, 2) + 2 * POINTS(:, 1) .^ 2) +...
		D_PARAMS(5) .* POINTS(:, 1) .*  POINTS(:, 2) * 2 ...
	;
	
	DISTORTED_POINTS = [x_tangential y_tangential];
end