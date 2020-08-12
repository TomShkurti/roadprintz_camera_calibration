%Plotting system to roughly calculate camera extrinsic positions.

%Deserialize the data we are given
data = csvread('detections.csv');
lefts = data(:, 1);
tf = data(:, 2 : 17);
u = data(:, 18);
v = data(:, 19);

tf = reshape(tf, [], 4, 4);
BASE_to_FOREARM_vec = nan(size(tf, 1), 4, 4);
FOREARM_to_BASE_vec = nan(size(tf, 1), 4, 4);
for i = 1 : size(tf, 1)
	BASE_to_FOREARM_vec(i, :, :) = inv(squeeze(tf(i, :, :)))';
	FOREARM_to_BASE_vec(i, :, :) = squeeze(tf(i, :, :))';
end

%Set estimates and ground truths.
BASE_to_TARGET_L = [-0.3;  0.4; 0.];
BASE_to_TARGET_R = [-0.3; -0.4; 0.];

CAM_to_FOREARM = inv([%Forearm to cam is easier to visualize
	 0  1  0  1.0;
	 0  0 -1  0;
	-1  0  0  0;
	 0  0  0  1
]);
% CAM_to_FOREARM = [
% 	1 0 0 0;
% 	0 1 0 0;
% 	0 0 1 0;
% 	0 0 0 1
% ];

PM = [
	400.0	0			500		0;
	0		400.0		330		0;
	0		0			1			0
];

%Generally useful visualization helpers
vp_size = 0.2;
qvec_x = [vp_size; 0; 0; 0];
qvec_y = [0; vp_size; 0; 0];
qvec_z = [0; 0; vp_size; 0];

figure('Name', 'Observation Points');
hold on;
axis equal;
quiver3(0, 0, 0, 0, 0, 1.0, 'b-');
quiver3(0, 0, 0, 1.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');
scatter3(BASE_to_TARGET_L(1), BASE_to_TARGET_L(2), BASE_to_TARGET_L(3), 'bx');
scatter3(BASE_to_TARGET_R(1), BASE_to_TARGET_R(2), BASE_to_TARGET_R(3), 'rx');
for i = 1 : size(lefts)
	if(lefts(i))
		code = 'bo';
	else
		code = 'ro';
	end
	
	BASE_to_FOREARM = squeeze(BASE_to_FOREARM_vec(i, :, :));
	
	BASE_to_CAM = BASE_to_FOREARM * [0; 0; 0; 1];
	scatter3(BASE_to_CAM(1), BASE_to_CAM(2), BASE_to_CAM(3), code, 'filled');

	q_o_t = BASE_to_CAM;
	q_x_t = BASE_to_FOREARM * qvec_x;
	q_y_t = BASE_to_FOREARM * qvec_y;
	q_z_t = BASE_to_FOREARM * qvec_z;
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_z_t(1), q_z_t(2), q_z_t(3), 'b-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_x_t(1), q_x_t(2), q_x_t(3), 'r-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_y_t(1), q_y_t(2), q_y_t(3), 'g-');
end
hold off;

figure('Name', 'Corrected Observation Points');
hold on;
axis equal;
quiver3(0, 0, 0, 0, 0, 1.0, 'b-');
quiver3(0, 0, 0, 1.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');
scatter3(BASE_to_TARGET_L(1), BASE_to_TARGET_L(2), BASE_to_TARGET_L(3), 'bx');
scatter3(BASE_to_TARGET_R(1), BASE_to_TARGET_R(2), BASE_to_TARGET_R(3), 'rx');
for i = 1 : size(lefts)
	if(lefts(i))
		code = 'bo';
	else
		code = 'ro';
	end
	FOREARM_to_BASE = squeeze(FOREARM_to_BASE_vec(i, :, :));
	
	CAM_to_BASE = CAM_to_FOREARM * FOREARM_to_BASE;
	cam_pnt = CAM_to_BASE \ [0; 0; 0; 1];
	scatter3(cam_pnt(1), cam_pnt(2), cam_pnt(3), code, 'filled');

	q_o_t = cam_pnt;
	q_x_t = CAM_to_BASE \ qvec_x;
	q_y_t = CAM_to_BASE \ qvec_y;
	q_z_t = CAM_to_BASE \ qvec_z;
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_z_t(1), q_z_t(2), q_z_t(3), 'b-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_x_t(1), q_x_t(2), q_x_t(3), 'r-');
	quiver3(q_o_t(1), q_o_t(2), q_o_t(3), q_y_t(1), q_y_t(2), q_y_t(3), 'g-');
end
hold off;

figure('Name', 'Target Points w.r.t. Camera');
hold on;
axis equal;
quiver3(0, 0, 0, 0, 0, 1.0, 'b-');
quiver3(0, 0, 0, 1.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 1.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');
sum_l = [0; 0; 0; 0];
sum_r = [0; 0; 0; 0];
for i = 1 : size(lefts)
	if(lefts(i))
		tvec = [BASE_to_TARGET_L; 1];
		code = 'bo';
	else
		tvec = [BASE_to_TARGET_R; 1];
		code = 'ro';
	end
	FOREARM_to_BASE = squeeze(FOREARM_to_BASE_vec(i, :, :));
	
	CAM_to_TARGET = CAM_to_FOREARM * FOREARM_to_BASE * tvec;
	scatter3(CAM_to_TARGET(1), CAM_to_TARGET(2), CAM_to_TARGET(3), code, 'filled');
	if(lefts(i))
		sum_l = sum_l + CAM_to_TARGET;
	else
		sum_r = sum_r + CAM_to_TARGET;
	end
end
scatter3(sum_l(1) / sum_l(4), sum_l(2) / sum_l(4), sum_l(3) / sum_l(4), 'bx');
scatter3(sum_r(1) / sum_r(4), sum_r(2) / sum_r(4), sum_r(3) / sum_r(4), 'rx');
hold off;

figure('Name', 'Projected Points');
hold on;
axis equal;
quiver3(0, 0, 0, 100.0, 0, 0, 'r-');
quiver3(0, 0, 0, 0, 100.0, 0, 'g-');
scatter3(0, 0, 0, 50.0, 'kO');
sum_l_real = [0; 0; 0];
sum_r_real = [0; 0; 0];
sum_l_calc = [0; 0; 0];
sum_r_calc = [0; 0; 0];
for i = 1 : size(lefts)
	if(lefts(i))
		tvec = [BASE_to_TARGET_L; 1];
		calc_code = 'bo';
		real_code = 'bx';
	else
		tvec = [BASE_to_TARGET_R; 1];
		calc_code = 'ro';
		real_code = 'rx';
	end
	FOREARM_to_BASE = squeeze(FOREARM_to_BASE_vec(i, :, :));
	
	CAM_to_TARGET = CAM_to_FOREARM * FOREARM_to_BASE * tvec;
	if(i == 1)
		tvec
		FOREARM_to_BASE * tvec
		CAM_to_FOREARM
		CAM_to_TARGET
	end
	p = PM * CAM_to_TARGET;
	
	if(lefts(i))
		sum_l_real = sum_l_real + [u(i); v(i); 1];
		sum_l_calc = sum_l_calc + [p(1)/p(3); p(2)/p(3); 1];
	else
		sum_r_real = sum_r_real + [u(i); v(i); 1];
		sum_r_calc = sum_r_calc + [p(1)/p(3); p(2)/p(3); 1];
	end
	
	scatter(p(1)/p(3), p(2)/p(3), calc_code);
	
	scatter(u(i), v(i), real_code);
end
scatter(sum_l_real(1) / sum_l_real(3), sum_l_real(2) / sum_l_real(3), 'bo', 'filled');
scatter(sum_l_calc(1) / sum_l_calc(3), sum_l_calc(2) / sum_l_calc(3), 'bs');
scatter(sum_r_real(1) / sum_r_real(3), sum_r_real(2) / sum_r_real(3), 'ro', 'filled');
scatter(sum_r_calc(1) / sum_r_calc(3), sum_r_calc(2) / sum_r_calc(3), 'rs');
hold off;