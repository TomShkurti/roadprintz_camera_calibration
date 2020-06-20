%big_block = csvread('columnarity_results/columnarity_undistorted.csv');
big_block = csvread('columnarity_results/columnarity_distorted.csv');

block_1 = big_block(big_block(:, 5) == 1, :);
block_2 = big_block(big_block(:, 5) == 2, :);
block_3 = big_block(big_block(:, 5) == 3, :);

findings_f_1 = block_1(:, 1);
findings_k_1 = block_1(:, 2);
findings_r_1 = block_1(:, 3);
n_1 = block_1(:, 4);
findings_f_2 = block_2(:, 1);
findings_k_2 = block_2(:, 2);
findings_r_2 = block_2(:, 3);
n_2 = block_2(:, 4);
findings_f_3 = block_3(:, 1);
findings_k_3 = block_3(:, 2);
findings_r_3 = block_3(:, 3);
n_3 = block_3(:, 4);

gt_fov_x = deg2rad(60.0);
focal_x = 320.0 / tan(0.5 * gt_fov_x);

findings_r_1 = findings_r_1 ./ n_1;
findings_f_1 = abs((findings_f_1 - focal_x) / focal_x);
findings_k_1 = abs((findings_k_1 - 0.1) / 0.1);
findings_r_2 = findings_r_2 ./ n_2;
findings_f_2 = abs((findings_f_2 - focal_x) / focal_x);
findings_k_2 = abs((findings_k_2 - 0.1) / 0.1);
findings_r_3 = findings_r_3 ./ n_3;
findings_f_3 = abs((findings_f_3 - focal_x) / focal_x);
findings_k_3 = abs((findings_k_3 - 0.1) / 0.1);

figure('Name', 'Distortion');
plot(n_1, abs(findings_k_1));
hold on;
plot(n_2, abs(findings_k_2));
plot(n_3, abs(findings_k_3));
xlabel 'n';
%ylabel 'k';
ylabel '\epsilon';

figure('Name', 'Focal');
plot(n_1, abs(findings_f_1));
hold on;
plot(n_2, abs(findings_f_2));
plot(n_3, abs(findings_f_3));
xlabel 'n';
ylabel '\epsilon';

figure('Name', 'Reprojection');
plot(n_1, abs(findings_r_1));
hold on;
plot(n_2, abs(findings_r_2));
plot(n_3, abs(findings_r_3));
xlabel 'n';
ylabel 'R';