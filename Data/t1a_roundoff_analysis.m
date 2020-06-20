big_block = csvread('roundoff_results/roundoff_undistorted.csv');
%big_block = csvread('roundoff_results/roundoff_distorted.csv');

findings_f = big_block(:, 1);
findings_k = big_block(:, 2);
findings_r = big_block(:, 3);
n = big_block(:, 4);
s = big_block(:, 5);

gt_fov_x = deg2rad(60.0);
focal_x = 320.0 / tan(0.5 * gt_fov_x);
findings_r = findings_r ./ n;
findings_f = abs((findings_f - focal_x) / focal_x);
findings_k = abs((findings_k - 0.1) / 0.1);
s = s / 10.0;

n = reshape(n, 20, 20);
s = reshape(s, 20, 20);
findings_k = reshape(findings_k, 20, 20);
findings_r = reshape(findings_r, 20, 20);
findings_f = reshape(findings_f, 20, 20);

figure('Name', 'Distortion');
surf(n, s, abs(findings_k));
xlabel 'n';
ylabel 'r';
zlabel 'k';
%zlabel '\epsilon';

figure('Name', 'Focal');
surf(n, s, abs(findings_f));
xlabel 'n';
ylabel 'r';
zlabel '\epsilon';

figure('Name', 'Reprojection');
surf(n, s, abs(findings_r));
xlabel 'n';
ylabel 'r';
zlabel '\epsilon';