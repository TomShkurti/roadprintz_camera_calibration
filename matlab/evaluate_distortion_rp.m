data_radial = csvread("../intermediate/radial_condense.csv");
data_random = csvread("../intermediate/random_knockout.csv");

n = data_radial(:, 11);

figure('Name', 'Reprojection Errors');
plot(n, data_radial(:, 1), 'b-o');
hold on;
plot(n, data_random(:, 1), 'r-s');
hold off;

figure('Name', 'Focal Errors');
plot(n, abs(data_radial(:, 2) - data_radial(1, 2)), 'b-o');
hold on;
plot(n, abs(data_radial(:, 3) - data_radial(1, 3)), 'b-o');

plot(n, abs(data_random(:, 2) - data_random(1, 2)), 'r-s');
plot(n, abs(data_random(:, 3) - data_random(1, 3)), 'r-s');
hold off;

figure('Name', 'Center Errors');
plot(n, abs(data_radial(:, 4) - data_radial(1, 4)), 'b-o');
hold on;
plot(n, abs(data_radial(:, 5) - data_radial(1, 5)), 'b-o');

plot(n, abs(data_random(:, 4) - data_random(1, 4)), 'r-s');
plot(n, abs(data_random(:, 5) - data_random(1, 5)), 'r-s');
hold off;

figure('Name', 'Distortion Errors');
plot(n, abs(data_radial(:, 6) - data_radial(1, 6)), 'b-o');
hold on;
plot(n, abs(data_radial(:, 7) - data_radial(1, 7)), 'b-o');
plot(n, abs(data_radial(:, 8) - data_radial(1, 8)), 'b-o');
plot(n, abs(data_radial(:, 9) - data_radial(1, 9)), 'b-o');
plot(n, abs(data_radial(:,10) - data_radial(1,10)), 'b-o');

plot(n, abs(data_random(:, 6) - data_random(1, 6)), 'r-s');
plot(n, abs(data_random(:, 7) - data_random(1, 7)), 'r-s');
plot(n, abs(data_random(:, 8) - data_random(1, 8)), 'r-s');
plot(n, abs(data_random(:, 9) - data_random(1, 9)), 'r-s');
plot(n, abs(data_random(:,10) - data_random(1,10)), 'r-s');
hold off;