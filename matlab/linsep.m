lefts = csvread("lefts.csv");
rights = csvread("rights.csv");
garbage = csvread("garbage.csv");

figure('Name', '0 vs 1');
scatter(lefts(:, 1), lefts(:, 2), 'bx');
hold on;
scatter(rights(:, 1), rights(:, 2), 'rx');
scatter(garbage(:, 1), garbage(:, 2), 'k.');
hold off;

figure('Name', '2 vs 3');
scatter(lefts(:, 3), lefts(:, 4), 'bx');
hold on;
scatter(rights(:, 3), rights(:, 4), 'rx');
scatter(garbage(:, 3), garbage(:, 4), 'k.');
hold off;

figure('Name', '4 vs 5');
scatter(lefts(:, 5), lefts(:, 6), 'bx');
hold on;
scatter(rights(:, 5), rights(:, 6), 'rx');
scatter(garbage(:, 5), garbage(:, 6), 'k.');
hold off;

figure('Name', '6 vs 7');
scatter(lefts(:, 7), lefts(:, 8), 'bx');
hold on;
scatter(rights(:, 7), rights(:, 8), 'rx');
scatter(garbage(:, 1), garbage(:, 8), 'k.');
hold off;