% Process an image of a Blue Morpho butterfly for mesh generation

clear all, close all, clc

% Load image, threshold to binary image, fill holes, and get edge of image
disp('Processing image')
image_rgb = imread('Butterfly.jpg');
image_bw = ~im2bw(image_rgb);
image_bw_fill = ~imfill(image_bw, 'holes');
image_bw_edge = ~edge(image_bw_fill);

% Store outline as two-column matrix
[i,j] = find(image_bw_edge~=1);
pixel_positions = [i j];

% 'Decimate' data
decimated_pixel_positions = [];
for k=1:20:length(pixel_positions)
    decimated_pixel_positions = [decimated_pixel_positions; pixel_positions(k,1) pixel_positions(k,2)];
end
num_decimated_pixels = length(decimated_pixel_positions);
I = spalloc(size(image_bw_edge,1), size(image_bw_edge,2), num_decimated_pixels);
for k=1:num_decimated_pixels
    I(decimated_pixel_positions(k,1), decimated_pixel_positions(k,2)) = 1;
end

% Rescale data to lie in [0,20]*[0,20]
max_x = max(decimated_pixel_positions(:,1));
max_y = max(decimated_pixel_positions(:,2));
decimated_pixel_positions(:,1) = 20*decimated_pixel_positions(:,1)/max_x;
decimated_pixel_positions(:,2) = 20*decimated_pixel_positions(:,2)/max_y;

figure, plot(decimated_pixel_positions(:,1), decimated_pixel_positions(:,2),'*')

for k=1:num_decimated_pixels
    text(decimated_pixel_positions(k,1), decimated_pixel_positions(k,2),num2str(k))
end
% Open a .node file with write permission
node_file = fopen('butterfly.node', 'w');

% Write first line of .node file
fprintf(node_file, '%d\t 2\t 0\t 1\n', num_decimated_pixels);

% Write decimated data to .node file
for k=1:num_decimated_pixels
    fprintf(node_file, '%d\t %6.6f\t %6.6f\t 1\n', k, decimated_pixel_positions(k,1), decimated_pixel_positions(k,2));
end

% Close .node file
fclose(node_file);

% Manually store the node connectivity
node_order = [68 67 66 65 64 63 62 61 59 58 56 51 49 48 46 44 43 42 40 39 36 ...
              35 33 21 1 2 3 4 5 6 7 17 14 12 10 11 9 8 19 25 28 22 20 16 18 ...
              15 13 23 24 26 27 29 30 31 32 34 37 38 41 45 47 50 52 53 54 55 ...
              57 60 69 70 71 75 79 81 82 86 87 89 90 91 93 99 100 101 102 103 ...
              104 105 106 107 108 109 111 110 113 114 115 119 123 125 124 127 ...
              133 134 122 118 116 117 120 121 129 131 132 126 130 135 136 137 ...
              138 139 140 141 128 112 98 97 96 95 94 92 88 85 84 83 80 78 77 ...
              76 74 73 72];

% Open a .poly file with write permission
poly_file = fopen('butterfly.poly', 'w');

% Write first line of .poly file (zero vertices indicates separate .node file)
fprintf(poly_file, '0\t 2\t 0\t 1\n');

% Write second section of .poly file
fprintf(poly_file, '%d\t 1\n', num_decimated_pixels);
for k=1:num_decimated_pixels-1
    fprintf(poly_file, '%d\t %d\t %d\t 1\n', k, node_order(k), node_order(k+1));
end
fprintf(poly_file, '%d\t %d\t %d\t 1\n', k, node_order(num_decimated_pixels), node_order(1));

% Write third section of .poly file (no holes)
fprintf(poly_file, '0\n');

% Close .poly file
fclose(poly_file);

% After running this script, type "triangle -pq butterfly" at the command
% line to generate a mesh