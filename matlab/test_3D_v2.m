clear all;
           
% LOAD CLOUDS
disp('Loading clouds...');
clouds = {pcread('MergeTests/0.pcd'), ...
          pcread('MergeTests/1.pcd'), ...
          pcread('MergeTests/2.pcd'), ...
          pcread('MergeTests/3.pcd'), ...
          pcread('MergeTests/4.pcd'), ...
          pcread('MergeTests/5.pcd'), ...
          pcread('MergeTests/6.pcd'), ...
          pcread('MergeTests/7.pcd'), ...
          pcread('MergeTests/8.pcd'), ...
          pcread('MergeTests/9.pcd')};
disp('DONE!');

%%

clc;close all;clf;hold on;grid minor;axis equal

xlabel('x');ylabel('y');zlabel('z');
showcloud1_idx = 5;
showcloud2_idx = 6;

showcloud1_idx = showcloud1_idx + 1;
showcloud2_idx = showcloud2_idx + 1;

% DEFINE PARAMETERS
axis_line_length = 0.1;
global_axes = [axis_line_length 0 0 0;
               0 axis_line_length 0 0;
               0 0 axis_line_length 0;
               0 0 0 1];

% Quat data
quat_data = [352.31,442.68,400.77,0.00269698,-0.061463,0.88315,-0.465039;
             543.49,319.76,423.88,0.0701989,-0.138667,-0.923328,0.351153;
             620.07,177.3,453.02,0.105299,-0.299859,-0.911761,0.260171;
             522.91,123.79,580.77,0.0274001,-0.42495,-0.892305,0.149861;
             569.78,-87.15,505.67,0.0999359,-0.822791,-0.547676,0.114361;
             365.24,372.95,362.69,0.026817,-0.0630976,0.905641,-0.418466;
             462.06,400.66,277.43,0.0413685,-0.0210879,-0.88155,0.469802;
             630.89,199.76,428.87,0.076134,-0.374407,-0.872353,0.304998;
             223.97,-136.44,536.23,0.156587,-0.706511,0.687414,-0.0615223;
             273.6,-255.61,454.2,0.221342,-0.768101,0.600845,0.00382677];
pos = quat_data(:,1:3) / 1000;
quat = quat_data(:,4:7);
         
R_HtoB = {};
T_tform = {};
T_HtoB_tform = {};
for i = 1:size(pos,1)
    R_HtoB{i} = quat2rotm(quat(i,:));
    T_HtoB{i} = [R_HtoB{i}' zeros(3,1) ; pos(i,:) 1];
    T_HtoB_tform{i} = affine3d(T_HtoB{i});
end

L = load('T_CtoH.mat');
T_CtoH = L.T_CtoH;
T_CtoH_tform = affine3d(T_CtoH);

% Transform axes [x y z]
H1_axes = global_axes*T_HtoB{showcloud1_idx}; 
H2_axes = global_axes*T_HtoB{showcloud2_idx};
C1_axes = global_axes*T_CtoH*T_HtoB{showcloud1_idx}; 
C2_axes = global_axes*T_CtoH*T_HtoB{showcloud2_idx};

% PLOT axes
plot3([0 H1_axes(1,1)]+H1_axes(4,1), [0 H1_axes(1,2)]+H1_axes(4,2), [0 H1_axes(1,3)]+H1_axes(4,3), 'r');
plot3([0 H1_axes(2,1)]+H1_axes(4,1), [0 H1_axes(2,2)]+H1_axes(4,2), [0 H1_axes(2,3)]+H1_axes(4,3), 'g');
plot3([0 H1_axes(3,1)]+H1_axes(4,1), [0 H1_axes(3,2)]+H1_axes(4,2), [0 H1_axes(3,3)]+H1_axes(4,3), 'b');
text(H1_axes(4,1), H1_axes(4,2), H1_axes(4,3), ['  H' num2str(showcloud1_idx)]);

plot3([0 H2_axes(1,1)]+H2_axes(4,1), [0 H2_axes(1,2)]+H2_axes(4,2), [0 H2_axes(1,3)]+H2_axes(4,3), 'r');
plot3([0 H2_axes(2,1)]+H2_axes(4,1), [0 H2_axes(2,2)]+H2_axes(4,2), [0 H2_axes(2,3)]+H2_axes(4,3), 'g');
plot3([0 H2_axes(3,1)]+H2_axes(4,1), [0 H2_axes(3,2)]+H2_axes(4,2), [0 H2_axes(3,3)]+H2_axes(4,3), 'b');
text(H2_axes(4,1), H2_axes(4,2), H2_axes(4,3), ['  H' num2str(showcloud2_idx)]);

%pause

plot3([0 C1_axes(1,1)]+C1_axes(4,1), [0 C1_axes(1,2)]+C1_axes(4,2), [0 C1_axes(1,3)]+C1_axes(4,3), 'r');
plot3([0 C1_axes(2,1)]+C1_axes(4,1), [0 C1_axes(2,2)]+C1_axes(4,2), [0 C1_axes(2,3)]+C1_axes(4,3), 'g');
plot3([0 C1_axes(3,1)]+C1_axes(4,1), [0 C1_axes(3,2)]+C1_axes(4,2), [0 C1_axes(3,3)]+C1_axes(4,3), 'b');
text(C1_axes(4,1), C1_axes(4,2), C1_axes(4,3), ['  C' num2str(showcloud1_idx)]);

plot3([0 C2_axes(1,1)]+C2_axes(4,1), [0 C2_axes(1,2)]+C2_axes(4,2), [0 C2_axes(1,3)]+C2_axes(4,3), 'r');
plot3([0 C2_axes(2,1)]+C2_axes(4,1), [0 C2_axes(2,2)]+C2_axes(4,2), [0 C2_axes(2,3)]+C2_axes(4,3), 'g');
plot3([0 C2_axes(3,1)]+C2_axes(4,1), [0 C2_axes(3,2)]+C2_axes(4,2), [0 C2_axes(3,3)]+C2_axes(4,3), 'b');
text(C2_axes(4,1), C2_axes(4,2), C2_axes(4,3), ['  C' num2str(showcloud2_idx)]);

%pause

showcloud1 = pctransform(clouds{showcloud1_idx}, T_CtoH_tform);
showcloud1 = pctransform(showcloud1, T_HtoB_tform{showcloud1_idx});
showcloud2 = pctransform(clouds{showcloud2_idx}, T_CtoH_tform);
showcloud2 = pctransform(showcloud2, T_HtoB_tform{showcloud2_idx});

pcshow(showcloud1);
pcshow(showcloud2);







