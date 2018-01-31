clear all;clc;close all;clf;hold on;grid minor;axis equal
xlabel('x');ylabel('y');zlabel('z');
           
% LOAD CLOUDS
disp('Loading clouds...');
cloud1 = pcread('clouds3/0.pcd');
cloud2 = pcread('clouds3/1.pcd');
cloud3 = pcread('clouds3/2.pcd');

showcloud1 = cloud2;
showcloud1_idx = 2;
showcloud2 = cloud3;
showcloud2_idx = 3;

% DEFINE PARAMETERS
axis_line_length = 0.1;
global_axes = [axis_line_length 0 0 0;
               0 axis_line_length 0 0;
               0 0 axis_line_length 0;
               0 0 0 1];

% POINTS AND ROTATIONS
% XYZ
pos = [0.56455,0.34409,0.48872;
       0.58125,-0.03703,0.62452;
       0.44283,-0.23750,0.48174
       ];
% XYZ
rot_deg = [-138.99,12.69,136.66;
           -158.06,1.03,66.26;
           -148.61,-5.85,20.39
           ];
rot = rot_deg .* (1/180*pi);

R_HtoB = {};
T_tform = {};
T_HtoB_tform = {};
for i = 1:size(pos,1)
    RZ = [cos(rot(i,3)) -sin(rot(i,3)) 0;
          sin(rot(i,3)) cos(rot(i,3)) 0;
          0 0 1];
    RY = [cos(rot(i,2)) 0 sin(rot(i,2));
          0 1 0;
          -sin(rot(i,2)) 0 cos(rot(i,2))];
    RX = [1 0 0;
          0 cos(rot(i,1)) -sin(rot(i,1));
          0 sin(rot(i,1)) cos(rot(i,1))];
      
    R_HtoB{i} = RZ*RY*RX;
    T_HtoB{i} = [R_HtoB{i}' zeros(3,1) ; pos(i,:) 1];
    T_HtoB_tform{i} = affine3d(T_HtoB{i});
end

L = load('T_CtoH.mat');
T_CtoH = L.T_CtoH;
% T_CtoH = [1 0 0 0;
%           0 1 0 0;
%           0 0 1 0;
%           0 0 .2 1];
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

pause

plot3([0 C1_axes(1,1)]+C1_axes(4,1), [0 C1_axes(1,2)]+C1_axes(4,2), [0 C1_axes(1,3)]+C1_axes(4,3), 'r');
plot3([0 C1_axes(2,1)]+C1_axes(4,1), [0 C1_axes(2,2)]+C1_axes(4,2), [0 C1_axes(2,3)]+C1_axes(4,3), 'g');
plot3([0 C1_axes(3,1)]+C1_axes(4,1), [0 C1_axes(3,2)]+C1_axes(4,2), [0 C1_axes(3,3)]+C1_axes(4,3), 'b');
text(C1_axes(4,1), C1_axes(4,2), C1_axes(4,3), ['  C' num2str(showcloud1_idx)]);

plot3([0 C2_axes(1,1)]+C2_axes(4,1), [0 C2_axes(1,2)]+C2_axes(4,2), [0 C2_axes(1,3)]+C2_axes(4,3), 'r');
plot3([0 C2_axes(2,1)]+C2_axes(4,1), [0 C2_axes(2,2)]+C2_axes(4,2), [0 C2_axes(2,3)]+C2_axes(4,3), 'g');
plot3([0 C2_axes(3,1)]+C2_axes(4,1), [0 C2_axes(3,2)]+C2_axes(4,2), [0 C2_axes(3,3)]+C2_axes(4,3), 'b');
text(C2_axes(4,1), C2_axes(4,2), C2_axes(4,3), ['  C' num2str(showcloud2_idx)]);

pause

showcloud1 = pctransform(showcloud1, T_CtoH_tform);
showcloud1 = pctransform(showcloud1, T_HtoB_tform{showcloud1_idx});
showcloud2 = pctransform(showcloud2, T_CtoH_tform);
showcloud2 = pctransform(showcloud2, T_HtoB_tform{showcloud2_idx});

pcshow(showcloud1);
pcshow(showcloud2);







