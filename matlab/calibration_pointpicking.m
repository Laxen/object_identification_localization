function calibration_pointpicking(P, data)
    P_raw = str2num(P);
    data_raw = str2num(data);
    
    P = {};
    for i=1:size(P_raw,1)
        P{i} = reshape(P_raw(i,:), [3 size(P_raw,2)/3])';
        P{i} = [P{i}' ; ones(1, size(P{i},1))];
    end
    
    R_HtoB = {};
    T_HtoB = {};
    
    % Data is in Euler format
    if size(data_raw(1,:),2) == 6
        pos = zeros(size(data_raw,1),3);
        rot_deg = zeros(size(data_raw,1),3);
        for i=1:size(data_raw,1)
            data = reshape(data_raw(i,:), [3 2])';
            pos(i,:) = data(1,:);
            rot_deg(i,:) = data(2,:);
        end

        rot = rot_deg .* (1/180*pi);

        % Compute transformation matrices from robot data
        disp('Computing robot transformation matrices...')
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
        end
    elseif size(data_raw(1,:),2) == 7
        pos = zeros(size(data_raw,1),3);
        quat = zeros(size(data_raw,1),4);
        for i=1:size(data_raw,1)
            pos(i,:) = data_raw(i,1:3);
            quat(i,:) = data_raw(i,4:7);
            R_HtoB{i} = quat2rotm(quat(i,:));
            T_HtoB{i} = [R_HtoB{i} pos(i,:)'/1000 ; zeros(1,3) 1];
            disp(['T for ' num2str(i) ' = '])
            T_HtoB{i}
        end
    else
        disp('ERROR: Input data matrix does not have correct format');
    end

    % Use points to compute transformation
    disp('Computing camera to hand transformation matrix...');
    
    %T = inv(T_HtoB{1}) * T_HtoB{2};
    T = {};
    for i=2:size(P,2)
        T{i} = inv(T_HtoB{1}) * T_HtoB{i};
    end
    
    f = @(x)myfun(x, T, P);
    options = optimoptions(@lsqnonlin, 'FunctionTolerance', 1.0e-10, 'OptimalityTolerance', 1.0e-10, 'MaxFunctionEvaluations', 2000);
    x = lsqnonlin(f, [1 0 0 0 0 0 0.2], [], [], options);
    w = x(1);
    a = x(2);
    b = x(3);
    c = x(4);
    t1 = x(5);
    t2 = x(6);
    t3 = x(7);
    
    T_CtoH = [quat2rotm([w a b c]) [t1 t2 t3]' ; zeros(1,3) 1]

%     x = lsqnonlin(f, [0 0 0.2], [], [], options);
%     t1 = x(1);
%     t2 = x(2);
%     t3 = x(3);
%     
%     T_CtoH = [eye(3) [t1 t2 t3]' ; zeros(1,3) 1]

    fid = fopen('T_CtoH', 'w');
    fprintf(fid, '%15.14f,%15.14f,%15.14f,%15.14f,\n', T_CtoH);
    fclose(fid);
    disp('Camera to hand transformation matrix saved as T_CtoH');
end

function F = myfun(x, T, P)
%     a = x(1);
%     b = x(2);
%     g = x(3);
%     t1 = x(4);
%     t2 = x(5);
%     t3 = x(6);
%     
%     T_CtoH = [cos(a)*cos(b) cos(a)*sin(b)*sin(g)-sin(a)*cos(g) cos(a)*sin(b)*cos(g)+sin(a)*sin(g) t1;
%         sin(a)*cos(b) sin(a)*sin(b)*sin(g)+cos(a)*cos(g) sin(a)*sin(b)*cos(g)-cos(a)*sin(g) t2;
%         -sin(b) cos(b)*sin(g) cos(b)*cos(g) t3;
%         0 0 0 1];

    w = x(1);
    a = x(2);
    b = x(3);
    c = x(4);
    t1 = x(5);
    t2 = x(6);
    t3 = x(7);
    
    T_CtoH = [quat2rotm([w a b c]) [t1 t2 t3]' ; zeros(1,3) 1];

%     t1 = x(1);
%     t2 = x(2);
%     t3 = x(3);
%     
%     T_CtoH = [eye(3) [t1 t2 t3]' ; zeros(1,3) 1];
    
    F = [];
    for i=2:size(T,2)
        F_new = P{1} - inv(T_CtoH)*T{i}*T_CtoH*P{i};
        F_new = F_new(1:3,:); % Removes the row of zeroes caused by the homogeneous coordinates
        F = [F ; F_new];
        
        %F = [F ; P{1} - inv(T_CtoH)*T{i}*T_CtoH*P{i}]
    end
end





