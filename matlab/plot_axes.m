function plot_axes(ax, thickness)
    % Plots axes defined as a 4x4 matrix 
    h1 = plot3([ax(4,1) ax(1,1)], [ax(4,2) ax(2,1)], [ax(4,3) ax(3,1)], 'r-');
    h2 = plot3([ax(4,1) ax(1,2)], [ax(4,2) ax(2,2)], [ax(4,3) ax(3,2)], 'g-');
    h3 = plot3([ax(4,1) ax(1,3)], [ax(4,2) ax(2,3)], [ax(4,3) ax(3,3)], 'b-');
    
    set(h1, 'LineWidth', thickness);
    set(h2, 'LineWidth', thickness);
    set(h3, 'LineWidth', thickness);
end