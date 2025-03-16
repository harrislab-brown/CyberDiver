function [m_func, dmdx_func] = get_sphere_added_mass(radius, water_density)

dmdx_table = readtable('shiffman_spencer_sphere_dmdx.csv'); 
x = radius .* dmdx_table.x;  % depth in physical units
dmdx = 0.5 * water_density * pi * radius^2 .* dmdx_table.y;  % derivative of added mass in physical units
m = cumtrapz(x, dmdx);  % integrate to get added mass in physical units


m_func = @(xq) interp1(x, m, xq, 'linear', m(end)); 
dmdx_func = @(xq) interp1(x, dmdx, xq, 'linear', 0);  % return zero if evaluated out of bounds

end