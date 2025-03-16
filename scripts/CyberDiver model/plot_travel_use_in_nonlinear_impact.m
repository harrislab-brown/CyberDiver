%% parameters

% fixed
radius = 0.027;  % m
water_density = 1000;  % kg/m^3
mass_total = 0.828;  % kg
mass_ratio = 0.228;  % nose mass divided by total mass
tank_depth = 0.7;  % m, for determining how long to run simulation

% change
stiffness = 45000;  % N/m
speed = linspace(1, 7, 50);  % m/s  -> these arrays dictate which simulations to run
accel_limit = linspace(0.25, 4.75, 50);  % g


%% simulation
[m_func, dmdx_func] = get_sphere_added_mass(radius, water_density);
max_disp = zeros(length(accel_limit), length(speed));  % mm, this is xb - xn, so positive is compression (we want to track the largest positive value)

for i = 1:length(accel_limit)
    force_limit = 9.81 * accel_limit(i) * mass_total * (1 - mass_ratio);  % N, max force on impactor body
    structure_func = @(x, v) nonlinear_structure(x, stiffness, force_limit);
    for j = 1:length(speed)
        t_max = tank_depth / speed(j);
        [t, xn, xb, fn, fb] = two_way_coupled_impactor_response(m_func, dmdx_func, structure_func, mass_total, mass_ratio, speed(j), t_max);
        max_disp(i, j) = 1000 * max(xb - xn);
    end
end


%% plotting
figure
set(gcf, 'color', 'w')
hold on
box on
xlabel('Impact speed (m/s)')
ylabel('Deceleration limit (g)')
xlim([1 7])
ylim([0.25 4.75])
levels = 0:6;
colormap(jet)
contourf(speed, accel_limit, max_disp, levels, 'ShowText', 'off', 'LabelFormat', "%d mm", 'FaceAlpha', 0.25)


%% functions

function force = nonlinear_structure(x, stiffness, force_limit)
    width_of_linear_region = force_limit / stiffness;
    if x < -width_of_linear_region
        force = -force_limit;
    elseif x > width_of_linear_region
        force = force_limit;
    else
        force = stiffness * x;
    end
end