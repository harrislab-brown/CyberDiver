function [t, xn, xb, fn, fb] = two_way_coupled_impactor_response(m_func, dmdx_func, structure_func, mass_total, mass_ratio, speed, t_max)


% computes the response of a flexible impactor consisting of two
% masses connected by a structural element using the two-way coupled model.
% we assume that the added mass associated with impact is a function of
% depth only which allows an ODE formulation of the impactor response. The
% required inputs are the added mass as a function of depth and its
% derivative. For the sphere, we use the solution from Shiffman and
% Spencer's theory in their 1945 paper, since this agrees well with our
% experiments with a rigid impactor with hemispherical nose. However, in
% the limit of large impactor mass relative to water added mass, the force
% of impact of a rigid impactor with arbitrary nose geometry should
% directly give a scaled version of the derivative of the added mass for 
% that goemetry, which could then be used in this theory.

% we neglect gravity and buoyancy but these would be straightforward to
% add. The effects of form drag (i.e. the steady state drag to which the 
% behavior transitions at sufficient depth) are also neglected although
% experiments show that these can have a substantial effect. It would be 
% more difficult to add form drag to the model in a consistent manner.

% mass_total is the total mass of the impactor.

% mass_ratio is the nose mass divided by the total impactor mass 
% (represented by alpha in the paper).

% speed is the speed of the impactor at the moment of impact

% t_max is how long to run the model for (t = 0 is the moment of initial
% impact).

% structure_func is a function handle that should take in the displacement
% (xb-xn) and its rate (\dot{x_b} - \dot{x_n}) and return the restoring 
% force from the structural element that connects the nose and body masses

% m_func and dmdx_func are function handles that take the nose depth and
% return the added mass of the water (and its derivative). These encode the
% size and shape of the impactor nose and the water density.

% returns time stamps and the position and forcing on the nose and body



x0 = [0, speed, 0, speed];  % x_n, \dot{x_n}, x_b, \dot{x_b}; positive is down into the water
opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[t, x] = ode45(@eq, [0, t_max], x0, opts); 

xn = x(:, 1);
xb = x(:, 3);

% recompute the forces
n = length(t);
fn = zeros(n, 1);
fb = zeros(n, 1);
for i = 1:n  % boomer loop so we don't require the caller to supply vectorized functions
    fn(i) = mass_ratio * mass_total / (mass_ratio * mass_total + m_func(xn(i))) * (structure_func(xb(i) - xn(i), x(i, 4) - x(i, 2)) - dmdx_func(xn(i)) * x(i, 2) * x(i, 2));
    fb(i) = -structure_func(xb(i) - xn(i), x(i, 4) - x(i, 2));
end

% function for ODE solver    
function dxdt = eq(~, x)
    dxdt = zeros(4, 1);

    % nose
    dxdt(1) = x(2);
    dxdt(2) = 1 / (mass_ratio * mass_total + m_func(x(1))) * (structure_func(x(3) - x(1), x(4) - x(2)) - dmdx_func(x(1)) * x(2) * x(2));
    
    % body
    dxdt(3) = x(4);
    dxdt(4) = -structure_func(x(3) - x(1), x(4) - x(2)) / ((1 - mass_ratio) * mass_total);
end


end