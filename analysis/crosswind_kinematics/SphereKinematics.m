function [r_ok_g, DCM_g2b, v_kg_g ,omega_bg_b, a_kg_g] =...
    SphereKinematics(Rs, psi_s, theta_s,...
    psid_s, thetad_s,...
    psidd_s, thetadd_s,...
    alpha, beta, phi_a,...
    alphad, betad, phid_a, v_ag_g)

% SphereKinematics.m
%
% Computes the consequences of kite motion on a sphere in wind.
% The kite motion is described using spherical coordinates with the
% assumption that the sphere radius is constant.
%
% The spherical coordinates, their time derivatives, the wind vector, and
% the specified kite alpha and beta combine to leave only a single degree
% of freedom for the kite state: a "roll" angle about the airspeed vector.
% The definition of "zero" for this angle is that the kite's wings are
% tangent to sphere (same as saying tether roll angle = 0 for a straight
% line tether).
%
% Inputs:
% Rs                Sphere Radius [m]
% psi_s             Azimuth [rad]
% theta_s           Elevation [rad]
% psid_s            Azimuth Rate [rad/s]
% thetad_s          Elevation Rate [rad/s]
% psidd_s           Azimuth Acceleration [rad/s^2]
% thetadd_s         Elevation Acceleration [rad/s^2]
% alpha             Kite angle of attack [rad]
% beta              Kite sideslip [rad]
% phi_a             "Roll" angle about the airspeed vector [rad]
% alphad            d/dt(Kite angle of attack) [rad/s]
% betad             d/dt(Kite sideslip) [rad/s]
% phid_a            d/dt("Roll" angle about the airspeed vector) [rad/s]
% v_ag_g            Velocity of Atmosphere WRT g, written in frame g [m/s]
%
% Outputs:
% r_ok_g            Kite position written in frame g [m]
% DCM_g2b           Direction Cosines Matrix of kite w.r.t. ground frame
% v_kg_g            Velocity of kite WRT g written in frame g [m/s]
% omega_bg_b        Angular Velocity of kite WRT g written in b [rad/s]
% v_ka_g            Kite Velocity WRT atmosphere written in frame g [m/s]
%
% Point Definitions:
% o                 Origin of frame g
% k                 Kite. Origin of frame b
%
% Frame Definitions:
% g                 "Ground" = NED
% s                 "Sphere" A rotation of NED. X axis points at kite.
%                                               Y axis horizontal.
% b                 "Body" = Rigidly attached to kite.
% f                 "Fly" X axis toward apparent wind,
%                         Y axis tangent to sphere
%                         Centered at the kite (point k)
% a                 "Atmosphere" = NED unit vector directions but with
%                                  the wind's translational motion



makani_home = getenv('MAKANI_HOME');
path(path,[makani_home '/analysis/plot/mabraham']);

%% Frame s Definition
% The x axis of Frame s points at the kite. The y axis of Frame s is
% parallel to the ground.
% Transformation from g to s is 1st an azimuth and then an elevation
DCM_g2s = L2(theta_s)*L3(psi_s);
DCM_s2g = DCM_g2s';

%% Kite Position
% Position vector from origin of frame g (point o) to kite written in
% frame s. By definition, frame s has its x axis pointing at the kite
r_ok_s = [Rs; 0; 0];

% Kite position in frame g
r_ok_g = DCM_s2g*r_ok_s;

%% Wind in Frame s
% Transform wind to frame s
v_ag_s = DCM_g2s*v_ag_g;

%% Angular Velocity of s WRT g
% Constructed from Euler Angle Rates
% d/dt(azimuth) along the azimuth axis + d/dt(elevation) along its axis
omega_sg_s =  [0; thetad_s; 0] + L2(theta_s)*[0; 0; psid_s];
omega_sg_g = DCM_s2g*omega_sg_s;

%% Angular Acceleration of s WRT g
% A direct time derivative of omega_sg_s
alpha_sg_s = [-psidd_s*sin(theta_s)-psid_s*thetad_s*cos(theta_s);...
    thetadd_s;...
    psidd_s*cos(theta_s)-psid_s*thetad_s*sin(theta_s)];

%% Kite Velocity WRT g
% Velocity of point k with respect to frame g written in frame s
% Assumes constant radius
v_kg_s = cross(omega_sg_s,r_ok_s);
v_kg_g = DCM_s2g*v_kg_s;

%% Kite Acceleration WRT g
% This assumes the sphere radius is constant
a_kg_s = cross(alpha_sg_s,r_ok_s) + ...
    cross(omega_sg_s,cross(omega_sg_s,r_ok_s));
a_kg_g = DCM_s2g*a_kg_s;

%% Kite Velocity WRT atmosphere
% Velocity of k with respect to the atmosphere
v_ka_s = v_kg_s - v_ag_s;
v_ka_g = DCM_s2g*v_ka_s;

%% Unit vector in the direction of kite airspeed
v_ka_hat_s = v_ka_s/norm(v_ka_s);
v_ka_hat_g = DCM_s2g*v_ka_hat_s;

%% Definition of the fly frame (transformation g to f)

% The Fly frame is defined by:
%    X-axis: The direction of the kite velocity relative to the atsmophere
%    Y-axis: Normal to X and tangent to the sphere
%    Z-axis: cross(x,y)

% Definition of the f frame i unit vector
if_g = v_ka_hat_g;

% Definition of the s frame i unit vector (it points at the kite)
% This is used to define the f frame j unit vector next
is_g = r_ok_g/norm(r_ok_g);

% Definition of the f frame j and k unit vectors
% The jf unit vector is the in the direction "airspeed cross sphere normal"
% This vector is tangent to the circle and normal to the airspeed
ifcrossis_g = cross(if_g,is_g);
jf_g = ifcrossis_g/norm(ifcrossis_g);

% The k unit vector is defined in a right handed sense
kf_g = cross(if_g,jf_g);

% Transformation from the fly frame to the ground frame
% DCM_f2g is frame f's unit vectors written in frame g components
DCM_f2g = [if_g, jf_g, kf_g];
DCM_g2f = DCM_f2g';

%% Angular velocity of f with respect to g (the only hard part)
% The g frame time derivative of the f frame i unit vector
% This is equal to cross(omega_fg_g, if_g)
%
% This time derivative is available to us analytically because:
% if is defined as the direction of the airspeed vector.
% The g time derivative of the airspeed vector is simply the acceleration
% of the kite with respect to frame g (here we assume the "mean" wind is
% changing slowly enough to have zero time derivative).
%
% The time derivative of a unit vector is normal to the unit vector (unit
% vectors only rotate). Therefore, to get the time derivative of the
% airspeed unit vector,
% 1. take the time derivative of the not-yet-normalized airspeed vector
%    (this is just the acceleration of the kite, a_kg)
% 2. Subtract the part parallel to the airspeed vector, keeping only the
%    rotational contribution. I do this with a projection operator.
% 3. Normalize by the airspeed. This scales the time derivative down,
%    correcting for the fact that we did not start with a unit vector in
%    step 1. What remains is the normalized rotational component of the g
%    frame time derivative of the airspeed vector <-- that's what we want.

gddt_if_g = (a_kg_g - if_g*if_g'*a_kg_g)/norm(v_ka_g);

% The g frame time derivative of the f frame j unit vector
% This is equal to cross(omega_fg_g, jf_g)
%
% This time derivative is available to us analytically because:
% jf is in the direction "airspeed cross sphere normal": cross(if,is).
% We can distribute a g-frame time derivative over this cross product with
% a product-rule style approach and then follow the 3 step procedure
% outlined above to extract the time derivative of jf.
% The time derivative of if is given above.
% The time derivative of is comes from the kite velocity which is given as
% an input.
%
% First, the time derivative of the is unit vector is computed using its
% definition and the 3 step procedure outlined above.
% This says: take the kite velocity, subtract the part normal to the sphere
% and scale down by the kite position.
gddt_is_g = (v_kg_g - if_g*if_g'*v_kg_g)/norm(r_ok_g);

% The time derivative of the cross product.
% The argument of this time derivative is not yet a unit vector.
% This is g-frame d/dt of cross(if_g,is_g)
gddt_ifcrossis_g = cross(gddt_if_g,is_g) + cross(if_g,gddt_is_g);

% We can now construct the time derivative of the j unit vector, also using
% the 3 step procedure desribed above.
gddt_jf_g = (gddt_ifcrossis_g - jf_g*jf_g'*gddt_ifcrossis_g)/...
             norm(ifcrossis_g);

% We now have enough information to reconstruct omega_fg_g using Kane's
% definition of the angular velocity vector:
%
% d/dt(DCM_f2g) = DCM_f2g*SkewSym(omega_fg_f)
%
% where SkewSym is the skew symmetric crossproduct operator such that
% cross(a,b) = SkewSym(a)*b
%
% SkewSym([x; y; z]) = [ 0  -z   y ]
%                      [ z   0  -x ]
%                      [-y   x   0 ]
%
% where omega_fg_f is the angular velocity of f with respect to g written
% using components in frame f
%
% Noting that a DCM transpose is the same as a DCM inverse, we can
% rearrange the equation to get
%
% SkewSym(omega_fg_f) = DCM_g2f*(d/dt(DCM_f2g))
%
% The definition of DCM_f2g is:
%
% DCM_f2g = [if_g,  jf_g, kf_g]

% This is the first and second columns of the skew symmetric matrix
% containing f frame components of omega_fg. No need to construct the third
% column because of symmetries.
SS12omega_fg_f= DCM_g2f*[gddt_if_g, gddt_jf_g];

% Extract the lower left triangle of the skew symmetric matrix
omega_fg_f = zeros(3,1);
omega_fg_f(1) = SS12omega_fg_f(3,2);
omega_fg_f(2) = -SS12omega_fg_f(3,1);
omega_fg_f(3) = SS12omega_fg_f(2,1);

%% DCM of the kite wrt g
DCM_f2b = L2(alpha)*L3(-beta)*L1(phi_a);
DCM_g2b = DCM_f2b*DCM_g2f;

%% Rotate some vectors
v_ka_f = DCM_g2f*v_ka_g;
a_kg_f = DCM_g2f*DCM_s2g*a_kg_s;

%% Angular velocity of b WRT f
% Constructed from euler angle rates.
% Because alpha and beta are specified and the kite is meant to roll about
% the airspeed vector, the Euler Angle sequence from frame f to frame b is
% "1-3-2" (roll-yaw-pitch) instead of the familiar "3-2-1" (yaw-pitch-roll)
%
% DCM_f2b = L2(alpha)*L3(-beta)*L1(phi_a)
%
% To construct the angular velocity vector associated with this DCM, do the
% usual: sum angular velocity vectors associated with each individual euler
% angle rate.
omega_bf_b = [0; alphad; 0] + L2(alpha)*[0; 0; -betad] +...
    L2(alpha)*L3(-beta)*[phid_a; 0; 0];

%% Angular velocity of Kite wrt ground
omega_bg_b = omega_bf_b + DCM_f2b*omega_fg_f;

%% Acceleration of kite wrt Ground written in ground frame
a_kg_g = DCM_s2g*a_kg_s;

%% Plotting
if 1
    load m600geometry rawm600

    % Scale factor for velocity vector drawing
    vsf = 1.0;

    % Draw the groundspeed vector
    plot3(r_ok_g(2)+[0 v_kg_g(2)]*vsf,r_ok_g(1)+[0 v_kg_g(1)]*vsf,...
        -r_ok_g(3)+[0 -v_kg_g(3)]*vsf,'r-','LineWidth',3);
    hold on
    % Draw the airspeed vector
    plot3(r_ok_g(2)+[0 v_ka_g(2)]*vsf,r_ok_g(1)+[0 v_ka_g(1)]*vsf,...
        -r_ok_g(3)+[0 -v_ka_g(3)]*vsf,'c-','LineWidth',3);

    % Complete the airspeed triangle
    plot3(r_ok_g(2)+vsf*(v_kg_g(2)-[0 v_ag_g(2)]),...
        r_ok_g(1)+vsf*(v_kg_g(1)-[0 v_ag_g(1)]),...
        -(r_ok_g(3)+vsf*(v_kg_g(3)-[0 v_ag_g(3)])),'b','LineWidth',3)

    % Draw a line representing the straight line tether
    plot3([0 r_ok_g(2)],[0 r_ok_g(1)],[0 -r_ok_g(3)],'k','LineWidth',3);

    % Draw the kite
    DrawM600(r_ok_g(1),r_ok_g(2),r_ok_g(3),...
        DCM_g2b,rawm600);

    % Draw the sphere
    [az,el] = meshgrid(psi_s*180/pi+[-5:5],theta_s*180/pi+[-5:5]);
    x = Rs*cosd(az).*cosd(el);
    y = Rs*sind(az).*cosd(el);
    z = -Rs*sind(el);
    plot3(y,x,-z,'k')
    plot3(y',x',-z','k')

    % Set the axes for a nice view
    axis equal;
    axis([r_ok_g(2)+[-30 30] r_ok_g(1)+[-30 30] -r_ok_g(3)+[-30 30]]);
end