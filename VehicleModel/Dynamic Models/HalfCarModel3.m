function [Zdot] = HalfCarModel3(t,z,hcar)
%HALFCARMODEL Half Car Model to be called by the Numerical Integrator
%   The HalfCar is modelled in this function. 
%   The equations of motion of the car are modelled here in a fashion which
%   can be used by the numerical integrator. 

%% Initializing the states of this time-step
zs = z(1);
phi = z(2);
za1 = z(3);
za2 = z(4);
zs_d = z(5);
phi_d = z(6);
za1_d = z(7);
za2_d = z(8);

%% Extracting the Input of the current time-step

% --------------------- ACCELERATION INPUT ------------------------ %
acc = interp1(hcar.simInputs.time,hcar.simInputs.acc,t,'linear','extrap')*1;

% -------------------- FORCE INPUT | ON CONTACT PATCH--------------------- %
fxy_1 = interp1(hcar.simInputs.time,hcar.simInputs.Fxy_1,t,'linear','extrap')*1;
fxy_2 = interp1(hcar.simInputs.time,hcar.simInputs.Fxy_2,t,'linear','extrap')*1;

% --------------------- ROAD INPUT ------------------------ %
% Computing delay of the input signal to reach rear axle
delay = hcar.w2w/hcar.simInputs.vx;
zr1 = interp1(hcar.simInputs.time,hcar.simInputs.zr1,t+delay,'linear','extrap')*1;
zr2 = interp1(hcar.simInputs.time,hcar.simInputs.zr2,t,'linear','extrap')*1;

zr1_d = interp1(hcar.simInputs.time,hcar.simInputs.zr1_d,t+delay,'linear','extrap')*1;
zr2_d = interp1(hcar.simInputs.time,hcar.simInputs.zr2_d,t,'linear','extrap')*1;

%% Initalizing the Deflection and Velocites of the current time-step
% Suspension Deflection and Velocities
defSus1 = za1 - zs + hcar.dis_a12cg*phi;
defSus2 = za2 - zs - hcar.dis_a22cg*phi;
velSus1 = za1_d - zs_d + hcar.dis_a12cg*phi_d;
velSus2 = za2_d - zs_d - hcar.dis_a22cg*phi_d;

% Tire Deflection and Velocities
defT1 = zr1 - za1;
defT2 = zr2 - za2;
velT1 = zr1_d - za1_d;
velT2 = zr2_d - za2_d;

%% Initializing the Component Parameters for the current time-step

% Extracing Instantaneous Spring and Damper Forces
fs1 = hcar.coil1.Get_Force(defSus1,hcar.coil1.stroke); %25000;
fs2 = hcar.coil2.Get_Force(defSus2,hcar.coil1.stroke); %18000;
fd1 = hcar.damper1.Get_Force(velSus1);%1700;
fd2 = hcar.damper2.Get_Force(velSus2);%1200;

% Tire Stiffness and Damping
kt1 = 2e5;
kt2 = 2e5;
dt1 = 8;
dt2 = 8;
g = 9.81;

%% Initializing the Weight Transfer Components due to Acceleration Input

% Geometric Weight Transfer = Jacking Force
fz1 = fxy_1*tan(hcar.axleSusp.th_1); 
fz2 = fxy_2*tan(hcar.axleSusp.th_2);

% Non-Supended Weight Transfer
fnsm = (((hcar.mass_usm_1+hcar.mass_usm_2)/2)*acc*hcar.cg_USM_Height)/hcar.w2w;


%% Modelling the Half Car - Equations of Motion

zs_dd = fs1/hcar.mass_sm + fd1/hcar.mass_sm - g...
        + fs2/hcar.mass_sm + fd2/hcar.mass_sm...
        + (fz1 - fz2)/hcar.mass_sm;

phi_dd = -hcar.dis_a12cg*fs1/hcar.Inertia - hcar.dis_a12cg*fd1/hcar.Inertia...
         + hcar.dis_a22cg*fs2/hcar.Inertia + hcar.dis_a22cg*fd2/hcar.Inertia...
         - ((fxy_1 + fxy_2)/hcar.Inertia)*(hcar.cg_SM_Height - hcar.axleSusp.rotCenterHeight)...
         + (fz1*hcar.dis_a12cg + fz2*hcar.dis_a22cg)/hcar.Inertia;

za1_dd = -fs1/hcar.mass_usm_1 - fd1/hcar.mass_usm_1 - g...
         + kt1/hcar.mass_usm_1*(defT1) + dt1/hcar.mass_usm_1*(velT1)...
         + (fnsm + fz1)/hcar.mass_usm_1;

za2_dd = -fs2/hcar.mass_usm_2 - fd2/hcar.mass_usm_2 - g...
         + kt2/hcar.mass_usm_2*(defT2) + dt2/hcar.mass_usm_2*(velT2)... 
         - (fnsm + fz2)/hcar.mass_usm_2;
 
%% Conditiong the equations to reflect the stat space form

Zdot = [zs_d;
        phi_d;
        za1_d;
        za2_d;
        zs_dd;
        phi_dd;
        za1_dd;
        za2_dd];

end

