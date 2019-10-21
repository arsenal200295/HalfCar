function [Zdot] = HalfCarModel6(t,z,hcar)
%HALFCARMODEL4 ITERATION 4 -  Half Car Model to be called by the Numerical Integrator
%   This iteration contains weight transfer equations modelled using
%   suspension kinematics. Hence, the elastic, geometric and non-suspended
%   weight transfers are modelled. 
%   The moments of Long and Jacking Forces are taken at the CG in this
%   iteration
%   The HalfCar is modelled in this function. 
%   The equations of motion of the car are modelled here in a fashion which
%   can be used by the numerical integrator. 

%% Initializing the states of this time-step
% Independant States
zs = z(1);
phi = z(2);
za1 = z(3);
za2 = z(4);
zs_d = z(5);
phi_d = z(6);
za1_d = z(7);
za2_d = z(8);

init = evalin('base','init');

%% Extracting the Input of the current time-step

% --------------------- ACCELERATION INPUT ------------------------ %
acc = interp1(hcar.simInputs.time,hcar.simInputs.acc,t,'linear','extrap')*1;

% -------------------- FORCE INPUT | ON CONTACT PATCH --------------------- %
fxy_1 = interp1(hcar.simInputs.time,hcar.simInputs.Fxy_1,t,'linear','extrap')*1;
fxy_2 = interp1(hcar.simInputs.time,hcar.simInputs.Fxy_2,t,'linear','extrap')*1;

% -------------------- MOMENT INPUT | ON CONTACT PATCH --------------------- %
myx_1 = interp1(hcar.simInputs.time,hcar.simInputs.M_1,t,'linear','extrap')*0;
myx_2 = interp1(hcar.simInputs.time,hcar.simInputs.M_2,t,'linear','extrap')*0;

% --------------------- ROAD INPUT ------------------------ %
% Computing delay of the input signal to reach rear axle
delay = hcar.w2w/hcar.simInputs.vx;
zr1 = interp1(hcar.simInputs.time,hcar.simInputs.zr1,t+delay,'linear','extrap')*0;
zr2 = interp1(hcar.simInputs.time,hcar.simInputs.zr2,t,'linear','extrap')*0;

zr1_d = interp1(hcar.simInputs.time,hcar.simInputs.zr1_d,t+delay,'linear','extrap')*0;
zr2_d = interp1(hcar.simInputs.time,hcar.simInputs.zr2_d,t,'linear','extrap')*0;


%% Initializing Axle Suspension Parameters

% IC Arm - Hyp of ICH and VSAL
hyp1 = hcar.axleSusp.hyp_1;
hyp2 = hcar.axleSusp.hyp_2;

% Static - IC Arm Angle
ths1 = hcar.axleSusp.th_1;
ths2 = hcar.axleSusp.th_2;

% Variation - IC Arm Angle
tha1 = -(za1 - init(3))/hyp1;
tha2 = (za2 - init(4))/hyp2;

% Total - IC Arm Angle
th1 = ths1 + tha1;
th2 = ths2 + tha2;

%% Initializing Mass and Inertia Properties
% Mass
ms = hcar.mass_sm;
ma1 = hcar.mass_usm_1;
ma2 = hcar.mass_usm_2;

% Inertia
Is = hcar.Inertia;
 
% Inertial of NSMs about Pitch Centers
Iapc1 = hcar.Inertia_NSM_1*0 + hcar.mass_usm_1*hyp1^2;
Iapc2 = hcar.Inertia_NSM_2*0 + hcar.mass_usm_2*hyp2^2;

%% Initalizing the Deflection and Velocites of the current time-step
% Suspension Deflection and Velocities
defSus1 = (za1) - zs + hcar.dis_a12cg*phi;
defSus2 = (za2) - zs - hcar.dis_a22cg*phi;
velSus1 = (za1_d) - zs_d + hcar.dis_a12cg*phi_d;
velSus2 = (za2_d) - zs_d - hcar.dis_a22cg*phi_d;

% Tire Deflection and Velocities
defT1 = zr1 - (za1);
defT2 = zr2 - (za2);
velT1 = zr1_d - (za1_d);
velT2 = zr2_d - (za2_d);

%% Initializing the Component Parameters for the current time-step

% Extracing Instantaneous Spring and Damper Forces
fs1 = hcar.coil1.Get_Force(defSus1,hcar.coil1.stroke); %25000;
fs2 = hcar.coil2.Get_Force(defSus2,hcar.coil1.stroke); %18000;
fd1 = hcar.damper1.Get_Force(velSus1);%1700;
fd2 = hcar.damper2.Get_Force(velSus2);%1200;

% Tire Stiffness and Damping
kt1 = hcar.tire1.kt;
kt2 = hcar.tire2.kt;
dt1 = hcar.tire1.dt;
dt2 = hcar.tire2.dt;
g = 9.81;

% Tire forces
fkt1= kt1*defT1;
fkt2 = kt2*defT2;
fdt1 = dt1*velT1;
fdt2 = dt2*velT2;

%% Initializing the Weight Transfer Components due to Acceleration Input

% Gravitational Forces
fgs = ms*9.81;
fga1 = ma1*9.81;
fga2 = ma2*9.81;

% Geometric Weight Transfer = Jacking Force
% fz1 = kt1*(tha1/hyp1); % Reaction component on Sprung Mass
% fz2 = -kt2*(tha2/hyp2); % Reaction component on Sprung Mass
fz1_s = -fxy_1*tan(th1) + (myx_1/hcar.axleSusp.vsal_1);% Reaction component on Sprung Mass
fz2_s = fxy_2*tan(th2) - (myx_2/hcar.axleSusp.vsal_2);% Reaction component on Sprung Mass

fz1_u = -fz1_s; % Action component of jacking force on Unsprung Mass
fz2_u =  -fz2_s;% Action component of jacking force on Unsprung Mass

% Non-Supended Weight Transfer
fnsm = (((ma1 + ma2)/2)*acc*hcar.cg_USM_Height)/hcar.w2w;

% Net Vertical Load on Unsprung Mass || Used in Moment Equation to find instant center angle change
fnetv1 = fkt1 + fdt1  + fz1_u*0 + fnsm - fga1 - fs1 - fd1;
fnetv2 = fkt2 + fdt2  + fz2_u*0 - fnsm - fga2 - fs2 - fd2;

%% Test
if fxy_2  ~= 0
    a = 1;
end

%% Modelling the Half Car - Equations of Motion
zs_dd = (fs1 + fd1 - fgs + fs2 + fd2 + (fz1_s + fz2_s))/ms;

phi_dd = (-hcar.dis_a12cg*fs1 - hcar.dis_a12cg*fd1...
         + hcar.dis_a22cg*fs2 + hcar.dis_a22cg*fd2...
         - (fxy_1 + fxy_2)*(hcar.cg_SM_Height)...
         - (fz1_s*hcar.dis_a12cg) + (fz2_s*hcar.dis_a22cg) )/Is;

    th1_dd = (myx_1 - fnetv1*hyp1*sin(pi/2 + th1)- fxy_1*hyp1*sin(th1))/...
             (Iapc1);

za1_dd = (-fs1 - fd1 - fga1 + fkt1 + fdt1 + fnsm + fz1_u)/hcar.mass_usm_1 - ...
          th1_dd*(hyp1);

    th2_dd = (myx_2 + fnetv2*hyp2*sin(pi/2 + th2)- fxy_2*hyp2*sin(th2))/...
             (Iapc2);
      
za2_dd = (-fs2 - fd2- fga2 + fkt2 + fdt2 - fnsm + fz2_u)/hcar.mass_usm_2 + ...
          th2_dd*(hyp2);


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

