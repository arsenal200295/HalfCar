function [Zdot] = HalfCarModel_11_AeroM(t,z,hcar)
%HALFCARMODEL4 ITERATION 7 -  Half Car Model to be called by the Numerical Integrator
%   ** NEW - ACCELERATION INPUT CONVERTED TO DRIVING/BRAKING TORQUE
%   ATEMPTING THE TORQUE EQUATIONS OF MOTION FOR UNSPRUNG MASS AGAIN
%   UNSPRUNG DEFLECTION IS COMPUTED BY CONVERTING ANGULAR ACC OF
%   UNSPRUNG MASD TO VERTICAL ACCELERATION OF UNSPRUNG MASS
%   Attempt to solve the Omega<0 during braking problem
%   This iteration contains weight transfer equations modelled using
%   suspension kinematics. Hence, the elastic, geometric and non-suspended
%   weight transfers are modelled. 
%   The moments of Long and Jacking Forces are taken at the CP in this
%   iteration
%   The HalfCar is modelled in this function. 
%   The equations of motion of the car are modelled here in a fashion which
%   can be used by the numerical integrator. 

%% Initializing the states of this time-step
% Independant States
zs = z(1); % Deflection Sprung Mass
phi = z(2); % Pitch Angle Sprung Mass
za1 = z(3); % Deflection Unsprung Mass - 2
za2 = z(4); % Deflection Unsprung Mass - 2
zs_d = z(5); % Velocity Sprung Mass
phi_d = z(6); % Velocity Pitch 
za1_d = z(7); % Velocity Unsprung Mass - 1
za2_d = z(8); % Velocity Unsprung Mass - 2

ang1 = z(9); % Displacement Angular  Wheel Unsprung Mass - 1
omega1 = z(10); % Velocity Angular Wheel Unsprung Mass - 1
sr1 = z(11); % Slip Ratio - 1
ang2 = z(12); % Displacement Angular Wheel Unsprung Mass - 2
omega2 = z(13); % Velocity Angular Wheel Unsprung Mass - 2
sr2 = z(14); % Slip Ratio - 2

if omega1 < 0
    omega1 = 0;
end
if omega2 < 0
    omega2 = 0;
end

X = z(15);
Vx = z(16);

init = evalin('base','r.init');

%% Initializing Axle Suspension Parameters

% IC Arm - Hyp of ICH and VSAL
hyp1 = hcar.axleSusp.hyp_1;
hyp2 = hcar.axleSusp.hyp_2;

hypCP1 = hcar.axleSusp.hypCP_1;
hypCP2 = hcar.axleSusp.hypCP_2;

% Static - IC Arm Angle
ths1 = hcar.axleSusp.th_1;
ths2 = hcar.axleSusp.th_2;

% Variation - IC Arm Angle
tha1 = -(za1 - init(3))/hyp1;
tha2 = (za2 - init(4))/hyp2;

% Total - IC Arm Angle
th1 = ths1 + 0*tha1;
th2 = ths2 + 0*tha2;

%% Initializing Mass and Inertia Properties
% Mass
ms = hcar.mass_sm;
ma1 = hcar.mass_usm_1;
ma2 = hcar.mass_usm_2;

% Inertia
Is = hcar.Inertia;
 
% Inertial of NSMs about Pitch Centers
Iapc1 = hcar.Inertia_NSM_1 + hcar.mass_usm_1*hyp1^2;
Iapc2 = hcar.Inertia_NSM_2 + hcar.mass_usm_2*hyp2^2;

%% Extracting the Input of the current time-step - DIRECT INPUTs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------- DIRECT INPUTS --------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------------------- LONG ACC ------------------------ %
ax_In = interp1(hcar.simInputs3.time,hcar.simInputs3.acc,t,'linear','extrap')*1;

% --------------------- ROAD INPUT ------------------------ %
% Computing delay of the input signal to reach rear axle
delay = hcar.w2w/Vx;
zr1 = interp1(hcar.simInputs3.time,hcar.simInputs3.zr1,t,'linear','extrap')*1;
zr2 = interp1(hcar.simInputs3.time,hcar.simInputs3.zr2,t-delay,'linear','extrap')*1;

zr1_d = interp1(hcar.simInputs3.time,hcar.simInputs3.zr1_d,t+delay,'linear','extrap')*1;
zr2_d = interp1(hcar.simInputs3.time,hcar.simInputs3.zr2_d,t,'linear','extrap')*1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------- COMPUTED INPUTS --------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------------------- DRIVING FORCE ------------------------ %
Fx_In = (ms + ma1 + ma2)*ax_In;
Fx_In_1 = Fx_In * hcar.simInputs3.forceDis;
Fx_In_2 = Fx_In - Fx_In_1;


% --------------------- DRIVING TORQUE ------------------------ %
Md_1 = Fx_In_1*(hcar.tire1.unloadedRadius - (zr1 - za1))*hcar.simInputs3.GainMD_1;
Md_2 = Fx_In_2*(hcar.tire2.unloadedRadius - (zr2 - za2))*hcar.simInputs3.GainMD_2;

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

%% COMPUTING THE INDIRECT INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------- COMPUTED INPUTS --------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -------------------- RADII --------------------- %
% Loaded Radii
rL1 = hcar.tire1.unloadedRadius - defT1;
rL2 = hcar.tire2.unloadedRadius - defT2;

% Effective Radii
rE1 = hcar.tire1.unloadedRadius - -init(3);
rE2 = hcar.tire2.unloadedRadius - -init(4);
% -------------------- VELOCITIES --------------------- %
% Longitudinal Velocity
vx1 = Vx;
vx2 = Vx;

% Slip Velocity
vsx1 = vx1 - rE1*omega1;
vsx2 = vx2 - rE2*omega2;
% -------------------- SLIP RATIO --------------------- %
sr1_d = (-vsx1 - vx1*sr1)/hcar.tire1.relX;
sr2_d = (-vsx2 - vx2*sr2)/hcar.tire2.relX;
% sr1T = -(vsx1)/Vx;
% sr2T = -(vsx2)/Vx;
% -------------------- FORCE | ON CONTACT PATCH --------------------- %
[fxy_1,~,~] = MF5ss_eval(fkt1 + fdt1,sr1,0,0,hcar.tire1.mf);
[fxy_2,~,~] = MF5ss_eval(fkt2 + fdt2,sr2,0,0,hcar.tire2.mf);

% --------------------- RIDE HEIGHTS ------------------------ %
del_zs = (zs - init(1));
del_phi = (phi - init(2));
del_za1 = (za1 - init(3));
del_za2 = (za2 - init(4));

frh = hcar.rh_1 - (del_za1 - del_zs + hcar.a_RH*del_phi);
rrh = hcar.rh_2 - (del_za2 - del_zs - hcar.b_RH*del_phi);

% --------------------- AERO PROPERTIES------------------------ %
if t  > 9.300015
    a = 1;
end
[cDown,cDrag,dist] = hcar.aero.Get_Aero(frh,rrh);

% --------------------- AERO DOWN FORCE------------------------ %
Faero_down = -0.5*hcar.aero.airDensity*hcar.aero.fArea*cDown*Vx^2;
Faero_down_1 = (dist/100)*Faero_down;
Faero_down_2 = (1 - dist/100)*Faero_down;

% --------------------- AERO DRAG FORCE------------------------ %
Faero_drag = 0.5*hcar.aero.airDensity*hcar.aero.fArea*cDrag*Vx^2;

% --------------------- ACCELERATION ------------------------ %
acc = (fxy_1 + fxy_2 - Faero_drag)/(ms + ma1 + ma2);

% -------------------- RESISTANCE/OVERTURNING MOMENT INPUT | ON CONTACT PATCH --------------------- %
fr1 = 0.02;
myx_1 = fr1*rL1*kt1*defT1*sign(omega1);
fr2 = 0.02;
myx_2 = fr2*rL2*kt2*defT2*sign(omega2);


%% FINAL TORQUE INPUT

md_1 = (Md_1 + myx_1); 
md_2 = (Md_2 + myx_2); 

%% Initializing the Weight Transfer Components due to Acceleration Input

% Gravitational Forces
fgs = ms*9.81;
fga1 = ma1*9.81;
fga2 = ma2*9.81;

% Net Torque at Wheel
torqueWheel1 = -(fxy_1*rL1) - myx_1 + md_1; 
torqueWheel2 = -(fxy_2*rL2) - myx_2 + md_2;
    
% Non-Supended Weight Transfer
fnsm = 0*(((ma1 + ma2))*acc*hcar.cg_USM_Height)/hcar.w2w;

% Net Vertical Load on Unsprung Mass || Used in Moment Equation to find instant center angle change
fnetv1 = fkt1 + fdt1 + fnsm - fga1 - fs1 - fd1;
fnetv2 = fkt2 + fdt2 - fnsm - fga2 - fs2 - fd2; 

%% Test
if t  > 9.300015
    a = 1;
end
if md_1 ~=0
    b = 1;
end
if md_1 == 0
    c = 1;
end

%% --------- Modelling the Half Car - Equations of Motion ----------------

%% --- Dynamic Wheel Equations of Motion --- % 
if omega1 <=0 && md_1 ~= 0 
    omega1_d = 0;
else
    omega1_d = ((torqueWheel1)) / (hcar.Inertia_NSM_1);
end

if omega2 <=0 && md_2 ~=0
    omega2_d = 0;
else
    omega2_d = ((torqueWheel2)) / (hcar.Inertia_NSM_2);
end

%% --- Longitudinal Equations of Motion

fx1 = fxy_1 - hcar.mass_usm_1*acc;
fx2 = fxy_2 - hcar.mass_usm_2*acc;

% Geometric Weight Transfer = Jacking Force
fz1_s = -fx1*tan(th1);% + ((md_1 + myx_1)/hcar.axleSusp.vsal_1);% Reaction component on Sprung Mass
fz2_s = fx2*tan(th2);% - ((md_2 + myx_2)/hcar.axleSusp.vsal_2);% Reaction component on Sprung Mass

% fz1_u = -fz1_s; % Action component of jacking force on Unsprung Mass
% fz2_u =  -fz2_s;% Action component of jacking force on Unsprung Mass

%% --- Vertical Equations of Motion --- %

hypRL1 = (sqrt((hcar.axleSusp.ich_1 - rL1)^2 + (hcar.axleSusp.vsal_1)^2));
hypRL2 = (sqrt((hcar.axleSusp.ich_2 - rL2)^2 + (hcar.axleSusp.vsal_2)^2));

zs_dd = (fs1 + fd1 - fgs + fs2 + fd2 + (fz1_s + fz2_s) + Faero_down_1 + Faero_down_2)/ms;

phi_dd = (-hcar.dis_a12cg*fs1 - hcar.dis_a12cg*fd1...
         + hcar.dis_a22cg*fs2 + hcar.dis_a22cg*fd2...
         - Faero_down_1*hcar.dis_a12cg + Faero_down_2*hcar.dis_a22cg...
         - (fx1*(hcar.cg_SM_Height))...
         - (fx2*(hcar.cg_SM_Height))...
         - (fz1_s*(hcar.dis_a12cg))...
         + (fz2_s*(hcar.dis_a22cg)))/Is;

za1_dd = -hypRL1*(md_1 - myx_1 - hcar.Inertia_NSM_1*omega1_d - (fxy_1)*rL1...
          - fnetv1*hcar.axleSusp.vsal_1...
          - (fxy_1)*(hcar.axleSusp.ich_1 - 0*rL1)...
          - hcar.mass_usm_1*acc*(hcar.axleSusp.ich_1 - rL1))/...
         (Iapc1);

% za1_dd = (-fs1 - fd1 - fga1 + fkt1 + fdt1 + fnsm + fz1_u)/hcar.mass_usm_1 - ...
%           th1_dd*hypRL1;


za2_dd = hypRL2*(md_2 - myx_2 - hcar.Inertia_NSM_2*omega2_d - (fxy_2)*rL2...
          + fnetv2*hcar.axleSusp.vsal_2...
          - (fxy_2)*(hcar.axleSusp.ich_2 - 0*rL2)...
          - hcar.mass_usm_2*acc*(hcar.axleSusp.ich_2 - rL2))/...
         (Iapc2);
      
% za2_dd = (-fs2 - fd2 - fga2 + fkt2 + fdt2 - fnsm + fz2_u)/hcar.mass_usm_2 + ...
%           0*th2_dd*hypRL2;


      
%% Conditiong the equations to reflect the stat space form

Zdot = [zs_d;
        phi_d;
        za1_d;
        za2_d;
        zs_dd;
        phi_dd;
        za1_dd;
        za2_dd;
        omega1;
        omega1_d;
        sr1_d;
        omega2;
        omega2_d
        sr2_d;
        Vx;
        acc];

end

