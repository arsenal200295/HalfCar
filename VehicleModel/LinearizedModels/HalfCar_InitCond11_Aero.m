function [F] = HalfCar_InitCond7(z,hcar)
%HALFCAR_INITCOND Computes and the initial deflections of the Half Car
%Model
%   This function contains the equations of motion linearized about the
%   initial equilibrium point of (0,0,0,0,0,0,0,0).
%   This function is run by FSOLVE and hence is based on an optimization
%   algorithm
%   Hence, along with the deflections, the solution will also contain the
%   stiffness of the springs and tires which correspond to static
%   conditions

%% INITIALIZING THE HALFCAR PARAMETERS
ms = hcar.mass_sm;
ma1 = hcar.mass_usm_1;
ma2 = hcar.mass_usm_2;

d1 = hcar.dis_a12cg;
d2 = hcar.dis_a22cg;

g = 9.81;
fs = ms*g;
fa1 = ma1*g;
fa2 = ma2*g;

%% INITIALIZING THE VARIABLES WHICH FSOLVE IS OPTIMIZING 
zs = z(1);
phi = z(2);
za1 = z(3);
za2 = z(4);

%% Initalizing the Deflections of the current time-step
% Suspension Deflection
defSus1 = za1 - zs + d1*phi;
defSus2 = za2 - zs - d2*phi;

% Tire Deflection 
defT1 = - za1;
defT2 = - za2;

%% INTIALIZING COMPONENT PARAMETERS

fs1 = hcar.coil1.Get_Force(defSus1,hcar.coil1.stroke); %25000;
fs2 = hcar.coil2.Get_Force(defSus2,hcar.coil2.stroke); %18000;
kt1 = 2e5;
kt2 = 2e5;

%% COMPUTING AERODYNAMIC DOWNFORCE
% --------------------- RIDE HEIGHTS ------------------------ %
frh = hcar.rh_1;
rrh = hcar.rh_2;

% --------------------- AERO PROPERTIES------------------------ %
[cDown,cDrag,dist] = hcar.aero.Get_Aero(frh,rrh);

% --------------------- AERO DOWN FORCE------------------------ %
Faero_down = -0.5*hcar.aero.airDensity*hcar.aero.fArea*cDown*hcar.simInputs3.vx^2;
Faero_down_1 = (dist/100)*Faero_down;
Faero_down_2 = (1 - dist/100)*Faero_down;

% --------------------- AERO DRAG FORCE------------------------ %
Faero_drag = 0.5*hcar.aero.airDensity*hcar.aero.fArea*cDrag*hcar.simInputs3.vx^2;

%% lINEARIZED EQUATIONS OF MOTION
F(1) = -fs + fs1 + fs2 + Faero_down;
F(2) = -d1*fs1 + d2*fs2...
       - Faero_down_1*hcar.dis_a12cg + Faero_down_2*hcar.dis_a22cg;
F(3) = -fa1 - fs1 + kt1*(defT1);
F(4) = -fa2 - fs2 + kt2*(defT2);


end

