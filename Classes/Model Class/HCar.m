classdef HCar
    %HCAR Summary of this class goes here
    %   Detailed explanation goes here
    
    %% PROPERTIES
    properties
        % Input Properties - Parameters
        name string % Name of the HalfCar 
        mass_sm double % Sprung Mass [Kg]
        mass_sm_dis double % Sprung Mass Distribution [%]
        Inertia double % Inertia of the Sprung Mass about its CG [Kgm^2]
        Inertia_NSM_1 double % Inertia of Unsprung Mass 1 about its own CG [Kgm^2]
        Inertia_NSM_2 double % Inertia of the Unsprung Mass 2 about its own CG [Kgm^2]
        mass_usm_1 double % Unsprung Mass of Axle 1 - Front or Left [Kg]
        mass_usm_2 double % Unsprung Mass of Axle 2 - Rear or Right [Kg]
        w2w double % Wheel-To-Wheel distance - Wheelbase or Trackwidth [m]
        cg_SM_Height double % Suspended Mass CG Height [m]
        cg_USM_Height double % Non-Suspended Mass CG Height [m]
        cg_USM_Height2 double % Non-Suspended Mass 2 CG Height [m]
        rh_1 double % Ride Height of the Front/Left Axle [m]
        rh_2 double % Ride height of the Rear/Right Axle [m]
        a_RH_X
        b_RH_X
        
        % Computed Properties - Parameters
        mass_sm_1 double % Sprung Mass on Front/Left Axle [Kg]
        mass_sm_2 double % Sprung Mass on Rear/Right Axle [Kg]
        dis_a12cg double % Distance of Axle 1 to CG - a [m]
        dis_a22cg double % Distance of Axle 2 to CG - b [m]
        a_RH double % Distance of front ride height reference point from CG [m]
        b_RH double % Distance of rear ride height reference point from CG [m]
        
        % Input Properties - Components
        coil1 Coilover
        coil2 Coilover
        damper1 Damper
        damper2 Damper
        tire1 TireCus
        tire2 TireCus
        axleSusp AxleSuspension
        aero Aerodynamics
        
        % Input Properties - Simulation Inputs
%         simInputs SimInputs % In this class the input is Long Acceleration which is converted to front and rear FORCE and imposed on the wheels (along with burmp profile input)
%         simInputs2 SimInputs2 % In this class the input is driving/braking torque front and rear (along with bump profile input)
        simInputs3 SimInputs3 % In this class the input is Long Acceleration which is converted to front and rear torque (along with bump profile input)
    end
    
    %% GENERAL AND UI METHODS
    methods (Access = public)
        function obj = HCar()
            %HCAR Construct an instance of this class
            %   Creates a default instance of the class
            
            % Input Properties - Paramters
            obj.name = "Default HalfCar";
            obj.mass_sm = 400;
            obj.mass_sm_dis = 50;
            obj.Inertia = 600;
            obj.Inertia_NSM_1 = 2.4; % 
            obj.Inertia_NSM_2 = 3.2;
            obj.mass_usm_1 = 42;
            obj.mass_usm_2 = 42;
            obj.w2w = 3;
            obj.cg_SM_Height = 0.336;
            obj.cg_USM_Height = 0.225;
            obj.cg_USM_Height2 = 0.310;
            obj.rh_1 = 0.067;
            obj.rh_2 = 0.081;
            obj.a_RH_X = 0.75; 
            obj.b_RH_X = -3.25; 
            
            % Input Properties - Components
            obj.coil1 = Coilover;
            obj.coil2 = Coilover;
            obj.damper1 = Damper;
            obj.damper2 = Damper;
            obj.tire1 = TireCus;
            obj.tire2 = TireCus;
            obj.axleSusp = AxleSuspension;
            obj.aero = Aerodynamics;
            
            % Input Properties - Simulation Inputs
            %obj.simInputs = SimInputs(obj);
            %obj.simInputs2 = SimInputs2();
            obj.simInputs3 = SimInputs3();
            
            % Computed Properties - Parameters
            [obj.mass_sm_1,obj.mass_sm_2] = obj.Compute_AxleMass();
            [obj.dis_a12cg,obj.dis_a22cg] = obj.Compute_DisFromCG();
            [obj.b_RH_X,obj.b_RH_X] = obj.Compute_RHPoint_DisFromCG();% a + Ax --> See Ipad Notes % -(a + Bx) --> See Ipad Notes
            obj = obj.Compute_Hyp();
            
        end
        
        
    end
    
    %% MATHEMATICAL METHODS
    methods(Access = public)
        % COMPUTE AXLE MASS
        function [m1,m2] = Compute_AxleMass(obj)
            %Compute_AxleMass Computes the masses on the axles from the
            %vehicle mass and mass distribution
            m1 = obj.mass_sm*(obj.mass_sm_dis)/100;
            m2 = obj.mass_sm*(100-obj.mass_sm_dis)/100;
        end
        
        % COMPUTE CG
        function [dis1,dis2] = Compute_DisFromCG(obj)
            %Compute_CG Computes the CG position and returns the a and b
            %lengths
            
            dis1 = (1-obj.mass_sm_dis/100)*obj.w2w;
            dis2 = (obj.mass_sm_dis/100)*obj.w2w; 
        end
        
        % COMPUTE RIDE HEIGHT MEASUREMENT POINT DISTANCE FROM CG
        function [a_rh,b_rh] = Compute_RHPoint_DisFromCG(obj)
            %Compute_RHPoint_DisFromCG Computes the distance from the CG of
            %the front and rear RIDE HEIGHT MEASUREMENT POINTS using the
            %coordinate inputs from the user
            
            a_rh = obj.dis_a12cg + obj.a_RH_X;
            b_rh = -(obj.dis_a12cg + obj.b_RH_X);
        end
        
        % COMPUTE DELAYED ROAD INPUT - FOR SV HALFCAR'S REAR AXLE
        function [hcarReturn] = Delay_RoadInput(obj)
            %Delay_RoadInput Delays the road input by an amount equal to
            %the wheelbase of the halfcar. 
            %   This is needed incase :
            %   -> User is performning SIDE VIEW simulation
            %   -> User is performing a distance based input. That is, NOT
            %      A POST RIG SIMULATION WHERE Vx = 0
            
            delay = obj.w2w/obj.simInputs.vx;
            
            
            hcarReturn = obj;
            
        end
        
        % COMPUTE HYP BETWEEN IC & NSM
        function [hcarReturn] = Compute_Hyp(obj)
            %Compute_Hyp Computes the hyp 
            %   Computes the hyp between the Unsprung Mass CG and the IC
            %   height
            
            % Instant Center
            ich_1 = obj.axleSusp.ich_1;
            ich_2 = obj.axleSusp.ich_2;

            % Virtual Swing Arm Length
            vsal_1 = obj.axleSusp.vsal_1;
            vsal_2 = obj.axleSusp.vsal_2;

            % Non-Suspended Mass CG Heights
            nsmh_1 = obj.cg_USM_Height;
            nsmh_2 = obj.cg_USM_Height;

            % Instant Center Heights
            y1 = ich_1 - nsmh_1;
            y2 = ich_2 - nsmh_2;
            
             % Computed Properties
%             obj.th_1 = atan(obj.ich_1/obj.vsal_1);
%             obj.th_2 = atan(obj.ich_2/obj.vsal_2);    
            obj.axleSusp.th_1 = atan(ich_1/vsal_1);
            obj.axleSusp.th_2 = atan(ich_2/vsal_2);

            % IC Arm - Hyp of ICH and VSAL
            obj.axleSusp.hypCP_1 = sqrt((ich_1)^2 + vsal_1^2);
            obj.axleSusp.hypCP_2 = sqrt((ich_2)^2 + vsal_2^2);
            
            obj.axleSusp.hyp_1 = sqrt((y1)^2 + vsal_1^2);
            obj.axleSusp.hyp_2 = sqrt((y2)^2 + vsal_2^2);
           
            hcarReturn = obj;
        end
    end

    %% SIDE VIEW SPECIFIC METHODS
    methods (Access = public)
        % MODIFY SIMULATION INPUTS FOR SIDE VIEW
        function [hcarReturn] = Delay_SimInput_SV(obj)
            %Delay_SimInput_SV Delays the input on the rear tire by an
            %amount equal to the wheel base 
            
            
            hcarReturn = obj;
        end
    end
    
end

