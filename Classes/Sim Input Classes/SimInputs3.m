classdef SimInputs3
    %SIMINPUTS Class representing Simulation Inputs - FOR A REAL SIMULATION
    %   ** NEW - ACCELERATION INPUT IS CONVERTED TO DRIVING TORQUE
    %   In this version of simulation inputs, the inputs are the
    %   driving/braking torques
    %   Contains all the properties and methods required to define the
    %   different Simulation Input Options
    
    %% PROPERTIES
    properties
        % Input Properties - Signal Properties
        tStart double % Start Time of Input Signal [s]
        tEnd double % End Time of Input Signal[s]
        ampZr double % Base amplitude of road input signal [m]
        ampAcc double % Base amplitude of acceleration input signal [g's of acceleration]
        fs double % Samping Frequencyy [Hz]
        
        % Input Properties
        Md_1 double % Driving or Braking Torque Front [Nm]
        Md_2 double % Driving or Braking Torque Rear [Nm]
        
        
        % Computed Properties
        forceDis double % Force Distribution between Front|Rear or Left|Right
        acc double % Longitudinal or Lateral Acceleration which the Car experiences [m/s^2]
        vx double % Longitudinal Velocity of the Vehicle [m/s]
        M_1 double % Moment Input (Mx or My) of mass 1 [Nm]
        M_2 double % Moment Input (Mx or My) of mass 2 [Nm]
        GainMD_1 double
        GainMD_2 double 
        
        
        time double % Time Vector
        zr1 double % Bump Profile which axle 1 experiences - Front/Left [m]
        zr2 double % Bump Profile which axle 2 experiences - Rear/Right [m]
        zr1_d double % Bump Velocity which axle 1 experiences - Front/Left [m/s]
        zr2_d double % Bump Velocity which axle 1 experiences - Rear/Right [m/s]
        Fxy_1 double % Fx or Fy input on the Front or Left Tire [N]
        Fxy_2 double % Fx or Fy input on the Rear or Right Tire [N]
    end
    
    %% GENERAL AND UI METHODS
    methods
        % DEFAULT CONSTRUCTOR
        function obj = SimInputs3()
            %SIMINPUTS Construct an instance of this class
            %   Creates a default instance of this class
            obj.tStart = 0;
            obj.tEnd = 30;
            obj.fs = 1000;
            obj.ampZr = 0.0;
            obj.ampAcc = 14;
            obj.vx = 1;
            obj.acc = 0;
            
            % Creating a time signal
            obj.time = [obj.tStart:(1/obj.fs):obj.tEnd]';
            
            % Creating a default road input signal
            def_Zin = obj.Create_RectPulse(obj.time,obj.ampZr,10,20);
%             def_Zin = obj.ampZr*chirp(obj.time,0,30,10,'linear',90);
%             [~,~,def_Zin] = gauspuls(obj.time-15,0.2,0.6);
            
            % Computed Properties
            obj.zr1 = def_Zin*obj.ampZr;
            obj.zr2 = def_Zin*obj.ampZr;
            [zr1_d,zr2_d] = obj.Compute_RoadInputVel();
            obj.zr1_d = [0;zr1_d];
            obj.zr2_d = [0;zr2_d];
            
            % Creating default acceleration signal
            [~,~,obj.acc] = gauspuls(obj.time-15,1,0.15);
            obj.acc = obj.ampAcc.*obj.acc; obj.acc(1,1) = 0;
            obj.forceDis = 0;
            obj.GainMD_1 = 1.25;
            obj.GainMD_2 = 1.25;
            
        end 
    end
    
    %% MATHEMATICAL METHODS
    methods (Access = public) 
        % COMPUTE VELOCITY OF ROAD INPUT - d_zr1 and d_zr2
        function [zr1_d,zr2_d] = Compute_RoadInputVel(obj)
            zr1_d = diff(obj.zr1)./diff(obj.time);
            zr2_d = diff(obj.zr2)./diff(obj.time);
        end
        
        % COMPUTE CONTACT PATCH INPUT FORCES
        function [fxy_1,fxy_2] = Compute_ContactPatchForces(obj,hcar)
            %Compute_ContactPatchForces Computes and returns the contact
            %patch LATERAL or LONGITUDINAL forces based on the acceleration
            %input
            m = hcar.mass_sm + hcar.mass_usm_1 + hcar.mass_usm_2;
            ftot = m.*obj.acc*9.81;
            fxy_1 = (obj.forceDis).*ftot;
            fxy_2 = (1-(obj.forceDis)).*ftot;
        end
    end
    
    %% INPUT CREATTION METHODS
    methods (Access = public)
        % CREATE STEP INPUT
        function [stepInput] = Create_Step(~,time,stepTime,stepSize)
            %Create_Step Creates and returns a step input
            st = time>= stepTime;
            stepInput = st.*stepSize;
        end
        
        % CREATE RAMP INPUT
        function [rampInput] = Create_Ramp(~,time,stepTime,stepSize)
            %Create_Step Creates and returns a step input
            st = time>= stepTime;
            step = st.*stepSize;
            rampInput = time.*step;
            
        end
        
        
        % CREATE RECTANGULAR PULSE INPUT
        function [rectInput] = Create_RectPulse(~,time,amp,tStart,width)
            %Create_RectPulse Creates and returns a rectangular pulse
            
            rectInput = amp*rectpuls(time-(tStart+width/2),width);
            
        end
    end
end

