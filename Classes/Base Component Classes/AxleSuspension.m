classdef AxleSuspension
    %AxleSuspension Class representing all Suspensions
    %   Contains all the properties and methods required to define an
    %   abstract Suspension
    
    properties
%         rotCenter double % Rotation Center of Sprung Mass - Pitch/Roll Center [m]
%                          % This property consists of 2 values
%                          % -> Lateral distance of RotCenter from CG
%                          % -> Height of RotCenter

        vsal_1 double % Virtual Swign Arm Length of Axle 1 [m] - Front/Left
                      % +VE || Rearward from Front CP | Rightward from Left CP
        
        vsal_2 double % Virtual Swign Arm Length of Axle 2 [m] - Rear/Right
                      % +VE || Forward from Rear CP | Leftward from Right CP
        
        ich_1 double % IC Height of Axle 1 [m] - Front/Left
                     % +VE || Upward from Ground
        
        ich_2 double % IC Height of Axle 1 [m] - Rear/Right
                     % +VE || Upward from Ground
        
        % Computed Properties
        th_1 double % IC angle with ground which described anti-effect of Front/Left [rad]
        
        th_2 double % IC angle with ground which described anti-effect of Rear/Right [rad]
        
        thCP_1 double
        
        thCP_2 double
        
        rotCenterHeight double % Rotation Center Height
        
        
        % Computed Properties
        hyp_1 double % Hyp completing the triangle with VSAL_1 and ICH_1 as sides [m]
        hyp_2 double % Hyp completing the triangle with VSAL_2 and ICH_2 as sides [m]
        
        hypCP_1 double
        hypCP_2 double
    end
    
    %% GENERAL AND UI METHODS
    methods (Access = public)
        function obj = AxleSuspension()
            %SUSPENSION Construct an instance of this class
            %   Creates a default instance of the class
            obj.rotCenterHeight = 0.1;
            obj.vsal_1 = 3.6; 
            obj.ich_1 = 0.35 + 0;
            obj.vsal_2 = 3.6;
            obj.ich_2 = 0.35 + 0;
                  
        end
    end
    
    %% MATHEMATICAL MEHODS
    methods (Access = public)
        % COMPUTE HYP
        
    end
    
end

