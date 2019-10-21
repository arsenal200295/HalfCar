classdef TireCus
    %TIRE Class representing Tire
    %   Contains all the properties and definitions required to fully
    %   define the vertical, lateral and longitudinal characteristics of a
    %   tire
    
    properties
        name string
        vertStiff ElasticComp
        kt double
        vertDamp DampingComp
        dt double
        unloadedRadius double
        mf
        myuX double
        myuY double
        relX double
        relY double;
    end
    
    %% General and UI methods
    methods
        % DEFAULT CONSTRUCTOR
        function obj = TireCus()
            %TIRE Construct an instance of this class
            %   Creates default instance of class
            
            obj.name = "Default Tire";
            obj.vertStiff = ElasticComp;
            obj.vertStiff.name = "Tire Rate";
            obj.kt = 2e5;
            obj.vertDamp = DampingComp;
            obj.dt = 8;
            obj.vertDamp.name = "Tire Damping";
            obj.unloadedRadius = 0.225;
            obj.mf = obj.Init_MF_Coeff();
            obj.myuY = 1.2;
            obj.myuX = 0.8;
            obj.relX = 0.5;
            obj.relY = 0.5;
        
        end
        
        % INITIALIZE MAGIC FORMULA COEFFICIENTS
        function [mfCoeff] = Init_MF_Coeff(~)
            % Tyre model Magic Formula coefficients
            %
            % Note 1: these coefficients use the Pacejka sign convention (See book: Tire and Vehicle Dynamics)
            % They are not compatible with the ISO sign convention used by TNO or ADAMS tyre models!
            % see tyre sign convention, Vehicle Dynamics lecture notes (4L150), page 118.
            %
            % Note 2: simplified, symmetric Magic Formula expressions are used, so a  number of coefficients 
            % have been eliminated (e.g. coefficients controlling static offsets: pHx1, pHx2, pHy1, pHy2, pVy1, pVy2, etc.)

            MF.fittyp       = 5;         % Magic Formula version [-]
            MF.R0           = 0.3160;    % unloaded tyre radius used in Magic Formula to make coefficients dimensionless [m]
            MF.Fz0          = 5782;      % reference force value (typically corresponding to the static vertical force), 
                                                % used in Magic Formula to make coefficients dimensionless [N]

            % limitations on the inputs of the Magic Formula (to prevent extrapolation errors)
            MF.limits.Fz    = [200 11564];
            MF.limits.kappa = [-1.5000 1.5000];
            MF.limits.alpha = [-1.5708 1.5708];
            MF.limits.gamma = [-0.2610 0.2610];

            % dimensionless scaling coefficients to modify global tyre behaviour
            MF.scaling.lmux = 1;   % longitudinal friction scaling
            MF.scaling.lKx  = 1;   % longitudinal slip stiffness scaling
            MF.scaling.lmuy = 1;   % lateral friction scaling
            MF.scaling.lKy  = 1;   % cornering stiffness scaling
            MF.scaling.lgay = 1;   % scaling of camber influence on lateral force
            MF.scaling.ltr  = 1;   % scaling of pneumatic trail
            MF.scaling.lres = 1;   % scaling of residual moment 
            MF.scaling.lgaz = 1;   % scaling of camber influence on self aligning moment
            MF.scaling.ls   = 1;   % scaling of moment arm Fx (combined slip)

            % longitudinal force coefficients
            MF.long.pCx1 =  1.6322;
            MF.long.pDx1 =  1.2750;
            MF.long.pDx2 = -0.1237;
            MF.long.pEx1 = -0.1048;
            MF.long.pEx2 =  0.7129;
            MF.long.pEx3 =  0.3907;
            MF.long.pKx1 = 23.7550;
            MF.long.pKx2 =  2.1950;
            MF.long.pKx3 =  0.3222;
            MF.long.rBx1 = 19.2780;
            MF.long.rBx2 =-14.0190;
            MF.long.rCx1 =  0.9819;

            % lateral force coefficients
            MF.lat.pCy1 =  1.2872;
            MF.lat.pDy1 =  1.0488;
            MF.lat.pDy2 = -0.2300;
            MF.lat.pDy3 =  0.8878;
            MF.lat.pEy1 = -0.8996;
            MF.lat.pEy2 = -0.5536;
            MF.lat.pEy4 = -5.2807;
            MF.lat.pKy1 = 16.8590;
            MF.lat.pKy2 =  1.9348;
            MF.lat.pKy3 =  0.1695;
            MF.lat.pHy3 =  0.0041;
            MF.lat.pVy3 =  0.5365;
            MF.lat.pVy4 =  0.4555;
            MF.lat.rBy1 =  6.9875;
            MF.lat.rBy2 =  7.2000;
            MF.lat.rCy1 =  1.0774;

            % self aligning moment coefficients
            MF.align.qBz1  =  8.6458;
            MF.align.qBz2  = -1.0905;
            MF.align.qBz3  = -2.8235;
            MF.align.qBz5  =  0.4723;
            MF.align.qBz9  =  9.4811;
            MF.align.qBz10 =  0;
            MF.align.qCz1  =  1.1479;
            MF.align.qDz1  =  0.1232;
            MF.align.qDz2  = -0.0086;
            MF.align.qDz4  = -0.2289;
            MF.align.qDz8  =  0.2597;
            MF.align.qDz9  =  0.0279;
            MF.align.qEz1  = -3.2802;
            MF.align.qEz2  = -0.7523;
            MF.align.qEz3  =  0;
            MF.align.qEz5  = -3.9464;
            MF.align.qHz3  =  0.0329;
            MF.align.qHz4  =  0.0465;
            MF.align.ssz2  =  0.0458;
            MF.align.ssz3  = -0.9372;
            MF.align.ssz4  =  0.5040;
            
            
            mfCoeff = MF;
        end
        
    end
end

