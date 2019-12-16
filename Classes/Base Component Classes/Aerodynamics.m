classdef Aerodynamics
    %AERODYNAMICS Class representing Aerodynamics
    %   Consists of all the methods and properties to fully define the
    %   aerodynamics of a half car
    
    properties
        % Input Properties
        aeromap double % Full Aero Map of Front RH, Rear RH vs cDownForce, cDrag, aeroDistribution
        cDown double % Aero Map of Front RH, Rear RH and Downforce Coefficient
        cDrag double % Aero Map of Front RH, Rear RH and Drag Coefficient
        cDist double % Aero Map of Front RH, Rear RH and Force Distribution
        airDensity double % Air Density [kg/m^3]
        fArea double % Frontal Area [m^2]
        
        % Computed Properties
        %aeroDownForce double % Aerodynamic downforce interpolated from aeromap
    end
    
    %% GENERAL & UI METTHODS
    methods
        function obj = Aerodynamics()
            %AERODYNAMICS Construct an instance of this class
            %   Creates a default instance of the class
            
            obj.fArea = 1;
            obj.airDensity = 1.225;
            aeroM = readtable('LMP_Aero.xlsx');
            obj.aeromap = aeroM.Variables;
            obj.cDown = obj.aeromap(:,3);
            obj.cDrag = obj.aeromap(:,4);
            obj.cDist = obj.aeromap(:,5);

        end
    end
    
    %% MATHEMATICAL METHODS
    
    methods (Access = public)
        % INTERPOLATE DOWNFORCE COEFFICIENT
        function [cDown,cDrag,cDist] = Get_Aero(obj,frh_m,rrh_m)
            %Get_Aero Returns all extractable aerodynamic properties
            %   Interpolates and returns the Coeff of Downforce, Coeff of
            %   Drag and Aerodynamic Distribution 
            
            % M to mm conversion
            % Done because Aeromap is in mm
            frh = frh_m*1000;
            rrh = rrh_m*1000;
            
            % ---------- SAFETY FEATURE ------------- %
            % Picking the minimum or maximum front ride height if the computed ride height is lower or greater than the min or max respectively                       
            if frh < min(obj.aeromap(:,1))
                frh = min(obj.aeromap(:,1));
            elseif frh > max(obj.aeromap(:,1))
                frh = max(obj.aeromap(:,1));
            end
            % Picking the minimum or maximum rear ride height if the computed ride height is lower or greater than the min or max respectively
            if rrh < min(obj.aeromap(:,2))
                rrh = min(obj.aeromap(:,2));
            elseif rrh > max(obj.aeromap(:,2))
                rrh = max(obj.aeromap(:,2));
            end
            
            cDown = griddata(obj.aeromap(:,1),obj.aeromap(:,2),obj.cDown,frh,rrh,'linear');
            cDrag = griddata(obj.aeromap(:,1),obj.aeromap(:,2),obj.cDrag,frh,rrh,'linear');
            cDist = griddata(obj.aeromap(:,1),obj.aeromap(:,2),obj.cDist,frh,rrh,'linear');
            

        end
    end
end

