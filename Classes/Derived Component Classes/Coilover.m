classdef Coilover < ElasticComp
    %COILOVER Derived ElasticComp Class - Coilover
    %   Defines a Coilover
    
    properties
        % Input Properties
        springGap double % Spring Preload/Gap [m]
        bsGap double % Bumpstop Gap [m]
        spring Spring % Spring of Coilover
        bumpStop Bumpstop % Bumpstop of Coilover
    end
    
    % GENERAL AND UI METHODS
    methods (Access = public)
        function obj = Coilover()
            %COILOVER Construct an instance of this class
            %   Creates default instance of class
            obj.name = "Default Coilover";
            obj.freeLength = 0.449;
            obj.stroke = 0.048;
            obj.bsGap = 0.038;
            obj.springGap = 0.016;
            
            % Initializing Components
            obj.spring = Spring;
            obj.bumpStop = Bumpstop;
            
            
            % Initialze Combined Spring + Bumpstop Stiffness here
            obj = obj.Combine_ForceDefData();
        end
    end
    
    % ACCESSIBILITY METHODS
    methods (Access = public)
        % COMBINE FORCE vs DEFLECTION DATA
        function [coilOverReturn] = Combine_ForceDefData(obj)
            %Combine_ForceDefData Combines  data of spring and bumpstop
            %   This method combines and returns the force vs deflection
            %   data of the spring and bumpstop into on dataset. 
                        
            % Conditioning the Spring Force vs Deflection Curve
            obj.spring.forceDefData = obj.Condition_ForceDefCurve(obj.spring.forceDefData);
                                   
            % Conditioning the Bumpstop Force vs Deflection Curve
            obj.bumpStop.forceDefData = obj.Condition_ForceDefCurve(obj.bumpStop.forceDefData);
                                    
            % Bringing the Spring Lookup Table to the Coilover lookup
            % table's reference
            obj.spring.forceDefData(:,1) = obj.spring.forceDefData(:,1) + obj.springGap;
            
            % Bringing the Bumpstop Lookup Table to the Coilover lookup
            % table's reference
            obj.bumpStop.forceDefData(:,1) = obj.bumpStop.forceDefData(:,1) + obj.bsGap;
            
            % Creating the Coilover lookup table
            steps = linspace(0,obj.stroke,50);
            obj.forceDefData = zeros(length(steps),2);
            for i = 1:length(steps)
                obj.forceDefData(i,1) = steps(i);
                springForce = interp1(obj.spring.forceDefData(:,1),obj.spring.forceDefData(:,2),steps(i),'linear','extrap');
                if isnan(springForce)
                    springForce = 0;
                end
                bsForce = interp1(obj.bumpStop.forceDefData(:,1),obj.bumpStop.forceDefData(:,2),steps(i),'linear','extrap');
                if isnan(bsForce)
                    bsForce = 0;
                end
                obj.forceDefData(i,2) = springForce + bsForce;
            end

            obj.forceDefData = obj.Condition_ForceDefCurve(obj.forceDefData);
            coilOverReturn = obj;
        end
        
        % FORCE CURVE CONDITIONING - MAKING INFINITELY STIFF AFTER SOLID
        % LENGTH
        function [forceDefData_Condition] = Condition_ForceDefCurve(obj,forceDefData)
            %Condition_ForceDefCurve Conditions the Force vs Deflection
            %Data Curve to include infinite force return in case solid
            %length is reached
            
            solidX = [forceDefData(end,1)+0.01;...
                      forceDefData(end,1)+1];
            
            solidY = [forceDefData(end,2)+obj.infForce;
                      forceDefData(end,2)+obj.infForce];
            
            forceDefData_Condition = [[-1 0];...
                                      forceDefData;
                                      [solidX,solidY]];  
        end
        
    end
    
end

