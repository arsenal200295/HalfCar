classdef Spring < ElasticComp
    %SPRING Derived Class of Spring
    %   Defines a Spring 
    
    properties
        
    end
    
    methods
        function obj = Spring()
            %SPRING Construct an instance of this class
            %   Creates default instance of class
            obj.name = "Default Spring";
            obj.freeLength = 0.2;
            obj.stroke = 0.1;
            obj.forceDefData = [...
                0,0;...
                0.020,6780;...
                0.040,13560;...
                0.060,20340;...
                0.080,27120;...
                0.100,33900];
%            obj.forceDefData(:,2) = obj.forceDefData(:,2) .*5;
            
%             obj.stiffness = obj.GetStiffness_FromDef(0.02,obj.stroke);
        end
        
        
    end
end

