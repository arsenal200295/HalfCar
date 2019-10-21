classdef Bumpstop < ElasticComp
    %BUMPSTOP Derived Class - Bumpstop
    %   Defines a Bumpstop
    
    properties
        
    end
    
    methods
        function obj = Bumpstop()
            %BUMPSTOP Construct an instance of this class
            %   Creates default instance of class
            obj.name = "Default Bumpstop";
            obj.freeLength = 0.015;
            obj.stroke = 0.0145;
            
            table = readtable('LMP_Bumpstop.xlsx');
            obj.forceDefData = [table.Variables];
        end
        
    end
end

