classdef Damper < DampingComp
    %DAMPER Derived DampingComp Class - Damper
    %   Defines a damper
    
    properties
        staticGasForce double % Static gas force exerted by damper at 0 m/s shaft velocity [N]
    end
    
    methods
        function obj = Damper()
            %DAMPER Construct an instance of this class
            %   Creates default instance of class
            
            obj.name = "Default Damper";
            obj.freeLength = 0.35;
            obj.stroke = 0.15;
            obj.staticGasForce = 0;
%             obj.forceVelData = [-2,-3480+obj.staticGasForce;...
%                 -1,-1740+obj.staticGasForce;...
%                 0,0+obj.staticGasForce;...
%                 1,740+obj.staticGasForce;...
%                 2,1480+obj.staticGasForce];
             
            table = readtable('LMP_Damper.xlsx');
            obj.forceVelData = [table.Variables];

        end
        

    end
end

