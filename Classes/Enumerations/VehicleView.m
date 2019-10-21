classdef VehicleView
    %VEHICLEVIEW Enumeration Class which helps define which view of vehicle
    %is to be considered
    %   The halfcar model is created in a generic fashion to enable
    %   simulation in 2 views
    %   -> SIDE VIEW (Acceleration and Braking along with road input)
    %   -> FRONT VIEW (Lateral Accelerations along with road input)
    
    enumeration
        SideView,FrontView
    end
end

