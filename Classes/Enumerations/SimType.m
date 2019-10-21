classdef SimType
    %SIMTYPE Enumeration to define the Simulation Type
    %   The different between the 2 types of simulation is:
    %   -> Post Rig :- No Vx consideration and hence no delay of front and
    %       rear input
    %   -> Track :- Vx is considered and hence there is delay between the
    %       front and rear input
    
    enumeration
        PostRigSimulation, TrackSimulation
    end
end

