classdef DampingComp
    %DAMPINGCOMP Class representing damping components
    %   Contains all the properties and definitions required to fully
    %   define a velocity based damping coponent
    %   Damping components include; dampers, tires etc
    
    properties
        % Input Properties
        name string % Damping Component Name
        forceVelData double % Force vs Velocity data [N vs m/s]
        freeLength double % Free Length of component [m]
        stroke double % Stroke of component [m]
        
        % Computed Properties
        dampingCoeff double % Damping Coefficient of component [Ns/m]
    end
    
    % GENERAL AND UI METHODS
    methods (Access = public)
        function obj = DampingComp()
            %DAMPINGCOMP Construct an instance of this class            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
    
    % ACCESSIBILITY METHODS
    methods (Access = public)
        % GET FORCE 
        function [force] = Get_Force(obj,currShaftVel)
            %Get_Force Extracts the Damping Force from the current shaft
            %velocity
            
            force = interp1(obj.forceVelData(:,1),obj.forceVelData(:,2),currShaftVel,'linear','extrap');
        end
        
%         % GET DAMPING COEFFICIENT - FROM SHAFT VELOCITY
%         function [dampingCoeff] = GetDamping_FromVel(obj,currShaftVel)
%             %GetDamping_FromVel Extracts the damping coefficient from shaft
%             %velocity
%             
%             currForce = obj.Get_Force(currShaftVel);
%             
%             dampingCoeff = currForce/currShaftVel;
%         end
    end
    
    % SAVE / LOAD OBJECT FUNCTIONS
    methods (Access = public)
        % SAVE 
        function Save(obj)
            %Save Method to save the object 
            %   Saves the object in a mat file in the location
            %   specified by the user 
            
            % Creating a filer to only allow the user to save as .mat files
            filter = {'*.mat'};
            % Extracting the name and file path of the user choice
            [file,path] = uiputfile(filter);
            % Creating the fullname of the file which includes the path of
            % the file as well
            fullname = fullfile(path,file);
            % Saving the file with the name and path of the user's choice
            [~,obj.name,~] = fileparts(fullname);
            
            % Try/Catch Statement to prevent error in case user presses
            % cancel or selects a file and then presses cancel
             try
                 save(fullname,"obj");
             catch
             end
        end
        
        % LOAD 
        function [objReturn] = Load(obj,multiSelectOn)
            %Load Method to load the object
            %   Loads the object from a mat filee in the location
            %   specified by the user 
            
            % Creating a filer to only allow the user to load .mat files
            filter = {'*.mat'};
            % Extracting the name and file path of the user choice
            [file,path] = uigetfile(filter,'MultiSelect',multiSelectOn);
            % Creating the fullname of the file which includes the path of
            % the file as well
            fullname = fullfile(string(path),string(file));
            % Loading the file which the user has selected
             % Try/Catch Statement to prevent error in case user presses
            % cancel or selects a file and then presses cancel 
            try
                for i = 1:length(fullname)
                    objLoaded = load(string(fullname(1,i)));
                    objReturn(i) = objLoaded.obj;
                end
            catch
                objReturn = obj;
            end 
        end
        
        % Load Excel File of 
        function [objReturn] = Load_XLS(obj,multiSelectOn)
            %Load_XLS Loads excel sheet data into 
            
            % Creating a filer to only allow the user to load .xls files
            filter = {'*.xlsx';'*.xlsx';'*.dat'};
            % Extracting the name and file path of the user choice
            [file,path] = uigetfile(filter,'MultiSelect',multiSelectOn);
            % Creating the fullname of the file which includes the path of
            % the file as well
            fullname = fullfile(string(path),string(file));
            % Creating a Table out of the loaded data
            try
                for i = 1:length(fullname)
                    table = readtable(fullname(1,i));
                end
                obj.forceVelData = table.Variables;
                objReturn = obj;
            catch
                objReturn = obj;
            end
        end
        
        
    end
end

