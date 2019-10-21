classdef ElasticComp
    %ELASTICCOMP Class representing all elastic components
    %   Contains all the properties and methods required to fully define an
    %   elastic component.
    %   Elastic components include; springs, tires, bumpstops etc
    
    properties
        % Input Properties
        name string % Name of elastic component
        forceDefData double % Force vs Deflection data [N vs m]
        freeLength double % Free length of component [m]
        stroke double % Stroke of component [m]
        
        % Computed Properties
        stiffness double % Stiffness [N/m]
        
        % Solid Length Value
        infForce double;
    end
    
    % GENERAL AND UI METHODS
    methods (Access = public)
        function obj = ElasticComp()
            %ELASTICCOMP Construct an instance of this class 
            obj.infForce = 9999999;
        end
    end
    
    % ACCESSIBILITY METHODS
    methods (Access = public)
        % GET FORCE 
        function [force] = Get_Force(obj,currDef,stroke)
            %Get_Force Method to extract the elastic component force based
            %on the its current deflection.
            %   This method used 1D linear interpolation to extract the
            %   force value
            
%             if currDef <= stroke
                force = interp1(obj.forceDefData(:,1),obj.forceDefData(:,2),currDef,'linear','extrap');
%             else % If deflection is greater than stroke, then it means solid length has been reached. Hence. force becomes infinity
%                 force = obj.infForce;
%             end
        end
%         % GET DEFLECTION
%         function [deflection] = Get_Def(obj,currForce,stroke)
%             %Get_Def Method to extract the elastic component's deflection
%             %based on the current force it is subject to 
%             %   This method applies 1D linear interpolation on the REVERSED
%             %   Force vs Deflection data 
%             
%             maxForce = interp1(obj.forceDefData(:,1),obj.forceDefData(:,2),stroke,'linear');
%             
%             if currForce <= maxForce
%                 reverseLookup = [obj.forceDefData(:,2),obj.forceDefData(:,1)];
%                 deflection = interp1(reverseLookup(:,1),reverseLookup(:,2),currForce,'linear');
%             else
%                 deflection = 1/obj.infForce;
%             end
%             
%         end
        
%         % GET STIFFNESS - FROM DEFLECTION
%         function [stiffness] = GetStiffness_FromForce(obj,currForce,stroke)
%             %GetStiffness_FromForce Extracts the stiffness of the elastic
%             %component using the force
%             %   This method uses the current force which the elastic
%             %   component is subject to to extract the stiffness
%             
%             currDef = obj.Get_Def(currForce,stroke);
%             
%             stiffness = currForce/currDef;
%         end
%         
%         % GET STIFFNESS - FROM FORCE
%         function [stiffness] = GetStiffness_FromDef(obj,currDef,stroke)
%             %GetStiffness_FromDef Extracts the stiffness of the elastic
%             %component using the deflection
%             %   This method uses the current deflection of the elastic
%             %   component to ccompute its currrent stiffness
%             
%             currForce = obj.Get_Force(currDef,stroke);
%             
%             stiffness = currForce/currDef;
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
        
        % LOAD SPRING
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
                    objLoaded = load((fullname(1,i)));
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
                obj.forceDefData= table.Variables;
                objReturn = obj;
            catch
                objReturn = obj;
            end
        end
        
        
    end
    
end

