
classdef Field < handle
    
    properties
        
        width = 0;
        height = 0;
        
        z = [];
    end
   
    methods
        % class constructor
        
        % myField = Field(w,h)
        % myField = Field(pre_generated_map)
        
        function field_obj = Field(varargin)
                    
            if (length(varargin) == 1)
                field_obj.z = varargin{1};
                field_obj.width = size(field_obj.z, 1);
                field_obj.height = size(field_obj.z, 2);
                
                return
            end
            
            width = varargin{1};
            height = varargin{2};
                        
            field_obj.width = width;
            field_obj.height = height;
            
            % create random field with autocorrelation
            std = 1;
            [X,~] = meshgrid(0: width - 1, 0: height - 1);
%             Z = mean + std*randn(size(X)); % normally distributed -- used rand() for a single round point
            Z = randn(size(X)); % normally distributed -- used rand() for a single round point
            
            if (length(varargin) == 3)
                sigma = varargin{3};
            else
                sigma = width;
            end
            

            % autocorrelation
            field_obj.z = std*imfilter(Z, fspecial('gaussian', [width, height], sigma));
            
        end % end constructor
        
        function [fx, fy, fz, error] = sample(field_obj, x, y)
            fx = round(x);
            fy = round(y);
            if (fx < 1 || fx > field_obj.width || fy < 1 || fy > field_obj.height)
                fz = Inf;
                error = true;
            else
                fz = field_obj.z(fx, fy); 
                error = false;
            end
        end
    end
end

