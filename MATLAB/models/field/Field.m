
classdef Field < handle
    
    properties
        
        width = 0;
        height = 0;
        
        z = [];
    end
   
    methods
        % class constructor
        function field_obj = Field(width, height)
            field_obj.width = width;
            field_obj.height = height;
            
            % create random field with autocorrelation
            mean = 100;
            std = 100;
            [X,~] = meshgrid(0: width - 1, 0: height - 1);
%             Z = mean + std*randn(size(X)); % normally distributed -- used rand() for a single round point
            Z = randn(size(X)); % normally distributed -- used rand() for a single round point
        
            sigma = 48;

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

