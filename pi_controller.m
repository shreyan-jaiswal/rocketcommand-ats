classdef pi_controller
    properties
        p;
        i;
    end
    
    methods
        
        function obj = pi_controller(p,i)
            obj.p = p;
            obj.i = i;
        end
        
        function U_k = control(obj,e)
            output = obj.p*e(1)+obj.i*e(2);
            
            if(output<0)
                U_k = 0;
            elseif(output>0.125)
                U_k = 0.125; 
            else
                U_k = output;
            end
        end
        
    end
end