classdef LVD < matlab.mixin.Copyable  
    % This is the prediction class used to determine the rocket's apogee
    % and enforce learning on the predictive model
    
    properties
        % constant system parameters
        m = 1.327;
        fin_wid = 0.125;
        rocket_rad = 0.25;
        
        % constant physical parameters
        g = 32; % 
        ro = 0.00237; %slugs/ft3
        
        % constant general prediction parameters
        apo_goal = 5100;
        dt = 0.512;
        
        % constant LINR prediction parameters
        W_size = 11;
        W = [0.40; 0; 0; 0; 0; 0; 0; 1.0464; 4.968; 21.25; 0];

          
    end
    
    % -----------------------A note about notation-------------------------
    % X designates a state, U designates a control
    % t subscript simply refers to a state or control at a given time
    % f subscript is for the final state or control at apogee
    % p subscript refers to a predicted state or control
    % these subscrtipts are often used together for states and controls
    
    methods
        function obj = LINR(varargin)
            if ~isempty(varargin)
                for i = 1:length(varargin)/2
                    switch varargin{2*i-1}
                        % rocket parameters
                        case 'm'
                            obj.m = varargin{2*i};
                        case 'fin_wid'
                            obj.fin_wid = varargin{2*i};
                        case 'rocket_rad'
                            obj.rocket_rad = varargin{2*i};
                        case 'apo_goal'
                            obj.apo_goal = varargin{2*i};
                        case 'W'
                            obj.W = varargin{2*i};

                    end
                end
            end
        end
        
        % computational functions
        function cd = compute_cd(obj,X_t,U_t)
            d1 = U_t(1);
            v1 = X_t(1);
            cd = [1 v1 v1^2 v1^3 v1^4 v1^5 v1^6 d1 d1^2 d1^3 d1^4]*obj.W;   
        end
        
        function fd = compute_fd(obj,X_t,U_t)
            area = compute_area(obj,U_t);
            
            v2 = X_t(1)^2 + (X_t(1)*tan(pi/180*X_t(3)))^2;
            fd = obj.compute_cd(X_t,U_t)*0.5*obj.ro*area*v2;
        end
        
        function area = compute_area(obj,U_t)
            area_flaps = 3 * obj.fin_wid * U_t(1);
            
            area_rocket = pi*obj.rocket_rad^2;
            area = area_flaps + area_rocket;
        end
        
        % prediction functions
        function X_tp = ss_predict(obj,X_t,U_t)
            X_tp = [0,0,0];
            
            theta = X_t(3)*pi/180;
            vy = X_t(1);
            vx = vy/tan(theta);
            h = X_t(2);
            
            fd = compute_fd(obj,X_t,U_t);
            
            dh = vy*obj.dt;

            dvy = (-obj.g - cos(theta)*fd/obj.m)*obj.dt;
            dvx = (-sin(theta)*fd/obj.m)*obj.dt;

            vy_p = vy + dvy;
            vx_p = vx + dvx;

            X_tp(1) = vy_p;
            X_tp(2) = h+dh;
            X_tp(3) = (180/pi)*atan(vy_p/vx_p);
        end
            
        function X_vpf = ms_predict(obj,X_t,U_t)
            while(X_t(1) > 0)
                X_vpm1 = obj.ss_predict(X_t,U_t);
                X_t = X_vpm1;
            end
            
            X_vpf = X_t;
        end
     
    end  
end