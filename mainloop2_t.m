% simulation parameters

% signal to noise ratio
snr = 0.04;


% initialize rocket prediction class
r = LVD();

% initialize simulation driver prediction class- we use this special
% instance of the prediction class to simulate rocket states so that we can
% test the prediction and control properties
s = LVD();

% create a seed state and control corresponding to the rocket state at 
% burnout. These will be updated to the actual states of the rocket
X_t = [600.7, 1030, 0]; 
U_t = 0;

% finally, we initialize the controller and the error it operates on
c = pi_controller(1E-5, 2.E-4);
e = [0,0];

while(X_t(1)>0)
    % in real life, we create this state by assimilating 
    % altitude, attitude and velocity data from the ATS sensors
    X_t = s.ss_predict(X_t,U_t);
    
    
    X_vm = X_t;
    X_vm(1) = X_vm(1) + snr*X_vm(1)*(2*rand()-1);
          
    % make a prediction and enact control
        
    X_vpf = r.ms_predict(X_vm,U_t);
    
    e(1) = X_vpf(2)-r.apo_goal;
    e(2) = e(2)+e(1);
    U_t = c.control(e); 
end