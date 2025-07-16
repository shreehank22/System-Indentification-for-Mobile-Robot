sample_time = 0.01;
% Dataset formulation of control input, position and velocity
control_input = -out.controlinput;
position = -out.position; 
velocity = -out.velocity;
output_matrix = [position, velocity];
data_position = iddata(output_matrix, control_input, sample_time);
data_position.InputName = {'Control Input'};
data_position.OutputName = {'Position', 'Velocity'};
data_position.TimeUnit = 'seconds';

% system identification using N4SID
n = 2;
sys_discrete = n4sid(data_position, n);
sys_continuous = d2c(sys_discrete);  

% extracting matrices A,B,C,D for building the state-space model
A=sys_continuous.A;B=sys_continuous.B;C=-sys_continuous.C;D=sys_continuous.D;
