dt = 1e-3;                              % sample rate 1kHz
t = dt*(1:1000);
N = length(t);
s = t + 0.5*sin(2*pi*10*t) - 2*(t>0.5); % the true signal

a = 0.97;                               % Sensor speed, range [0,1), smaller a-value gives faster sensor 
sigma_e1 = 0.1;                         % Noise level on red measurements
sigma_e2 = 0.01;                        % Noise level on green measurements
randn('state',0)                        % Initialise random number generator
e1 = sigma_e1*randn(1,N);               % Measurement noise
e2 = sigma_e2*randn(1,N);               % Measurement noise
bias = 0.3 + 2*sin(t);                  % Slowly varying bias
y1 = filter(1-a,[1 -a],s) + e1 ;        % Red signal, slow no bias
y2 = s + bias + e2;                     % Green signal, fast but with bias 

figure(1)
plot(t,s,'b',t,y1,'r',t,y2,'g')
hold on

sigma_bias = 0.02;     % Estimate of bias change rate, found by some trial and error
sigma_signal = 0.06;   % Estimate of signal change rate, same 
Q = blkdiag(0,sigma_bias^2,sigma_signal^2); % process noise covariance matrix
R = blkdiag(sigma_e1^2,sigma_e2^2);  % measurement noise covariance matrix
F = [a 0 1-a; 0 1 0; 0 0 1];
H1 = [1 0 0];          % slow sensor
H2 = [0 1 1];          % fast sensor with bias    
H = [H1;H2]; 
y = [y1;y2];
%ind = 1;             % only first measurement available
%ind = 2;             % only second measurement available
ind = 1:2;            % both measurements available
y = y(ind,:);         % pick out relevant parts
H = H(ind,:);
R = R(ind,ind);

% Initialise Kalman filter
P = 0*eye(3);       % influences initial convergence
xhat = zeros(3,N);  % storage for state estimates 
% Run Kalm% Initialise Kalman filter
P = 0*eye(3);       % influences initial convergence
xhat = zeros(3,N);  % storage for state estimates 
% Run Kalman filter
for k = 1:N
    % time update
    if k>1, xhat(:,k) = F*xhat(:,k-1); end
    P = F*P*F' + Q;
    % measurement update
    K = (P*H')/(H*P*H'+R);
    xhat(:,k) = xhat(:,k) + K*(y(:,k)-H*xhat(:,k));
    P = P - K*H*P;
end
shat = xhat(3,:); % the sensor fusion estimate of s(t)an filter

figure(1)
plot(t,shat,'k')
legend('True signal', 'y1','y2','Kalman filter')
rmserror = sqrt(mean((shat-s).^2))

%close all
ind = 1;
y = y(ind,:);      
H = H(ind,:);
R = R(ind,ind);

% Initialise Kalman filter
P = 0*eye(3);       % influences initial convergence
xhat = zeros(3,N);  % storage for state estimates 
% Run Kalm% Initialise Kalman filter
P = 0*eye(3);       % influences initial convergence
xhat = zeros(3,N);  % storage for state estimates 
% Run Kalman filter
for k = 1:N
    % time update
    if k>1, xhat(:,k) = F*xhat(:,k-1); end
    P = F*P*F' + Q;
    % measurement update
    K = (P*H')/(H*P*H'+R);
    xhat(:,k) = xhat(:,k) + K*(y(:,k)-H*xhat(:,k));
    P = P - K*H*P;
end
shat = xhat(3,:); % the sensor fusion estimate of s(t)an filter

figure(1)
plot(t,shat,'c')
legend('True signal', 'y1','y2','Kalman filter')
rmserror = sqrt(mean((shat-s).^2))