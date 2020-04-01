Ts=0.01;
Tmax=60;
t=0:Ts:Tmax;
N=length(t);

Stockholm = 59;
Lund = 56;

% Initialize state vector x
x=zeros(10,1);
x(end)=1;

% Try out different biases
accbias = [0;0;0];
gyrobias = [0.01*pi/180;0;0]
u = [gravity(Lund,0) + accbias ; gyrobias];

% simulate IMU standalone navigation system
pos=zeros(3,N);
for n=2:N
   x = Nav_eq(x,u,Ts);
   pos(:,n) = x(1:3);
end

pos(:, N)

figure(1)
clf
plot(t,pos')
%hold on
%plot(t,exp(t/14)-1)
grid on
ylabel('Position error [m]')
xlabel('Time [s]')

figure(2)
loglog(t,pos')
pos2=9.82*gyrobias(1)*t.^3/6;  % theoretical 
pos3=9.82*(gyrobias(1))^2*t.^4/24; % theoretical
hold on
loglog(t,pos2,'k--')
loglog(t,pos3,'k--')


