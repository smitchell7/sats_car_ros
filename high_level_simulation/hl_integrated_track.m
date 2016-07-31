
%% functionname: function description
function pe=hl_integrated_track(v_des,lad,kk,plot_on,plot_mov)

switch nargin
    case 0
        v_des = 2; 
        lad = 41; 
        kk = [1;1]; 
        plot_on = 1; 
        plot_mov = 0; 
    case 1
        lad = 19; 
        kk = 1; 
        plot_on = 1; 
        plot_mov = 0; 
    case 2
        kk = 1; 
        plot_on = 1; 
        plot_mov = 0; 
    case 3
        plot_on = 1; 
        plot_mov = 0; 
    case 4 
        plot_mov = 0; 
end

deglat = 111069; % meters
deglong = 83162; 
N=2;
HP = struct(...
    'lookahead',lad...
    ,'kk',kk...
    ,'v_des',v_des...
    ,'L',0.319...
    ,'k',[0.5;0.3]...
    ,'N',N ...
    ); 

c1 = [41.75939;... %lat
 -111.8166653;... % long
 1.*pi;... % heading
 0;0; ... %error integral
 0;0 ... %velocity
 ];
c2 = c1; c2(1)=c2(1)+1/deglat; 
dyn = reshape([c1,c2]',[],1); 
sim_time = 200; dt = 0.2; 

t=linspace(0,sim_time,sim_time/dt).';
% v_des = 5; %lad = 30; 
f_w = @(t,Z) f(t,Z,HP); 
y = ode4(f_w,t,dyn); 

lat =           y(:,1:N); 
long=           y(:,N+1:2*N); 
heading =       y(:,2*N+1:3*N); 
error_int_L =   y(:,3*N+1:4*N); 
error_int_R =   y(:,4*N+1:5*N); 
vL =            y(:,5*N+1:6*N); 
vR =            y(:,6*N+1:7*N); 

pn = csvread('../path_norm/path.csv');
if plot_on
    pnm = bsxfun(@times,bsxfun(@minus,pn.',min(pn.')),[deglat,deglong]); 
    ym = bsxfun(@times,bsxfun(@minus,y(:,1:2),min(pn.')),[deglat,deglong]); 
    latm = (lat-min(pn(1,:))) * deglat; 
    lonm = (long-min(pn(2,:))) * deglong; 
    
    figure(1)
    plot(pnm(:,2),pnm(:,1),'LineWidth',2);hold on
    plot(lonm,latm); hold off
    title('Vehicle following gps coordinates'); 
    legend('test track','bb1','bb2')

    figure(2)
    hold off
    vid = VideoWriter('on_test_track'); 
    vid.FrameRate = round(1/dt); 
    open(vid); 
    % 
    plot(pn(2,:),pn(1,:))
    axis([min(long(:)),max(long(:)),min(lat(:)),max(lat(:))])
    title('Vehicle following gps coordinates')
    xlabel('degrees longitude')
    ylabel('degrees latitude')
    hold on
    handle = plot(long(1,:),lat(1,:),'o'); 
end
path_error = nan(length(y),N); 
dist = path_error; 
curv = path_error; 
theta= path_error; 
    for i = 1:length(y)
        [path_error(i,:),~] = min(...
            sqrt(...
            (bsxfun(@minus,kron(pn(1,:),ones(N,1))',lat(i,:)).*111069).^2+...
            (bsxfun(@minus,kron(pn(2,:),ones(N,1))',long(i,:)).*83162).^2));
%         [~,curv(i)]= hl_ctrl(y(i,1:3),v_des,lad,kk); 
        [~,~,curv(i,:),dist(i,:),theta(i,:)] = hl_ctrl(lat(i,:),long(i,:),heading(i,:),v_des,lad,kk,(vL(i,:)+vR(i,:))/2,dt); 
        if plot_mov && ~mod(i,1)
            set(handle,'XData',long(i,:),'YData',lat(i,:)); 
            title(['Vehicle following gps coordinates. t=',num2str(t(i))])

            drawnow
            writeVideo(vid,getframe); 
        end
    end
if plot_on    
    hold off
    figure(3)
    subplot(3,1,1)
    plot(t,path_error);title('path error')
    subplot(3,1,2)
    plot(t,curv); title('Commanded curvature'); 
    subplot(3,1,3)
    plot(t,dist); title('Distance to target'); 
    figure(4)
    subplot(3,1,1)
    
    plot(t,(vL+vR)/2)%,t,v_des+v_des./2.*sin(0.1.*t))
    title('velocity')
    subplot(3,1,2)
    plot(t,heading)
    title('heading')
    subplot(3,1,3)
    plot(t,wrapTo180(theta*180/pi))
    title('angle to target');legend('bb1','bb2')
%     subplot(3,1,1)
    
    close(vid)
end
pe = max(path_error);
end
%% f: 
function [Z_dot] = f(t,Z,HP)
persistent counter
if isempty(counter)
    counter = 0;
end

lookahead = HP.lookahead; 
kk = HP.kk; 
v_des = HP.v_des; 
% v_des = v_des+v_des * sin(0.1*t)/2; 

L = HP.L; 
k = HP.k(:); 
N = HP.N; 
db = [0.13;0.14]*36; 
a = [-2.7,0;0,-2.6]; 
b = [0.8,0;0, 0.77]; 


%L = 0.319;
%k = [0.5;0.3]; 
deglat = 1/111069; % meters
deglong = 1/83162; 

% fill the position and velocity vectors. 
noise = kron([0.5*deglat;0.5*deglong;10*pi/180;0;0;0.05;0.05],ones(N,1)) .* randn(length(Z),1); 
noise = 0; 

Z = Z+noise; 

lat = Z(1:N); 
long= Z(N+1:2*N); 
heading = Z(2*N+1:3*N); 

error_int_L = Z(3*N+1:4*N); 
error_int_R = Z(4*N+1:5*N); 

vL = Z(5*N+1:6*N); 
vR = Z(6*N+1:7*N); 

vel = (vL+vR)./2; 

vN = vel .* cos(heading);
vE = vel .* sin(heading); 
dt = 0.2; 
[vL_des,vR_des] = hl_ctrl(lat,long,heading,v_des,lookahead,kk,vel,dt); 

v_err = [vL_des,vR_des] - [vL,vR]; 
vL_err = vL_des - vL; 
vR_err = vR_des - vR; 

uL = [vL_err,error_int_L] * k; 
uR = [vR_err,error_int_R] * k; 
% u = v_err' * k; 
%u = [uL,uR]';'
%error = vd_vec-v; 
%u = [error,error_int] * k; 

% v = [vL,vR]'; 
% dv = a*v + b*(u+db); 
dvL = a(1,1)*vL + b(1,1)*(uL+db(1)); 
dvR = a(2,2)*vL + b(2,2)*(uR+db(2)); 


Z_dot = [...
    vN*deglat;... % lat
    vE*deglong;... % long
    (vL-vR)./L;... % heading
    dvL;dvR; ... % motor accel
    vL_err; ... % motor error
    vR_err ... % motor error
    ];

end
