function [vL,vR,curv,dist,theta] = hl_ctrl(lat,long,heading,v_d,lad,kk,v,dt)
  lat = lat(:); long=long(:); heading = heading(:); v = v(:); 
  pg = find_goal([lat(1),long(1),heading(1)],lad); 

  gLat = [pg(1);lat(1:end-1)]; 
  gLong = [pg(2);long(1:end-1)]; 
  [dist,bearing] = haversine(gLat,gLong,lat,long);

  kp=0.75;kd=1;h=1; nom_dist = 1; 
  accel = [dist(2:end)-nom_dist,-v(2:end),v(1:end-1)-v(2:end)]*[kp;kp*h;kd];
  persistent v_des
  if isempty(v_des)
      v_des = [v_d;0*v(2:end)]; 
  end
  v_des = [v_d;v_des(2:end)+accel*dt]; 
  
  theta = bearing - heading; 
  curv = pure_pursuit(dist,theta);
  [vL,vR] = steering(v_des,curv,kk);
  %v = [vL;vR]; 
end

function curv = pure_pursuit(dist, theta)
  curv = 2.*sin(theta)./dist; 
end
function [vL,vR] = steering(vel,curv,kk)
  L = 0.319 .* kk;
  vL = vel .* (1 + curv .* (L ./ 2)); 
  vR = vel .* (1 - curv .* (L ./ 2)); 
end
function pg = find_goal(pr,l)
  persistent pn  
  if isempty(pn)
%     file = load('../set_goal/pn3.mat','pn'); %coord-by-obs (2-by-n)
%     pn = file.pn; 
    pn = csvread('../path_norm/path.csv');
    clear file; 
  end

  % find nearest point on path
  d = abs(pr(1)-pn(1,:)) + abs(pr(2)-pn(2,:));
%   d = sqrt((pr(1)-pn(1,:)).^2 + (pr(2)-pn(2,:)).^2);
  [~,Ipc] = min(d);

%   pc = pn(:,Ipc); %closest point

  % get goal point
  Ipg = mod(Ipc + l - 1, size(pn,2)) + 1;
  pg = pn(:,Ipg);
end
function [distance,bearing] = haversine(glat,glong,rlat,rlong)
  R = 6372800; % radius of earth in meters
  lat1 = degtorad(rlat); lat2=degtorad(glat);
  dLat = lat2-lat1;
  dLon = degtorad(glong-rlong);

  a = sin(dLat./2).^2 + cos(lat1).*cos(lat2).*sin(dLon./2).^2; 
  c = 2.*asin(sqrt(a)); 
  distance = R .* c; 
  y = sin(dLon) .* cos(lat2); 
  x = cos(lat1) .* sin(lat2) - sin(lat1) .* cos(lat2) .* cos(dLon); 
  bearing = atan2(y,x); 
  %[distance/1000, bearing * 180/pi]
end

