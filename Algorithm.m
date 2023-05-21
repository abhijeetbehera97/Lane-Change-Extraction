clear 
close all
clc

data=load('Data.mat');

[I,I_tr]=LaneChangeExtraction(data);

geoplot(data.lat_tractor(I(1:2)),data.long_tractor(I(1:2)),'-b','linewidth',2)
hold on
geoplot(data.lat_tractor(I_tr(1:2)),data.long_tractor(I_tr(1:2)),'--r','linewidth',2)
geoplot(data.lat_road,data.long_road,'y','linewidth',1)
legend('Tractor','Trailer','Road','interpreter','latex')

   x0=10;
   y0=5;
   width=9;
   height=6.5;
   set(gcf,'units','centimeters','position',[x0,y0,width,height])


%% Function for lane change extraction
function [I,I_tr] = LaneChangeExtraction(data)

yawrate_tractor = data.yawrate_tractor;
yawrate_trailer = data.yawrate_trailer;
yawangle_trailer = data.yawangle_trailer;
x_tractor=data.x_tractor;
y_tractor=data.y_tractor;
x_road= data.x_road; 
y_road= data.y_road;
head_angle_road=data.head_angle_road;
time=data.time;


interval_info=[];
lanechange_info=[];
I=[];
I_tr=[];

k=1;
yawrate_thres=1;
interval_time=5;

zero_crossing=[];

% Zero crossing for tractor and trailer
while(k~=length(yawrate_tractor)-1)
    if(yawrate_tractor(k)*yawrate_tractor(k+1)<= 0 || k==1)
       m=k+1; 
        while( m < length(yawrate_trailer))
           if(yawrate_trailer(m)*yawrate_trailer(m+1)<=0 ) 
               break;
           end 
              m=m+1;
        end
        zero_crossing(end+1,:)=[k+1 m];
    end
    k=k+1;
end

% Yaw rate threshold

for i=1:length(zero_crossing)-1
    max_yawrate=max(abs(yawrate_tractor(zero_crossing(i,1):zero_crossing(i+1,1))));
   if(max_yawrate> yawrate_thres)
     interval_info(end+1,:)=[zero_crossing(i,1) zero_crossing(i+1,1) zero_crossing(i+1,2) ...
                              max_yawrate*sign(mean(yawrate_tractor(zero_crossing(i,1):zero_crossing(i+1,1))))];
   end

end


k = 1;
i = 1;

% Stacking up intervals 

while(i<=size(interval_info,1)-1)  

% Time threshold condition

  if((time(interval_info(i+1,1))-time(interval_info(i,2))) <= interval_time ...
          && (interval_info(i,4))*(interval_info(i+1,4))<0)   
 
 % Lateral Displacement calculation   
 
     [~,start_indx]=min(sqrt((x_tractor(interval_info(i,1))- (x_road)).^2 + (y_tractor(interval_info(i,1))- (y_road)).^2));
     [~,end_indx]=min(sqrt((x_tractor(interval_info(i+1,2))- (x_road)).^2 + (y_tractor(interval_info(i+1,2))- (y_road)).^2));

        angle_road_end= wrapTo180(head_angle_road(end_indx)-180);
        end_start_road_yaw=atan2d(y_road(start_indx)-y_road(end_indx),x_road(start_indx)-x_road(end_indx));
        lat_disp_road = (sqrt((y_road(end_indx)-y_road(start_indx))^2 + (x_road(end_indx)-x_road(start_indx))^2))*sind(abs(end_start_road_yaw-angle_road_end));
           
      yaw_road_end = angle_road_end;

      end_start_truck_yaw=atan2d(y_tractor(interval_info(i,1))-y_tractor(interval_info(i+1,2)),x_tractor(interval_info(i,1))-x_tractor(interval_info(i+1,2)));
   
      lat_disp_truck=sqrt((x_tractor(interval_info(i,1))-x_tractor(interval_info(i+1,2)))^2+...
         (y_tractor(interval_info(i,1))-y_tractor(interval_info(i+1,2)))^2)*...
         sind(abs(end_start_truck_yaw-yaw_road_end));
    
    lanechange_info(k,1)  = interval_info(i,1);    % Tractor start interval 1
    lanechange_info(k,2)  = interval_info(i,2);    % Tractor end interval 1
    lanechange_info(k,3)  = interval_info(i,3);    % Trailer end interval 1
    lanechange_info(k,4)  = interval_info(i+1,2);  % Tractor end interval 2
    lanechange_info(k,5)  = interval_info(i+1,3);  % Trailer end interval 2

   % Lateral displacement and yaw angle threshold. 

   if(abs(lat_disp_road-lat_disp_truck)<=7.5 && abs(lat_disp_road-lat_disp_truck)>=2 &&...
            abs(max(yawangle_trailer(lanechange_info(k,1):lanechange_info(k,5)))-...
                min(yawangle_trailer(lanechange_info(k,1):lanechange_info(k,5))))<20)
        I(end+1,:)=[lanechange_info(k,1) lanechange_info(k,4)];  
        I_tr(end+1,:)=[lanechange_info(k,1) lanechange_info(k,5)];  
        

   end
    
     k=k+1;  

    y_dot=[]; 

  end  
      
  i=i+1;    

end



end

      

