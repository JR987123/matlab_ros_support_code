%% HW #9 Jaren & Josh & Joel Mechatronics
%% Clearing Variables and Reseting World and Robot
clc;
clear;
rosshutdown;
masterhostIP = "192.168.19.128"
rosinit(masterhostIP);
goHome('qr');


%% Positions from Gazebo

models = getModels;                         
rCan1 = models.ModelNames{25}; 
fprintf('Picking up 1st Red Can: %s \n',rCan1);
[rCan1_R_T_G, rCan1_R_T_M] = get_robot_object_pose_wrt_base_link(rCan1)

                       

strategy = 'topdown';
ret = pick_HW09_JR(strategy, rCan1_R_T_M);

if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,rCan1_R_T_M,place_pose);
end

% Can 2
rCan2 = models.ModelNames{26}; 
fprintf('Picking up 2nd Red Can: %s \n',rCan2);
[rCan2_R_T_G, rCan2_R_T_M] = get_robot_object_pose_wrt_base_link(rCan2)          

goHome('qr');
ret = pick_HW09_JR(strategy, rCan2_R_T_M);
if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,rCan1_R_T_M,place_pose);
end


% Can 3
rCan3 = models.ModelNames{24}; 
fprintf('Picking up 3rd Ren Can: %s \n',rCan3);
[rCan3_R_T_G, rCan3_R_T_M] = get_robot_object_pose_wrt_base_link(rCan3)

goHome('qr');
pause(5)
ret = pick_HW09_JR(strategy, rCan3_R_T_M);
if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,rCan1_R_T_M,place_pose);
end













