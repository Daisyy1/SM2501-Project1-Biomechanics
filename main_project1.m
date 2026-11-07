%%% Basic read data and visualization for the course SM2501
%%% Biomechanics of Human Movement at KTH - 2026
clear; close all; clc;
W_R_FS1 = 124; W_R_TO = 177; W_R_FS2 = 212; 
W_L_FS1 = 168; W_L_TO = 223; W_L_FS2 = 257; 
C_R_FS1 = 190; C_R_TO = 250; C_R_FS2 = 288;
%% Setup files
name_motion={'Walking'    'Jogging'   'Crouch'};
name_grf   ={'Walking_FP' 'Jogging_FP' 'Crouch_FP'};

% frame_sel  =[];

index=1; % select the motion to be loaded and visualized e.g., index=1 -> NormWalk
%% Read marker trajectory and ground reaction data
% data files should be in the same folder as the .m file
file_dir = pwd;
data_trc = readtable(fullfile(file_dir,[name_motion{index} '.csv']));
data_grf = readtable(fullfile(file_dir,[name_grf{index} '.csv']));

%% Downsample ground reaction data
% down sample the ground reaction data, so it has the same length as marker trajectory
data_grf_s = data_grf(1:10:end,:);

%% Assign the uploaded table to variables in MATLAB
toMeters=1/1000; % data is originally in mm, it has to be divided by 1000 to have it in meters

RTOE_y=data_trc.RTOO_Y*toMeters;   RTOE_z=data_trc.RTOO_Z*toMeters;    % horizontal coordinate of the right toe & vertical coordinate of the right toe 
LTOE_y=data_trc.LTOO_Y*toMeters;   LTOE_z=data_trc.LTOO_Z*toMeters;    % horizontal coordinate of the left toe & vertical coordinate of the left toe 

RANKLE_y=data_trc.RAJC_Y*toMeters;   RANKLE_z=data_trc.RAJC_Z*toMeters;   
LANKLE_y=data_trc.LAJC_Y*toMeters;   LANKLE_z=data_trc.LAJC_Z*toMeters;   

RKNEE_y=data_trc.RKJC_Y*toMeters;   RKNEE_z=data_trc.RKJC_Z*toMeters;    
LKNEE_y=data_trc.LKJC_Y*toMeters;   LKNEE_z=data_trc.LKJC_Z*toMeters;   

RHIP_y=data_trc.RHJC_Y*toMeters;   RHIP_z=data_trc.RHJC_Z*toMeters;    
LHIP_y=data_trc.LHJC_Y*toMeters;   LHIP_z=data_trc.LHJC_Z*toMeters;  

PELO_y=data_trc.PELO_Y*toMeters;   PELO_z=data_trc.PELO_Z*toMeters;    
PELP_y=data_trc.PELP_Y*toMeters;   PELP_z=data_trc.PELP_Z*toMeters;   

TRXO_y=data_trc.TRXO_Y*toMeters;   TRXO_z=data_trc.TRXO_Z*toMeters;    
TRXP_y=data_trc.TRXP_Y*toMeters;   TRXP_z=data_trc.TRXP_Z*toMeters;  

FP1_force_y=data_grf_s.FP1_Force_Y;            FP1_force_z=data_grf_s.FP1_Force_Z;
FP1_COP_y  =(600 - data_grf_s.FP1_COP_Y)*toMeters;     FP1_COP_z  =data_grf_s.FP1_COP_Z*toMeters;

FP2_force_y=data_grf_s.FP2_Force_Y;            FP2_force_z=data_grf_s.FP2_Force_Z;
FP2_COP_y=(-600 - data_grf_s.FP2_COP_Y)*toMeters;       FP2_COP_z=data_grf_s.FP2_COP_Z*toMeters;

Frame_W = data_trc.Frame;

%%PHASE 2: Kinematics - Joint Angles
% WALKING
% Trunk absolute angle (relative to vertical)
trunk_dy = TRXO_y - TRXP_y;
trunk_dz = TRXO_z - TRXP_z;
trunk_angle_W = atan2(trunk_dy, trunk_dz) %it's in radiant!
% Convert trunk angle from radians to degrees for easier interpretation
trunk_angle_W_deg = rad2deg(trunk_angle_W);
%{
figure (1);
plot(trunk_angle_W_deg)
ylabel('Trunk angle (deg)')
xlabel('Frame')
title('Trunk angle - Walking')
%}
% Interpretation: >0 = trunk leans forward (anterior tilt)
% ~0 = trunk upright
% <0 = trunk leans backward (posterior tilt)

% Pelvis: PELO = origin/inferior; PELP = proximal/superior
pelvis_dy = PELP_y - PELO_y;
pelvis_dz = PELP_z - PELO_z;
pelvis_angle_W = atan2(pelvis_dy, pelvis_dz);
pelvis_angle_W_deg = rad2deg(pelvis_angle_W)
% >0 = anterior pelvic tilt; <0 = posterior
%{
figure (2);
plot(pelvis_angle_W)
ylabel('Pelvis angle (deg)')
xlabel('Frame')
title('Pelvis angle - Walking')
%}
% REVIEW these results!!!! the angles seemed too big

% Right hip angle (walking)
thigh_R_dy = RHIP_y - RKNEE_y; % hip minus knee = points upward
thigh_R_dz = RHIP_z - RKNEE_z;
thigh_R_abs = atan2(thigh_R_dy, thigh_R_dz);
hip_R_W = rad2deg(thigh_R_abs - pelvis_angle_W);
% >0 = hip flexion (thigh forward of pelvis)
% <0 = hip extension (thigh behind pelvis)
% Left hip: same but use LHIP and LKNEE
thigh_L_dy = LHIP_y - LKNEE_y;
thigh_L_dz = LHIP_z - LKNEE_z;
thigh_L_abs = atan2(thigh_L_dy, thigh_L_dz);
hip_L_W = rad2deg(thigh_L_abs - pelvis_angle_W);
%{
figure (3);
plot(hip_R_W)
ylabel('Hip relative angle R_deg')
xlabel('Frame')
title('R Hip rel angle - Walking')
figure (4);
plot(hip_L_W)
ylabel('Hip relative angle L_deg')
xlabel('Frame')
title('L Hip rel angle - Walking')
%}
%CHECK ANGLES! do they make sense?

% Right knee angle (walking)
shank_R_dy = RKNEE_y - RANKLE_y; % knee minus ankle = points upward
shank_R_dz = RKNEE_z - RANKLE_z;
shank_R_abs = atan2(shank_R_dy, shank_R_dz);
knee_R_W = rad2deg(shank_R_abs - thigh_R_abs);
% >0 = knee flexion; <0 = hyperextension (rare, ~0 at full extension)
shank_L_dy = LKNEE_y - LANKLE_y; % knee minus ankle = points upward
shank_L_dz = LKNEE_z - LANKLE_z;
shank_L_abs = atan2(shank_L_dy, shank_L_dz);
knee_L_W = rad2deg(shank_L_abs - thigh_L_abs);
%{
figure (5);
plot(knee_R_W)
ylabel('Knee relative angle R_deg')
xlabel('Frame')
title('R Knee rel angle - Walking')
figure (6);
plot(knee_L_W)
ylabel('Knee relative angle L_deg')
xlabel('Frame')
title('L Knee rel angle - Walking')
%}

% Right ankle angle (walking)
foot_R_dy = RTOE_y - RANKLE_y; % toe minus ankle = foot vector
foot_R_dz = RTOE_z - RANKLE_z;
foot_R_abs = atan2(foot_R_dy, foot_R_dz);
ankle_R_W = rad2deg(foot_R_abs - shank_R_abs) -90 + 5;
% +90 → makes anatomically neutral = 0°
% +5 → anatomical correction (project-specified)
% >0 = dorsiflexion; <0 = plantarflexion
foot_L_dy = LTOE_y - LANKLE_y; % toe minus ankle = foot vector
foot_L_dz = LTOE_z - LANKLE_z;
foot_L_abs = atan2(foot_L_dy, foot_L_dz);
ankle_L_W = rad2deg(foot_L_abs - shank_L_abs) - 90 + 5;
%{
figure (7);
plot(ankle_R_W)
ylabel('Ankle relative angle R_deg')
xlabel('Frame')
title('R Ankle rel angle - Walking')
figure (8);
plot(ankle_L_W)
ylabel('Ankle relative angle L_deg')
xlabel('Frame')
title('L Ankle rel angle - Walking')
%}


% ---------- Walking, right (124–212) ----------
rows_W_R   = find(Frame_W == W_R_FS1) : find(Frame_W == W_R_FS2);
n_W_R      = length(rows_W_R);% 89
gc_W_R     = linspace(0, 100, n_W_R);
TO_pct_W_R = (W_R_TO - W_R_FS1) / (W_R_FS2 - W_R_FS1) * 100;

trunk_W_R_gc  = trunk_angle_W(rows_W_R);
pelvis_W_R_gc = pelvis_angle_W(rows_W_R);
hip_R_W_gc    = hip_R_W(rows_W_R);
knee_R_W_gc   = knee_R_W(rows_W_R);
ankle_R_W_gc  = ankle_R_W(rows_W_R);

% ---------- Walking, left (168–257) ----------
rows_W_L   = find(Frame_W == W_L_FS1) : find(Frame_W == W_L_FS2);
n_W_L      = length(rows_W_L);% 90
gc_W_L     = linspace(0, 100, n_W_L);
TO_pct_W_L = (W_L_TO - W_L_FS1) / (W_L_FS2 - W_L_FS1) * 100;
trunk_W_L_gc  = trunk_angle_W(rows_W_L);
pelvis_W_L_gc = pelvis_angle_W(rows_W_L);
hip_L_W_gc    = hip_L_W(rows_W_L);
knee_L_W_gc   = knee_L_W(rows_W_L);
ankle_L_W_gc  = ankle_L_W(rows_W_L);

% ---------- Crouch, right (190–288) ----------
%{
rows_C_R   = find(Frame_C == C_R_FS1) : find(Frame_C == C_R_FS2);
n_C_R      = length(rows_C_R);                 % 99
gc_C_R     = linspace(0, 100, n_C_R);
TO_pct_C_R = (C_R_TO - C_R_FS1) / (C_R_FS2 - C_R_FS1) * 100;

trunk_C_R_gc  = trunk_angle_C(rows_C_R);
pelvis_C_R_gc = pelvis_angle_C(rows_C_R);
hip_R_C_gc    = hip_R_C(rows_C_R);
knee_R_C_gc   = knee_R_C(rows_C_R);
ankle_R_C_gc  = ankle_R_C(rows_C_R);
%}
% ---------- check ----------
fprintf('W_R: frames %d–%d, n = %d\n', Frame_W(rows_W_R(1)), Frame_W(rows_W_R(end)), n_W_R);
fprintf('W_L: frames %d–%d, n = %d\n', Frame_W(rows_W_L(1)), Frame_W(rows_W_L(end)), n_W_L);
%fprintf('C_R: frames %d–%d, n = %d\n', Frame_C(rows_C_R(1)), Frame_C(rows_C_R(end)), n_C_R);
