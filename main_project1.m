%%% Basic read data and visualization for the course SM2501
%%% Biomechanics of Human Movement at KTH - 2026
clear; close all; clc;
W_R_FS1 = 124; W_R_TO = 177; W_R_FS2 = 212; 
W_L_FS1 = 168; W_L_TO = 223; W_L_FS2 = 257; 
C_R_FS1 = 190; C_R_TO = 250; C_R_FS2 = 288;

dt = 0.01;      % sampling interval (s), 100 Hz

BM = 61.9;      % body mass (kg)
H  = 1.770;     % height (m)

% Segment mass
m_foot   = 0.0145 * BM;
m_shank  = 0.0465 * BM;
m_thigh  = 0.100  * BM;
m_pelvis = 0.142  * BM; 

% CoM location
com_foot_frac   = 0.50;     % from ankle toward toe
com_shank_frac  = 0.43;     % from knee toward ankle
com_thigh_frac  = 0.43;     % from hip toward knee
com_pelvis_frac = 0.25;     % from PELO toward PELP

%% Read WALKING marker trajectory and ground reaction data
% data files should be in the same folder as the .m file
file_dir = pwd;
data_trc_W = readtable(fullfile(file_dir,'Walking.csv'));
data_grf_W = readtable(fullfile(file_dir,'Walking_FP.csv'));

%% Downsample ground reaction data
% down sample the ground reaction data, so it has the same length as marker trajectory
data_grf_s_W = data_grf_W(1:10:end,:);

%% Assign the uploaded table to variables in MATLAB
toMeters=1/1000; % data is originally in mm, it has to be divided by 1000 to have it in meters

RTOE_y_W=data_trc_W.RTOO_Y*toMeters;   RTOE_z_W=data_trc_W.RTOO_Z*toMeters;    % horizontal coordinate of the right toe & vertical coordinate of the right toe 
LTOE_y_W=data_trc_W.LTOO_Y*toMeters;   LTOE_z_W=data_trc_W.LTOO_Z*toMeters;    % horizontal coordinate of the left toe & vertical coordinate of the left toe 

RANKLE_y_W=data_trc_W.RAJC_Y*toMeters;   RANKLE_z_W=data_trc_W.RAJC_Z*toMeters;   
LANKLE_y_W=data_trc_W.LAJC_Y*toMeters;   LANKLE_z_W=data_trc_W.LAJC_Z*toMeters;   

RKNEE_y_W=data_trc_W.RKJC_Y*toMeters;   RKNEE_z_W=data_trc_W.RKJC_Z*toMeters;    
LKNEE_y_W=data_trc_W.LKJC_Y*toMeters;   LKNEE_z_W=data_trc_W.LKJC_Z*toMeters;   

RHIP_y_W=data_trc_W.RHJC_Y*toMeters;   RHIP_z_W=data_trc_W.RHJC_Z*toMeters;    
LHIP_y_W=data_trc_W.LHJC_Y*toMeters;   LHIP_z_W=data_trc_W.LHJC_Z*toMeters;  

PELO_y_W=data_trc_W.PELO_Y*toMeters;   PELO_z_W=data_trc_W.PELO_Z*toMeters;    
PELP_y_W=data_trc_W.PELP_Y*toMeters;   PELP_z_W=data_trc_W.PELP_Z*toMeters;   

TRXO_y_W=data_trc_W.TRXO_Y*toMeters;   TRXO_z_W=data_trc_W.TRXO_Z*toMeters;    
TRXP_y_W=data_trc_W.TRXP_Y*toMeters;   TRXP_z_W=data_trc_W.TRXP_Z*toMeters;  

FP1_force_y_W=data_grf_s_W.FP1_Force_Y;            FP1_force_z_W=data_grf_s_W.FP1_Force_Z;
FP1_COP_y_W  =(600 - data_grf_s_W.FP1_COP_Y)*toMeters;     FP1_COP_z_W  =data_grf_s_W.FP1_COP_Z*toMeters;

FP2_force_y_W=data_grf_s_W.FP2_Force_Y;            FP2_force_z_W=data_grf_s_W.FP2_Force_Z;
FP2_COP_y_W=(-600 - data_grf_s_W.FP2_COP_Y)*toMeters;       FP2_COP_z_W=data_grf_s_W.FP2_COP_Z*toMeters;

Frame_W = data_trc_W.Frame;

%%PHASE 2: Kinematics - Joint Angles
% WALKING
% Trunk absolute angle (relative to vertical)
trunk_dy_W = TRXO_y_W - TRXP_y_W;
trunk_dz_W = TRXO_z_W - TRXP_z_W;
trunk_angle_W = atan2(trunk_dy_W, trunk_dz_W); %it's in radiant!
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
pelvis_dy_W = PELP_y_W - PELO_y_W;
pelvis_dz_W = PELP_z_W - PELO_z_W;
pelvis_angle_W = atan2(pelvis_dy_W, pelvis_dz_W);
pelvis_angle_W_deg = rad2deg(pelvis_angle_W);
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
thigh_R_dy_W = RHIP_y_W - RKNEE_y_W; % hip minus knee = points upward
thigh_R_dz_W = RHIP_z_W - RKNEE_z_W;
thigh_R_abs_W = atan2(thigh_R_dy_W, thigh_R_dz_W);
hip_R_W = rad2deg(thigh_R_abs_W - pelvis_angle_W);
% >0 = hip flexion (thigh forward of pelvis)
% <0 = hip extension (thigh behind pelvis)
% Left hip: same but use LHIP and LKNEE
thigh_L_dy_W = LHIP_y_W - LKNEE_y_W;
thigh_L_dz_W = LHIP_z_W - LKNEE_z_W;
thigh_L_abs_W = atan2(thigh_L_dy_W, thigh_L_dz_W);
hip_L_W = rad2deg(thigh_L_abs_W - pelvis_angle_W);
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
shank_R_dy_W = RKNEE_y_W - RANKLE_y_W; % knee minus ankle = points upward
shank_R_dz_W = RKNEE_z_W - RANKLE_z_W;
shank_R_abs_W = atan2(shank_R_dy_W, shank_R_dz_W);
knee_R_W = rad2deg(shank_R_abs_W - thigh_R_abs_W);
% >0 = knee flexion; <0 = hyperextension (rare, ~0 at full extension)
shank_L_dy_W = LKNEE_y_W - LANKLE_y_W; % knee minus ankle = points upward
shank_L_dz_W = LKNEE_z_W - LANKLE_z_W;
shank_L_abs_W = atan2(shank_L_dy_W, shank_L_dz_W);
knee_L_W = rad2deg(shank_L_abs_W - thigh_L_abs_W);
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
foot_R_dy_W = RTOE_y_W - RANKLE_y_W; % toe minus ankle = foot vector
foot_R_dz_W = RTOE_z_W - RANKLE_z_W;
foot_R_abs_W = atan2(foot_R_dy_W, foot_R_dz_W);
ankle_R_W = rad2deg(foot_R_abs_W - shank_R_abs_W) -90 + 5;
% +90 → makes anatomically neutral = 0°
% +5 → anatomical correction (project-specified)
% >0 = dorsiflexion; <0 = plantarflexion
foot_L_dy_W = LTOE_y_W - LANKLE_y_W; % toe minus ankle = foot vector
foot_L_dz_W = LTOE_z_W - LANKLE_z_W;
foot_L_abs_W = atan2(foot_L_dy_W, foot_L_dz_W);
ankle_L_W = rad2deg(foot_L_abs_W - shank_L_abs_W) - 90 + 5;
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

%% Read CROUCH marker trajectory and ground reaction data
data_trc_C = readtable(fullfile(file_dir,'Crouch.csv'));
data_grf_C = readtable(fullfile(file_dir,'Crouch_FP.csv'));
data_grf_s_C = data_grf_C(1:10:end,:);

RTOE_y_C=data_trc_C.RTOO_Y*toMeters;   RTOE_z_C=data_trc_C.RTOO_Z*toMeters;
RANKLE_y_C=data_trc_C.RAJC_Y*toMeters; RANKLE_z_C=data_trc_C.RAJC_Z*toMeters;
RKNEE_y_C=data_trc_C.RKJC_Y*toMeters;  RKNEE_z_C=data_trc_C.RKJC_Z*toMeters;
RHIP_y_C=data_trc_C.RHJC_Y*toMeters;   RHIP_z_C=data_trc_C.RHJC_Z*toMeters;
PELO_y_C=data_trc_C.PELO_Y*toMeters;   PELO_z_C=data_trc_C.PELO_Z*toMeters;
PELP_y_C=data_trc_C.PELP_Y*toMeters;   PELP_z_C=data_trc_C.PELP_Z*toMeters;
TRXO_y_C=data_trc_C.TRXO_Y*toMeters;   TRXO_z_C=data_trc_C.TRXO_Z*toMeters;
TRXP_y_C=data_trc_C.TRXP_Y*toMeters;   TRXP_z_C=data_trc_C.TRXP_Z*toMeters;

FP2_force_y_C=data_grf_s_C.FP2_Force_Y;  FP2_force_z_C=data_grf_s_C.FP2_Force_Z;
FP2_COP_y_C=(-600 - data_grf_s_C.FP2_COP_Y)*toMeters;  FP2_COP_z_C=data_grf_s_C.FP2_COP_Z*toMeters;

Frame_C = data_trc_C.Frame;

% ===== CROUCH: joint angles (right side) =====
trunk_dy_C = TRXO_y_C - TRXP_y_C;
trunk_dz_C = TRXO_z_C - TRXP_z_C;
trunk_angle_C = atan2(-trunk_dy_C, trunk_dz_C);          % rad
trunk_angle_C_deg = rad2deg(trunk_angle_C);

pelvis_dy_C = PELP_y_C - PELO_y_C;
pelvis_dz_C = PELP_z_C - PELO_z_C;
pelvis_angle_C = atan2(-pelvis_dy_C, pelvis_dz_C);       % rad
pelvis_angle_C_deg = rad2deg(pelvis_angle_C);

thigh_R_dy_C = RHIP_y_C - RKNEE_y_C;
thigh_R_dz_C = RHIP_z_C - RKNEE_z_C;
thigh_R_abs_C = atan2(-thigh_R_dy_C, thigh_R_dz_C);
hip_R_C = rad2deg(thigh_R_abs_C - pelvis_angle_C);

shank_R_dy_C = RKNEE_y_C - RANKLE_y_C;
shank_R_dz_C = RKNEE_z_C - RANKLE_z_C;
shank_R_abs_C = atan2(-shank_R_dy_C, shank_R_dz_C);
knee_R_C = rad2deg(shank_R_abs_C - thigh_R_abs_C);

foot_R_dy_C = RTOE_y_C - RANKLE_y_C;
foot_R_dz_C = RTOE_z_C - RANKLE_z_C;
foot_R_abs_C = atan2(-foot_R_dy_C, foot_R_dz_C);
ankle_R_C = rad2deg(foot_R_abs_C - shank_R_abs_C) - 90 + 5;


% ---------- Walking, right (124–212) ----------
rows_W_R   = find(Frame_W == W_R_FS1) : find(Frame_W == W_R_FS2);
n_W_R      = length(rows_W_R);% 89
gc_W_R     = linspace(0, 100, n_W_R);
TO_pct_W_R = (W_R_TO - W_R_FS1) / (W_R_FS2 - W_R_FS1) * 100;

trunk_W_R_gc  = trunk_angle_W_deg(rows_W_R);
pelvis_W_R_gc = pelvis_angle_W_deg(rows_W_R);
hip_R_W_gc    = hip_R_W(rows_W_R);
knee_R_W_gc   = knee_R_W(rows_W_R);
ankle_R_W_gc  = ankle_R_W(rows_W_R);

% ---------- Walking, left (168–257) ----------
rows_W_L   = find(Frame_W == W_L_FS1) : find(Frame_W == W_L_FS2);
n_W_L      = length(rows_W_L);% 90
gc_W_L     = linspace(0, 100, n_W_L);
TO_pct_W_L = (W_L_TO - W_L_FS1) / (W_L_FS2 - W_L_FS1) * 100;
trunk_W_L_gc  = trunk_angle_W_deg(rows_W_L);
pelvis_W_L_gc = pelvis_angle_W_deg(rows_W_L);
hip_L_W_gc    = hip_L_W(rows_W_L);
knee_L_W_gc   = knee_L_W(rows_W_L);
ankle_L_W_gc  = ankle_L_W(rows_W_L);

% ---------- Crouch, right (190–288) ----------
rows_C_R   = find(Frame_C == C_R_FS1) : find(Frame_C == C_R_FS2);
n_C_R      = length(rows_C_R);                 % 99
gc_C_R     = linspace(0, 100, n_C_R);
TO_pct_C_R = (C_R_TO - C_R_FS1) / (C_R_FS2 - C_R_FS1) * 100;

trunk_C_R_gc  = trunk_angle_C_deg(rows_C_R);
pelvis_C_R_gc = pelvis_angle_C_deg(rows_C_R);
hip_R_C_gc    = hip_R_C(rows_C_R);
knee_R_C_gc   = knee_R_C(rows_C_R);
ankle_R_C_gc  = ankle_R_C(rows_C_R);
% ---------- check ----------
fprintf('W_R: frames %d–%d, n = %d\n', Frame_W(rows_W_R(1)), Frame_W(rows_W_R(end)), n_W_R);
fprintf('W_L: frames %d–%d, n = %d\n', Frame_W(rows_W_L(1)), Frame_W(rows_W_L(end)), n_W_L);
fprintf('C_R: frames %d–%d, n = %d\n', Frame_C(rows_C_R(1)), Frame_C(rows_C_R(end)), n_C_R);


%% ===== Figure 1: Normal walking, left vs right, joint angles =====
% Panel order follows the report template: trunk, pelvis, hip, knee, ankle
titles1 = {'Trunk', 'Pelvis', 'Hip', 'Knee', 'Ankle'};
R1 = {trunk_W_R_gc, pelvis_W_R_gc, hip_R_W_gc, knee_R_W_gc, ankle_R_W_gc};
L1 = {trunk_W_L_gc, pelvis_W_L_gc, hip_L_W_gc, knee_L_W_gc, ankle_L_W_gc};
posLbl = {'Anterior', 'Anterior', 'Flexion',   'Flexion',   'Dorsiflexion'};
negLbl = {'Posterior','Posterior','Extension', 'Extension', 'Plantarflexion'};

fig1 = figure('Color','w', 'Units','centimeters', 'Position',[2 2 18 11]);
tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');

for k = 1:5
    nexttile; hold on; box on
    hR = plot(gc_W_R, R1{k}, 'b-',  'LineWidth', 1.5);
    hL = plot(gc_W_L, L1{k}, 'r--', 'LineWidth', 1.5);
    xline(TO_pct_W_R, 'b:', 'LineWidth', 1);     % right toe-off
    xline(TO_pct_W_L, 'r:', 'LineWidth', 1);     % left toe-off
    yline(0, 'k-', 'Color', [0.6 0.6 0.6]);      % zero reference

    xlim([0 100])
    title(titles1{k})
    xlabel('Gait cycle (%)')
    ylabel('Angle (deg)')

    % sign convention next to the y-axis
    text(0.02, 0.95, ['\uparrow ' posLbl{k}],   'Units','normalized', 'FontSize', 8)
    text(0.02, 0.05, ['\downarrow ' negLbl{k}], 'Units','normalized', 'FontSize', 8)
end

% legend in the empty 6th tile (invisible copies of the two line styles)
nexttile(6); hold on; axis off
plot(NaN, NaN, 'b-',  'LineWidth', 1.5);
plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
legend({'Right', 'Left'}, 'Location', 'north', 'FontSize', 10);
text(0.5, 0.45, {'Dotted lines: toe-off', ...
    sprintf('R %.1f %%,  L %.1f %%', TO_pct_W_R, TO_pct_W_L)}, ...
    'Units','normalized', 'HorizontalAlignment','center', 'FontSize', 9)

sgtitle('Figure 1. Normal walking: sagittal-plane angles, left vs right')
fontsize(fig1, 10, 'points')

exportgraphics(fig1, 'Fig1_walking_angles.png', 'Resolution', 300)


%% ===== Figure 3: Normal walking vs crouch walking, right side, joint angles =====
R3_W = {trunk_W_R_gc, pelvis_W_R_gc, hip_R_W_gc, knee_R_W_gc, ankle_R_W_gc};
R3_C = {trunk_C_R_gc, pelvis_C_R_gc, hip_R_C_gc, knee_R_C_gc, ankle_R_C_gc};

fig3 = figure('Color','w', 'Units','centimeters', 'Position',[2 2 18 11]);
tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');

for k = 1:5
    nexttile; hold on; box on
    hW = plot(gc_W_R, R3_W{k}, 'b-', 'LineWidth', 1.5);
    hC = plot(gc_C_R, R3_C{k}, 'Color', [0.85 0.4 0], 'LineStyle', '--', 'LineWidth', 1.5);
    xline(TO_pct_W_R, 'b:', 'LineWidth', 1);                    % walking toe-off
    xline(TO_pct_C_R, ':', 'Color', [0.85 0.4 0], 'LineWidth', 1); % crouch toe-off
    yline(0, 'k-', 'Color', [0.6 0.6 0.6]);

    xlim([0 100])
    title(titles1{k})
    xlabel('Gait cycle (%)')
    ylabel('Angle (deg)')

    text(0.02, 0.95, ['\uparrow ' posLbl{k}],   'Units','normalized', 'FontSize', 8)
    text(0.02, 0.05, ['\downarrow ' negLbl{k}], 'Units','normalized', 'FontSize', 8)
end

% legend in the empty 6th tile (invisible copies of the two line styles)
nexttile(6); hold on; axis off
plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
plot(NaN, NaN, 'Color', [0.85 0.4 0], 'LineStyle', '--', 'LineWidth', 1.5);
legend({'Normal walking', 'Crouch walking'}, 'Location', 'north', 'FontSize', 10);
text(0.5, 0.35, {'Dotted lines: toe-off', ...
    sprintf('Walking %.1f %%,  Crouch %.1f %%', TO_pct_W_R, TO_pct_C_R)}, ...
    'Units','normalized', 'HorizontalAlignment','center', 'FontSize', 9)

sgtitle('Figure 3. Normal vs crouch walking (right side): sagittal-plane angles')
fontsize(fig3, 10, 'points')

exportgraphics(fig3, 'Fig3_walking_vs_crouch_angles.png', 'Resolution', 300)

%% PHASE 3 — Segment Geometry, CoM, Accelerations, Inertia
% ─────────────────────────────────────────────────────────
% 3.1 Segment lengths (frame-varying arrays)
L_foot_R_W  = hypot(RTOE_y_W  - RANKLE_y_W, RTOE_z_W  - RANKLE_z_W);
L_foot_L_W  = hypot(LTOE_y_W  - LANKLE_y_W, LTOE_z_W  - LANKLE_z_W);
L_shank_R_W = hypot(RKNEE_y_W - RANKLE_y_W, RKNEE_z_W - RANKLE_z_W);
L_shank_L_W = hypot(LKNEE_y_W - LANKLE_y_W, LKNEE_z_W - LANKLE_z_W);
L_thigh_R_W = hypot(RHIP_y_W  - RKNEE_y_W,  RHIP_z_W  - RKNEE_z_W);
L_thigh_L_W = hypot(LHIP_y_W  - LKNEE_y_W,  LHIP_z_W  - LKNEE_z_W);

L_foot_R_C  = hypot(RTOE_y_C  - RANKLE_y_C, RTOE_z_C  - RANKLE_z_C);
L_shank_R_C = hypot(RKNEE_y_C - RANKLE_y_C, RKNEE_z_C - RANKLE_z_C);
L_thigh_R_C = hypot(RHIP_y_C  - RKNEE_y_C,  RHIP_z_C  - RKNEE_z_C);

% ─────────────────────────────────────────────────────────
% 3.2 Segment CoM positions (frame-varying)
% Walking — right
com_foot_R_y_W  = RANKLE_y_W + com_foot_frac  * (RTOE_y_W   - RANKLE_y_W);
com_foot_R_z_W  = RANKLE_z_W + com_foot_frac  * (RTOE_z_W   - RANKLE_z_W);
com_shank_R_y_W = RKNEE_y_W  + com_shank_frac * (RANKLE_y_W - RKNEE_y_W);
com_shank_R_z_W = RKNEE_z_W  + com_shank_frac * (RANKLE_z_W - RKNEE_z_W);
com_thigh_R_y_W = RHIP_y_W   + com_thigh_frac * (RKNEE_y_W  - RHIP_y_W);
com_thigh_R_z_W = RHIP_z_W   + com_thigh_frac * (RKNEE_z_W  - RHIP_z_W);

% Walking — left
com_foot_L_y_W  = LANKLE_y_W + com_foot_frac  * (LTOE_y_W   - LANKLE_y_W);
com_foot_L_z_W  = LANKLE_z_W + com_foot_frac  * (LTOE_z_W   - LANKLE_z_W);
com_shank_L_y_W = LKNEE_y_W  + com_shank_frac * (LANKLE_y_W - LKNEE_y_W);
com_shank_L_z_W = LKNEE_z_W  + com_shank_frac * (LANKLE_z_W - LKNEE_z_W);
com_thigh_L_y_W = LHIP_y_W   + com_thigh_frac * (LKNEE_y_W  - LHIP_y_W);
com_thigh_L_z_W = LHIP_z_W   + com_thigh_frac * (LKNEE_z_W  - LHIP_z_W);

% Crouch — right
com_foot_R_y_C  = RANKLE_y_C + com_foot_frac  * (RTOE_y_C   - RANKLE_y_C);
com_foot_R_z_C  = RANKLE_z_C + com_foot_frac  * (RTOE_z_C   - RANKLE_z_C);
com_shank_R_y_C = RKNEE_y_C  + com_shank_frac * (RANKLE_y_C - RKNEE_y_C);
com_shank_R_z_C = RKNEE_z_C  + com_shank_frac * (RANKLE_z_C - RKNEE_z_C);
com_thigh_R_y_C = RHIP_y_C   + com_thigh_frac * (RKNEE_y_C  - RHIP_y_C);
com_thigh_R_z_C = RHIP_z_C   + com_thigh_frac * (RKNEE_z_C  - RHIP_z_C);

% ─────────────────────────────────────────────────────────
%  3.3  CoM linear accelerations  (gradient preserves length)
% Walking right
a_foot_R_y_W  = gradient(gradient(com_foot_R_y_W,  dt), dt);
a_foot_R_z_W  = gradient(gradient(com_foot_R_z_W,  dt), dt);
a_shank_R_y_W = gradient(gradient(com_shank_R_y_W, dt), dt);
a_shank_R_z_W = gradient(gradient(com_shank_R_z_W, dt), dt);
a_thigh_R_y_W = gradient(gradient(com_thigh_R_y_W, dt), dt);
a_thigh_R_z_W = gradient(gradient(com_thigh_R_z_W, dt), dt);

% Walking left
a_foot_L_y_W  = gradient(gradient(com_foot_L_y_W,  dt), dt);
a_foot_L_z_W  = gradient(gradient(com_foot_L_z_W,  dt), dt);
a_shank_L_y_W = gradient(gradient(com_shank_L_y_W, dt), dt);
a_shank_L_z_W = gradient(gradient(com_shank_L_z_W, dt), dt);
a_thigh_L_y_W = gradient(gradient(com_thigh_L_y_W, dt), dt);
a_thigh_L_z_W = gradient(gradient(com_thigh_L_z_W, dt), dt);

% Crouch right
a_foot_R_y_C  = gradient(gradient(com_foot_R_y_C,  dt), dt);
a_foot_R_z_C  = gradient(gradient(com_foot_R_z_C,  dt), dt);
a_shank_R_y_C = gradient(gradient(com_shank_R_y_C, dt), dt);
a_shank_R_z_C = gradient(gradient(com_shank_R_z_C, dt), dt);
a_thigh_R_y_C = gradient(gradient(com_thigh_R_y_C, dt), dt);
a_thigh_R_z_C = gradient(gradient(com_thigh_R_z_C, dt), dt);

% ─────────────────────────────────────────────────────────
%  3.4  Segment angular accelerations
alpha_foot_R_W  = gradient(gradient(foot_R_abs_W,  dt), dt);
alpha_shank_R_W = gradient(gradient(shank_R_abs_W, dt), dt);
alpha_thigh_R_W = gradient(gradient(thigh_R_abs_W, dt), dt);

alpha_foot_L_W  = gradient(gradient(foot_L_abs_W,  dt), dt);
alpha_shank_L_W = gradient(gradient(shank_L_abs_W, dt), dt);
alpha_thigh_L_W = gradient(gradient(thigh_L_abs_W, dt), dt);

alpha_foot_R_C  = gradient(gradient(foot_R_abs_C,  dt), dt);
alpha_shank_R_C = gradient(gradient(shank_R_abs_C, dt), dt);
alpha_thigh_R_C = gradient(gradient(thigh_R_abs_C, dt), dt);

% ─────────────────────────────────────────────────────────
%  3.5  Moments of inertia  I = m*(rho*L_mean)^2  [scalar per segment]
% Radii of gyration (Winter 2009)
rk_foot  = 0.475;% k is the radius of gyration and this number is the ratio of k/segment's length
rk_shank = 0.302;
rk_thigh = 0.323;

I_foot_R_W  = m_foot  * (rk_foot  * mean(L_foot_R_W,  'omitnan'))^2;
I_foot_L_W  = m_foot  * (rk_foot  * mean(L_foot_L_W,  'omitnan'))^2;
I_shank_R_W = m_shank * (rk_shank * mean(L_shank_R_W, 'omitnan'))^2;
I_shank_L_W = m_shank * (rk_shank * mean(L_shank_L_W, 'omitnan'))^2;
I_thigh_R_W = m_thigh * (rk_thigh * mean(L_thigh_R_W, 'omitnan'))^2;
I_thigh_L_W = m_thigh * (rk_thigh * mean(L_thigh_L_W, 'omitnan'))^2;

I_foot_R_C  = m_foot  * (rk_foot  * mean(L_foot_R_C,  'omitnan'))^2;
I_shank_R_C = m_shank * (rk_shank * mean(L_shank_R_C, 'omitnan'))^2;
I_thigh_R_C = m_thigh * (rk_thigh * mean(L_thigh_R_C, 'omitnan'))^2;