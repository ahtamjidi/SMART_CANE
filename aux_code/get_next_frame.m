function [current_v_, current_w_] = get_next_frame()
persistent current_step_in_source  next_step_in_source
persistent current_v current_w next_v next_w
persistent threshold_dist threshold_orient
persistent time_stamp
global myCONFIG step_global


source_folder = myCONFIG.PATH.SOURCE_FOLDER; 
destination_folder = myCONFIG.PATH.KEYFRAMES_FOLDER;
time_elapsed = 0;
dq_aggregate = [1;0;0;0];

if isempty(current_step_in_source)
    current_step_in_source = step_global - 1;
    %     next_step_in_source = 2;
    current_v = [0;0;0]; %% m/s
    current_w = [0;0;0];  %% rad/s
    threshold_dist = 0.06; %% m
    threshold_orient = 7*pi/180; %% rad
    load([myCONFIG.PATH.SOURCE_FOLDER,'/TimeStamp/TimeStamp.mat'],'time_stamp');
    [current_v,current_w,dq_aggregate,time_elapsed] = update_velocity_( current_step_in_source +1,...
                                                         time_stamp,time_elapsed,...
                                                         current_v,current_w,...
                                                         dq_aggregate);
                                                     
    source_data_file = sprintf('%s/d1_%04d.dat',source_folder,step_global-1);
    destination_data_file = sprintf('%s/d1_%04d.dat',destination_folder,step_global-1);
    copyfile(source_data_file,destination_data_file)
 
    destination_velocity_file = sprintf('%s/vel_%04d.dat',[destination_folder,'VelocityData'],step_global-1);
    save(destination_velocity_file,'current_v','current_w','dq_aggregate','time_elapsed');
                                                     
                                                     
                                                     
end
flag_next_found =0;
next_step_in_source = current_step_in_source +1;
predicted_dist = 0;
predicted_orinetation_shift = 0;

while ~flag_next_found
    %% Dr Ye's scheme
    %%% 1- see if the current frame will be used based on the velocities and
    %%% other thresholds
    time_difference = (time_stamp(1,next_step_in_source) - time_stamp(1,next_step_in_source-1))*0.001; %% second
%     time_elapsed = time_elapsed + time_difference;
    predicted_dist = predicted_dist + current_v * time_difference;
    predicted_orinetation_shift = predicted_orinetation_shift + q2e(v2q(current_w * time_difference));
    if (norm(predicted_dist)<threshold_dist) && (norm(predicted_orinetation_shift) < threshold_orient)
        [current_v,current_w,dq_aggregate,time_elapsed] = update_velocity_(next_step_in_source ,...
                                                 time_stamp,time_elapsed,...
                                                 current_v,current_w,...
                                                 dq_aggregate);
        next_step_in_source = next_step_in_source + 1
        continue
    end
    current_step_in_source = next_step_in_source - 1
    flag_next_found =1;
    %%%%%%%% COPY THE XYZ_DATA AND SIFT_FEATURE FILES FORM SOURCE
    %%%%%%%% TO DESTINATION
%     xyz_file_in_source = sprintf('%s/xyz_data/xyz_%04d.mat',...
%         source_folder,next_step_in_source);
%     SIFT_features_file_in_source = sprintf('%s/FeatureExtractionMatching/SIFT_result%04d.mat',...
%         source_folder,next_step_in_source);
    
%     xyz_file_in_destination = sprintf('%s/xyz_data/xyz%04d.mat',...
%         destination_folder,step_global);
%     SIFT_features_file_in_destination = sprintf('%s/FeatureExtractionMatching/SIFT_result%04d.mat',...
%         destination_folder,step_global);
    
%     copyfile(xyz_file_in_source,xyz_file_in_destination)
%     copyfile(SIFT_features_file_in_source,SIFT_features_file_in_destination)
 
    %%%%%%%% COPY RANSAC FILE TO DESTINATINO AND DELETE IT IN THE
    %%%%%%%% SOURCE
%     RANSAC_file_in_source = sprintf('%s/RANSAC_pose_shift_dr_Ye/RANSAC_RESULT_%d_%d.mat',source_folder,idx_key_frame_in_source,i);
%     RANSAC_file_in_destination = sprintf('%s/RANSAC_pose_shift_dr_Ye/RANSAC_RESULT_%d_%d.mat',...
%         destination_folder,step_global-1,step_global);
%     copyfile(RANSAC_file_in_source,RANSAC_file_in_destination)
%     
%     save(RANSAC_file_in_destination,'idx_key_frame_in_source','prev_key_frame_source','-append')
%     delete(RANSAC_file_in_source)
%     
    
    %%%%%%%% COPY DATA FILE
    source_data_file = sprintf('%s/d1_%04d.dat',source_folder,next_step_in_source);
    destination_data_file = sprintf('%s/d1_%04d.dat',destination_folder,step_global);
    copyfile(source_data_file,destination_data_file)
 
    destination_velocity_file = sprintf('%s/vel_%04d.dat',[destination_folder,'VelocityData'],step_global);
    save(destination_velocity_file,'current_v','current_w','dq_aggregate','time_elapsed');

    %     [dX_gt,dq_calc,R_RANSAC,State_RANSAC,RANSAC_STAT]=Calculate_V_Omega_RANSAC_dr_ye(idx_key_frame_in_source,i);
    %%% 2- if we have to drop this frame update (v,w)
    %%% 3-
    
    %% my Scheme
end
end
function [current_v,current_w,dq_aggregate,time_elapsed] = update_velocity_(next_step_in_source,...
                                                           time_stamp,time_elapsed,...
                                                           current_v,current_w,...
                                                           dq_aggregate)
pre_step = next_step_in_source-1 ;
current_step = next_step_in_source;
valid_RANSAC =0;
while ~valid_RANSAC
    [dX_gt,dq_calc,R,State_RANSAC,RANSAC_STAT]=Calculate_V_Omega_RANSAC_dr_ye(pre_step,current_step);
    if State_RANSAC==1
        valid_RANSAC =1;
    else
        pre_step = pre_step -1;
    end
    if pre_step < next_step_in_source - 4
        disp('BAD FRAME FOR RANSAC')
        break
    end
end

if norm(dX_gt)>0.01
time_difference = (time_stamp(1,current_step) - time_stamp(1,pre_step))*0.001;
dq_aggregate = qprod(dq_aggregate,dq_calc);
current_v = (current_v*time_elapsed+dX_gt)/(time_difference+time_elapsed); %% m/s average velocity
[a_,u_] = q2au(dq_aggregate);
a_dt = a_/(time_difference + time_elapsed);
current_w = a_dt*u_; %% rotation vector rate
time_elapsed = time_difference+time_elapsed;
end
end

%
% % idxRange = [1 100];
% % idxRange = [1 data_file_counting(SOURCE_FOLDER,'d1')];
% % fig_key_frames = figure;
%
% %% initialize state vector
% % H = [1 0 0 0;
% %     0 1 0 0;
% %     0 0 1 0;
% %     0 0 0 1];
% % xx =zeros(7,100);
% % figure_debug =figure();
% % xlims=[0,0];
% % ylims=[0,0];
% % zlims=[0,0];
%
%
%
% % mycolor=[0 0 0;1 0 0;0 1 0;0 0 1]; %%% color map for the bars
%
% % mycolor(1) : black
% % mycolor(2) : red
% % mycolor(3) : green
% % mycolor(4) : blue
%
%
% nDataFiles = data_file_counting(SOURCE_FOLDER,'d1'); %% number of data files in the directory
% % index_color = zeros(1,nDataFiles); %%% color map index for indivudual data
% STACKED_nFeatures1 = zeros(1,nDataFiles); %%%
% STACKED_nF1_Confidence_Filtered = zeros(1,nDataFiles); %%%
% STACKED_nFeatures2 =zeros(1,nDataFiles); %%%
% STACKED_nF2_Confidence_Filtered = zeros(1,nDataFiles); %%%
% STACKED_nMatches = zeros(1,nDataFiles); %%%
% STACKED_nIterationRansac = zeros(1,nDataFiles); %%%
% STACKED_InlierRatio = zeros(1,nDataFiles); %%%
% STACKED_nSupport = zeros(1,nDataFiles); %%%
% STACKED_ErrorMean = zeros(1,nDataFiles); %%%
% STACKED_ErrorStd = zeros(1,nDataFiles); %%%
% STACKED_T_RANSAC = zeros(3,nDataFiles); %%%
% STACKED_q_RANSAC = zeros(4,nDataFiles); %%%
% STACKED_RANSAC_STATE = zeros(1,nDataFiles);
%
%
%
%
%
%
%
%
%
% for i=idxRange(1):idxRange(2)
%     mon_pos = get(0,'monitorPositions');
%     width = mon_pos(3);
%     height = mon_pos(4);
%     stopping_window_size = mon_pos(3:4)/100;
%     mouse_pos = get(0,'PointerLocation');
%     if all(mouse_pos>=mon_pos(3:4)-stopping_window_size)
%         keyboard
%     end
%
%     %% special treatment of the first index
%     if i==idxRange(1)  %%% for first step initialize the idx_key_frame_in_source and idx_key_frame_in_destination
%         %%% (the first one is the index of the keyframe in the source directory and the second one is the index in the second directory)
%         %%% this code copies the relevant data of the keyframe to the
%         %%% KeyFrame folder and renames the file name such that key frames
%         %%% have consecutive indices
%         idx_key_frame_in_source = idxRange(1);
%         idx_key_frame_in_destination = 1;
%         source_data_file = sprintf('%s/d1_%04d.dat',source_folder,i);
%         destination_data_file = sprintf('%s/d1_%04d.dat',destination_folder,idx_key_frame_in_destination);
%         copyfile(source_data_file,destination_data_file)
%         %%%%%%%% COPY THE XYZ_DATA AND SIFT_FEATURE FILES FORM SOURCE
%         %%%%%%%% TO DESTINATION
%         %         xyz_file_in_source = sprintf('%s/xyz_data/xyz_%04d.mat',...
%         %             source_folder,idx_key_frame_in_source);
%         %         SIFT_features_file_in_source = sprintf('%s/FeatureExtractionMatching/SIFT_result%04d.mat',...
%         %             source_folder,idx_key_frame_in_source);
%
%         %         xyz_file_in_destination = sprintf('%s/xyz_data/xyz_%04d.mat',...
%         %             destination_folder,idx_key_frame_in_destination);
%         %         SIFT_features_file_in_destination = sprintf('%s/FeatureExtractionMatching/SIFT_result%04d.mat',...
%         %             destination_folder,idx_key_frame_in_destination);
%         %
%         [dX_gt,dq_calc,R,State_RANSAC,RANSAC_STAT]=Calculate_V_Omega_RANSAC_dr_ye(idx_key_frame_in_source,idx_key_frame_in_source);
%         STACKED_nFeatures1(i) = RANSAC_STAT.nFeatures1; %%%
%         STACKED_nF1_Confidence_Filtered(i+1) = RANSAC_STAT.nF1_Confidence_Filtered; %%%
%         STACKED_nFeatures2(i) = RANSAC_STAT.nFeatures2; %%%
%         STACKED_nF2_Confidence_Filtered(i) = RANSAC_STAT.nF2_Confidence_Filtered; %%%
%         STACKED_nMatches(i) = RANSAC_STAT.nMatches; %%%
%         STACKED_nIterationRansac(i) = RANSAC_STAT.nIterationRansac; %%%
%         STACKED_InlierRatio(i) = RANSAC_STAT.InlierRatio; %%%
%         STACKED_nSupport(i) = RANSAC_STAT.nSupport; %%%
%         STACKED_T_RANSAC(:,i) = dX_gt;
%         STACKED_q_RANSAC(:,i) = dq_calc;
%         STACKED_ErrorMean(:,i) = RANSAC_STAT.ErrorMean;
%         STACKED_ErrorStd(:,i) = RANSAC_STAT.ErrorStd;
%         STACKED_RANSAC_STATE(i) = RANSAC_STAT.SolutionState;
%         index_color(1,i) = 3; %%  green
%
%
%
%
%
%
%         %         State_RANSAC = RANSAC_CALC_SAVE_SR4000(idx_key_frame_in_source,idx_key_frame_in_source);
%
%         %         copyfile(xyz_file_in_source,xyz_file_in_destination)
%         %         copyfile(SIFT_features_file_in_source,SIFT_features_file_in_destination)
%         idx_key_frame_in_destination  = idx_key_frame_in_destination + 1;
%     else
%         %% calculate RANSAC
%         %%% (calculates the RANSAC result between the latest keyframe and the current keyframe)
%         if mod(i,3)==0
%             try
%                 [dX_gt,dq_calc,R_RANSAC,State_RANSAC,RANSAC_STAT]=Calculate_V_Omega_RANSAC_dr_ye(idx_key_frame_in_source,i);
%                 STACKED_nFeatures1(i) = RANSAC_STAT.nFeatures1; %%%
%                 STACKED_nF1_Confidence_Filtered(i) = RANSAC_STAT.nF1_Confidence_Filtered; %%%
%                 STACKED_nFeatures2(i) = RANSAC_STAT.nFeatures2; %%%
%                 STACKED_nF2_Confidence_Filtered(i) = RANSAC_STAT.nF2_Confidence_Filtered; %%%
%                 STACKED_nMatches(i) = RANSAC_STAT.nMatches; %%%
%                 STACKED_nIterationRansac(i) = RANSAC_STAT.nIterationRansac; %%%
%                 STACKED_InlierRatio(i) = RANSAC_STAT.InlierRatio; %%%
%                 STACKED_nSupport(i) = RANSAC_STAT.nSupport; %%%
%                 STACKED_T_RANSAC(:,i) = dX_gt;
%                 STACKED_q_RANSAC(:,i) = dq_calc;
%                 STACKED_ErrorMean(:,i) = RANSAC_STAT.ErrorMean;
%                 STACKED_ErrorStd(:,i) = RANSAC_STAT.ErrorStd;
%                 STACKED_RANSAC_STATE(i) = RANSAC_STAT.SolutionState;
%
%                 T_RANSAC = dX_gt;
%                 %             State_RANSAC = RANSAC_CALC_SAVE_SR4000(idx_key_frame_in_source,i);
%                 cprintf('-magenta',['steps : [',num2str(idx_key_frame_in_source),' ',num2str(i),']\n' ]);
%                 cprintf('magenta','-----------------------------------\n' )
%             catch
%                 cprintf('-red','RANSAC is not working\n'); %%% in case that RANSAC fail for any reason for example not enough number of features
%                 %             disp('RANSAC is not working') %%% in case that RANSAC fail for any reason for example not enough number of features
%                 cprintf('red','--------------------------------\n')
%                 cprintf('red','--------------------------------\n')
%                 continue
%             end
%             %% Load the results
%
%             RANSAC_file_in_source = sprintf('%s/RANSAC_pose_shift_dr_Ye/RANSAC_RESULT_%d_%d.mat',source_folder,idx_key_frame_in_source,i);
%             %         load(RANSAC_file_in_source)
%             %         %% copy the result into KeyFrame directory and remove it from source directory
%
%             %%% display the results
%             euler_rot = R2e(R_RANSAC);
%             [a_rot,u_rot]=q2au(R2q(R_RANSAC));
%             cprintf(['euler angle (deg)= [',num2str(180*euler_rot(1)/pi),...
%                 ' ',num2str(180*euler_rot(2)/pi),' ',...
%                 num2str(180*euler_rot(3)/pi),']\n '])
%
%             cprintf(['T (m) = [',num2str(T_RANSAC(1)),...
%                 ' ',num2str(T_RANSAC(2)),' ',...
%                 num2str(T_RANSAC(3)),']\n '])
%
%             cprintf(['norm T = ',num2str(norm(T_RANSAC)),']\n '])
%             cprintf(['norm Angle (deg) = ',num2str(180*a_rot(1)/pi),']\n '])
%
%             %         disp(['euler angle (deg) and norm T= [',num2str(180*euler_rot(1)/pi),...
%             %             ' ',num2str(180*euler_rot(2)/pi),' ',...
%             %             num2str(180*euler_rot(3)/pi),' T = ',num2str(norm(T_RANSAC)),'] '])
%             %         disp(['a_rot angle (deg) = [',num2str(180*a_rot(1)/pi),'] '])
%
%
%             %%% if the rotation and tranlation are not big enough or the RANSAC
%             %%% result is not acceptqable, discard the frame and delte the
%             %%% calculated RANSAC rfesults
%             %         combination_rot_trans =  180*a_rot/pi + norm(T_RANSAC)*100;
%
%             %% key Frame selection criteria
%             KeyFrameFlag = 1;
%
%
%             %% FOR GENERAL MOTINO
%             %             if 180*a_rot/pi>3
%             %                 KeyFrameFlag = 1;
%             %             elseif norm(T_RANSAC)*100>4
%             %                 KeyFrameFlag = 1;
%             %             end
%
%             %% FOR LINEAR MOTINO
%             %         if norm(T_RANSAC)*100>50
%             %             KeyFrameFlag = 1;
%             %         end
%             %         if i==idxRange(2)
%             %             KeyFrameFlag = 1;
%             %         end
%
%             % %% FOR ROTATION
%             %         if 180*a_rot/pi>9
%             %             KeyFrameFlag = 1;
%             %         end
%             %         if i==idxRange(2)
%             %             KeyFrameFlag = 1;
%             %         end
%
%
%
%
%
%
%             %         (a_rot<4*pi/180 && norm(T_RANSAC)<0.05)
%             if ~KeyFrameFlag || (State_RANSAC~=1 || abs(abs(det(R_RANSAC))-1)>0.1)
%                 %         disp('rotation exceeded 3 degees')
%                 disp(['data ',num2str(i),' is not acceptable\n'])
%                 delete(RANSAC_file_in_source);
%                 cprintf('red', '------------------------------------------\n');
%                 if State_RANSAC~=1
%                     index_color(1,i) = 2; %%% red
%                 else
%                     index_color(1,i) = 4; %%% blue
%                 end
%             else %%% otherwise copy the result into the keyframe folder and delete the source file
%                 index_color(1,i) = 3; %%% green
%                 prev_key_frame_source = idx_key_frame_in_source;
%                 idx_key_frame_in_source = i;
%
%                 %%%%%%%% COPY THE XYZ_DATA AND SIFT_FEATURE FILES FORM SOURCE
%                 %%%%%%%% TO DESTINATION
%                 %             xyz_file_in_source = sprintf('%s/xyz_data/xyz_%04d.mat',...
%                 %                 source_folder,idx_key_frame_in_source);
%                 %             SIFT_features_file_in_source = sprintf('%s/FeatureExtractionMatching/SIFT_result%04d.mat',...
%                 %                 source_folder,idx_key_frame_in_source);
%                 %
%                 %             xyz_file_in_destination = sprintf('%s/xyz_data/xyz%04d.mat',...
%                 %                 destination_folder,idx_key_frame_in_destination);
%                 %             SIFT_features_file_in_destination = sprintf('%s/FeatureExtractionMatching/SIFT_result%04d.mat',...
%                 %                 destination_folder,idx_key_frame_in_destination);
%                 %
%                 %             copyfile(xyz_file_in_source,xyz_file_in_destination)
%                 %             copyfile(SIFT_features_file_in_source,SIFT_features_file_in_destination)
%
%
%
%
%                 %%%%%%%% COPY RANSAC FILE TO DESTINATINO AND DELETE IT IN THE
%                 %%%%%%%% SOURCE
%                 RANSAC_file_in_destination = sprintf('%s/RANSAC_pose_shift_dr_Ye/RANSAC_RESULT_%d_%d.mat',...
%                     destination_folder,idx_key_frame_in_destination-1,idx_key_frame_in_destination);
%                 copyfile(RANSAC_file_in_source,RANSAC_file_in_destination)
%
%                 save(RANSAC_file_in_destination,'idx_key_frame_in_source','prev_key_frame_source','-append')
%                 delete(RANSAC_file_in_source)
%
%
%                 %%%%%%%% COPY DATA FILE
%                 source_data_file = sprintf('%s/d1_%04d.dat',source_folder,idx_key_frame_in_source);
%                 destination_data_file = sprintf('%s/d1_%04d.dat',destination_folder,idx_key_frame_in_destination);
%                 copyfile(source_data_file,destination_data_file)
%
%                 %%%%%%%% PLOT THE TWO MOST RECENT FRAMES
%                 figure(fig_key_frames)
%                 subplot(121)
%                 im1=read_image_sr4000(destination_folder,max(idx_key_frame_in_destination-1,1));
%                 imagesc(im1);colormap gray;
%                 title(['frmae index in source folder is: ',num2str(idx_key_frame_in_source)],'Color','b')
%
%                 subplot(122)
%
%                 im1=read_image_sr4000(destination_folder,idx_key_frame_in_destination);
%                 imagesc(im1);colormap gray;
%                 title(['frmae index in destination folder is: ',num2str(idx_key_frame_in_destination)],'Color','r')
%
%                 idx_key_frame_in_destination =idx_key_frame_in_destination+1;
%                 cprintf('blue', '------------------------------------------\n');
%
%                 %%%%  plot the dead reckoning result
%
%                 %             cprintf('-green',['steps : [',num2str(last_starting_idx),' ',num2str(i+1),']\n' ]);
%                 if State_RANSAC~=1
%                     dX_gt = [0;0;0];
%                     dq_calc = [1;0;0;0];
%                     idx_RANSAC_fail = [idx_RANSAC_fail,idx_key_frame_in_destination];
%                 end
%                 H = H*Pose2H([dX_gt;q2e(dq_calc)]);
%                 xx(:,idx_key_frame_in_destination) = [H(1:3,4);R2q(H(1:3,1:3))];
%                 figure(figure_debug)
%
%                 subplot(211)
%                 %     hold off
%
%                 %     draw_camera( [V*0;q' ], 'r' );
%                 %     hold on
%                 %     axis equal
%                 %     grid on
%                 %     grid minor
%                 % % %     draw_camera( [H(1:3,4)*0; R2q(H(1:3,1:3))], 'k' );
%                 % % %
%                 % % %     xlim(0.35*[-1 1])
%                 % % %     zlim(0.35*[-1 1])
%                 % % %     ylim(0.35*[-1 1])
%                 im = read_image_sr4000(myCONFIG.PATH.DATA_FOLDER,idx_key_frame_in_destination);
%                 imagesc(im);colormap gray;
%                 %     hold off
%                 subplot(212)
%                 xlims(1)=min([xlims(1),H(1,4)-0.3]);
%                 xlims(2)=max([xlims(2),H(1,4)+0.3]);
%
%                 ylims(1)=min([zlims(1),H(2,4)-0.3]);
%                 ylims(2)=max([zlims(2),H(2,4)+0.3]);
%
%                 zlims(1)=min([zlims(1),H(3,4)-0.3]);
%                 zlims(2)=max([zlims(2),H(3,4)+0.3]);
%                 plot3( xx(1, 1:idx_key_frame_in_destination), xx(2, 1:idx_key_frame_in_destination),...
%                     xx(3, 1:idx_key_frame_in_destination), 'k', 'LineWidth', 2 );
%                 grid on
%                 hold on
%                 draw_camera( [H(1:3,4); R2q(H(1:3,1:3))], 'r' );
%                 axis equal
%                 xlim(xlims)
%                 ylim(ylims)
%                 zlim(zlims)
%                 hold off
%                 hold off
%                 disp(['---',num2str(i),'--->\n'])
%             end
%         end
%     end
% end
% % for i=80:122
% %     destination_data_file = sprintf('%s/d1_%04d.dat',destination_folder,i);
% %     load(destination_data_file)
% %     clear myCONFIG
% %     data_from_mat_file = sprintf('d1_%04d',i);
% %     eval('clear data_from_mat_file')
% %     save(destination_data_file)
% % end
%
% RANSAC_RESULT = struct('nFeatures1',STACKED_nFeatures1,...
%     'nF1_Confidence_Filtered',STACKED_nF1_Confidence_Filtered,...
%     'nFeatures2',STACKED_nFeatures2,...
%     'nF2_Confidence_Filtered',STACKED_nF2_Confidence_Filtered,...
%     'nMatches',STACKED_nMatches,...
%     'nIterationRansac',STACKED_nIterationRansac,...
%     'InlierRatio',STACKED_InlierRatio,...
%     'nSupport',STACKED_nSupport,...
%     'T_RANSAC',STACKED_T_RANSAC,...
%     'q_RANSAC',STACKED_q_RANSAC,...
%     'index_color',index_color,...
%     'ErrorMean',STACKED_ErrorMean,...
%     'ErrorStd',STACKED_ErrorStd,...
%     'SolutionState',STACKED_RANSAC_STATE);
%
%
%
%
% %             STACKED_nFeatures1(i) = RANSAC_STAT.nFeatures1; %%%
% %             STACKED_nF1_Confidence_Filtered(i) = RANSAC_STAT.nF1_Confidence_Filtered; %%%
% %             STACKED_nFeatures2(i) = RANSAC_STAT.nFeatures2; %%%
% %             STACKED_nF2_Confidence_Filtered(i) = RANSAC_STAT.nF2_Confidence_Filtered; %%%
% %             STACKED_nMatches(i) = RANSAC_STAT.nMatches; %%%
% %             STACKED_nIterationRansac(i) = RANSAC_STAT.nIterationRansac; %%%
% %             STACKED_InlierRatio(i) = RANSAC_STAT.InlierRatio; %%%
% %             STACKED_nSupport(i) = RANSAC_STAT.nSupport; %%%
% %             STACKED_T_RANSAC(:,i) = dq_calc;
% %             STACKED_q_RANSAC(:,i) = dq_calc;
%
% elapsed_time = toc;
% disp(['ELAPSED TIME is : ',num2str(elapsed_time) ]);
% end
%
%
% % RANSAC_CALC_SAVE_SR4000
%
%
%
%
% end
%
%
