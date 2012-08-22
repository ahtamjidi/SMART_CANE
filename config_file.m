function config_file
global myCONFIG
persistent initialization_done
% persistent  extractor
% % % %      /home/axtamjidi/SR4000/SR4000/EKF_monoSLAM_1pRANSAC/matlab_code
% % % %      /home/axtamjidi/SR4000/eitlog
% % % %      /home/axtamjidi/SR4000/SR4000/SR_4000_data/lab13
if isempty(initialization_done)
    initialization_done = 0;
end

% /media/FreeAgentGoFlexDrive/SOONHAC/square_1000/KeyFrames
% myCONFIG.PATH.DATA_FOLDER = '/home/axtamjidi/SR4000/eitlog/';

% myCONFIG.PATH.DATA_FOLDER = '/home/axtamjidi/SR4000/square/';
myCONFIG.PATH.SOURCE_FOLDER = 'D:\august24\TestCode\'; %% exp3
myCONFIG.PATH.KEYFRAMES_FOLDER = [myCONFIG.PATH.SOURCE_FOLDER,'KeyFrames\']; %% exp3
myCONFIG.PATH.DATA_FOLDER = myCONFIG.PATH.KEYFRAMES_FOLDER; %% exp3

myCONFIG.PATH.AVI_OUTPUT = [myCONFIG.PATH.DATA_FOLDER,'/aviOutput/result11.avi'];
myCONFIG.IDX.idxCam = 1;
myCONFIG.STEP.START = 3;
myCONFIG.STEP.END = 400;data_file_counting(myCONFIG.PATH.SOURCE_FOLDER,'d1');
myCONFIG.FLAGS.GICP_CAM_ID = -1; %%% -1 means whole laser data is fed into GICP
myCONFIG.FLAGS.EST_METHOD = '1PRE'; %%% ( 1PRE = 1Point RANSAC | PURE_EKF = pure ekf. no feature management)
myCONFIG.FLAGS.MOTION_INPUT='RANSAC'; %%% 'GT' == ground truth -- 'RANSAC' == RANSAC
myCONFIG.FLAGS.DO_ANIM = 'no'; %%% yes means create avi file from image sequences
myCONFIG.FLAGS.VERBOSE = 'yes';
myCONFIG.FLAGS.ORIGINAL_DATASET = 1; %% initializing flag for dataset  flag if 1 we use ORIGINAL dataset
myCONFIG.FLAGS.MODIFIED_DATASET = 0; %% initializing flag for dataset  flag if 1 we use MODIFIED dataset
myCONFIG.FLAGS.DATA_PLAY =0; % if 1 just loads the previously saved data
myCONFIG.FLAGS.CONFIDENCE_MAP = 0; % if 0 it means that confidence data is not available
myCONFIG.FLAGS.INITIAL_ORIENTATION_COMPENSATION =1; %% (if 1 compensate the initial orientation)
myCONFIG.FLAGS.GROUND_TRUTH_AVAILABLE =0; %% (if 1 it means that we have ground truth)
myCONFIG.FLAGS.PLOT_RESULTS = 0; %% (if 1 plot the results otherwise no)
myCONFIG.FLAG.FEATURE_INITIALIZATION = 'PICK RANDOM SUBSET'; %% ('PICK RANDOM SUBSET'|'USE ALL FEATURES')
myCONFIG.FLAG.ONLY_PREDICT = 0; %% if equal to 1 only do the prediction otherwise do complete EKF



%%% determines whether to initialze a random subset of features or all
%%% features availabe at each frame
if ~initialization_done
    create_data_folders(myCONFIG.PATH.DATA_FOLDER)
    create_data_folders(myCONFIG.PATH.KEYFRAMES_FOLDER)
    initialization_done = 1;
    read_save_time_stamp()
end



%  myCONFIG.FLAGS.FEATURE_EXTRACTOR = 'FAST';
myCONFIG.FLAGS.FEATURE_EXTRACTOR = 'SIFT';
myCONFIG.PATH.RANSAC = [myCONFIG.PATH.DATA_FOLDER,'RANSAC/'];

%%%%
myCONFIG.TMEP_CODE =1;
myCONFIG.REF_COORDINATE = 'cam1';
myCONFIG.FLAGS.OVERWRITE = 1; %% if equal to 1 overwrite the snapshot data files
addpath(genpath(pwd))
myCONFIG.FLAGS.RECALCULATE = 0; %%% if equal to 1 recalculate the results (RANSAC)



%% CLEAR PERSISTENT VARIABLES
clear functions

end
function create_folder(folder_)
if ~exist(['./',folder_],'dir') %% if folder does not already exist make new folder
    mkdir(folder_)
end
end
function create_data_folders(path_)
global myCONFIG
current_path = pwd;
create_folder(path_);
eval(['cd ','''',path_,'''']);
create_folder('FeatureExtractionMatching');
create_folder('images');
create_folder('RANSAC_pose_shift');
create_folder('RANSAC_pose_shift_dr_Ye');
create_folder('xyz_data');
create_folder('DataSnapshots');
create_folder('FeaturePerformance');
create_folder('OrientationData');
create_folder('TimeStamp');
create_folder('VelocityData');

eval(['cd ','''',current_path,'''']);
end
%%
function read_save_time_stamp()
global myCONFIG
if ~exist([myCONFIG.PATH.SOURCE_FOLDER,'/TimeStamp/TimeStamp.mat'],'file')
    
    
    nDataFiles = data_file_counting(myCONFIG.PATH.DATA_FOLDER,'d1');
    time_stamp = zeros(2,nDataFiles);
    for i=1:nDataFiles
        %     source_data_file = sprintf('%s/d1_%04d.dat',source_folder,i);
        
        [s, err]=sprintf('%s/d1_%04d.dat',myCONFIG.PATH.SOURCE_FOLDER,i);
        sr_data = load(s);
        
        if size(sr_data,1)==721
            time_stamp(:,i) = [sr_data(721,1);i];
        else
            time_stamp(:,i) = [-1;i];
        end
        i
    end
    
    save([myCONFIG.PATH.SOURCE_FOLDER,'/TimeStamp/TimeStamp.mat'],'time_stamp')
    time_difference = time_stamp(1,2:end) - time_stamp(1,1:end-1);
    figure;plot(time_difference/1000)
end
end

%%








% RANSAC_FileName = '/home/amirhossein/Desktop/Current_Work/august 2011/EKF_monoSLAM_1pRANSAC/RANSAC_0_200_mod.mat';
% scan_file_name_prefix ='/home/amirhossein/Desktop/Current_Work/TestAlgorithm/IJRR-Dataset-1-subset/SCANS';
% sequencePath = '/home/amirhossein/Desktop/Current_Work/TestAlgorithm/IJRR-Dataset-1-subset/SCANS';


% if ~exist('./FeatureExtractionMatching','dir') %% if folder does not already exist make new folder
%     mkdir FeatureExtractionMatching
% end
% if ~exist('./images','dir')  %% if folder does not already exist make new folder
%     mkdir images
% end
% if ~exist('./RANSAC_pose_shift','dir')  %% if folder does not already exist make new folder
%     mkdir RANSAC_pose_shift
% end
% if ~exist('./RANSAC_pose_shift_dr_Ye','dir')  %% if folder does not already exist make new folder
%     mkdir RANSAC_pose_shift_dr_Ye
% end
%
% if ~exist('./xyz_data','dir')  %% if folder does not already exist make new folder
%     mkdir xyz_data
% end
% if ~exist('./DataSnapshots','dir')  %% if folder does not already exist make new folder
%     mkdir DataSnapshots
% end
%
% if ~exist('./FeaturePerformance','dir')  %% if folder does not already exist make new folder
%     mkdir FeaturePerformance
% end
% if ~exist('./OrientationData','dir')  %% if folder does not already exist make new folder
%     mkdir OrientationData
% end
% if ~exist('./TimeStamp','dir')  %% if folder does not already exist make new folder
%     mkdir TimeStamp
% end

%% DEBUG UNCOMMENT LATER
% % % % if isempty(extractor)
% % % %     extractor =input('What Type of extractor do you want to work with: [ 0 = FAST , 1 = SIFT] : ','s');
% % % % end
% % % % if str2double(extractor) ==0
% % % %     disp('FAST extractor is used')
% % % %
% % % %     myCONFIG.FLAGS.FEATURE_EXTRACTOR = 'FAST';
% % % % end
% % % %
% % % % if str2double(extractor) ==1
% % % %     disp('SIFT extractor is used')
% % % %     myCONFIG.FLAGS.FEATURE_EXTRACTOR = 'SIFT';
% % % % end
% % % %




