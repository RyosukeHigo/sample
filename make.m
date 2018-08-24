function make(arg_char)
clear mex
makeConst

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SysDef.h を読み込んで，使用するシステムの番号を SYSTEM_SET へ代入
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SYSTEM_SET = [];
for sys_num = 1:SYSTEM_NUM
    ret = setSYSTEM(sys_num, SYSTEM_REGEXP);
    if ret  SYSTEM_SET = [SYSTEM_SET, ret];   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SysDef.h で 1に設定してあるライブラリだけをコンパイル
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin == 0
    for sys_num = 1:numel(SYSTEM_SET)
        eval(['delete ' SYSTEM_DLL_DELETE(SYSTEM_SET(sys_num),:)]);
        disp(['    ', SYSTEM_STRING(SYSTEM_SET(sys_num),:), ' compiling ...']);
        makeSYSTEM{SYSTEM_SET(sys_num)}(SYSTEM_PATH);
    end
    disp(['    ', 'Finished !']);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% すべてのライブラリをコンパイル
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(arg_char,'all')
    for sys_num=1:SYSTEM_NUM
        disp(['    ', SYSTEM_STRING(sys_num,:), ' compiling ...']);
        makeSYSTEM{sys_num}(SYSTEM_PATH);
    end
    disp(['    ', 'Finished !']);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SysDef.h で 1に設定してあるライブラリだけを削除
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(arg_char,'clean')
    for sys_num=1:numel(SYSTEM_SET)-1
        eval(['delete ' SYSTEM_DLL_DELETE(SYSTEM_SET(sys_num),:)]);
        disp(['    ', SYSTEM_STRING(SYSTEM_SET(sys_num),:), ' deleted !']);
    end
    cleanCurrentDir();
    disp(['    ', SYSTEM_STRING(SYSTEM_DIR,:), ' deleted !']);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% すべてのライブラリを削除
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(arg_char,'allclean')
    for sys_num = 1:5
        eval(['delete ' SYSTEM_DLL_DELETE(sys_num,:)]);
        disp(['    ', SYSTEM_STRING(sys_num,:), ' deleted !']);
    end
    cleanCurrentDir();
    disp(['    ', SYSTEM_STRING(SYSTEM_DIR,:), ' deleted !']);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% その他
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(arg_char,'path')
    makePath()
elseif strcmp(arg_char,'hand')
    disp(['    ', SYSTEM_STRING(SYSTEM_HAND,:), ' compiling ...']);
    makeHAND(SYSTEM_PATH);
elseif strcmp(arg_char,'wam')
    disp(['    ', SYSTEM_STRING(SYSTEM_WAM,:), ' compiling ...']);
    makeWAM(SYSTEM_PATH);
elseif strcmp(arg_char,'avs')
    disp(['    ', SYSTEM_STRING(SYSTEM_AVS,:), ' compiling ...']);
    makeAVS(SYSTEM_PATH);
elseif strcmp(arg_char,'xyz')
    disp(['    ', SYSTEM_STRING(SYSTEM_XYZ,:), ' compiling ...']);
    makeXYZ(SYSTEM_PATH);
elseif strcmp(arg_char,'force')
    disp(['    ', SYSTEM_STRING(SYSTEM_FORCE,:), ' compiling ...']);
    makeFORCE(SYSTEM_PATH);
elseif strcmp(arg_char,'dir')
    disp(['    ', SYSTEM_STRING(SYSTEM_DIR,:), ' compiling ...']);
    makeCurrentDir(SYSTEM_PATH);
else
    disp('  Such argument does NOT exist !')
end
clear mex

%%%%%%%%%%%%%%%%%%%%%%%%%%
% サブ関数
%%%%%%%%%%%%%%%%%%%%%%%%%%
% パスの設定
function    makePath()
    addpath ../../bin ../../hand ../../wam ../../avs ../../force ../../xyzstage
% システム使用設定の取得
function    ret = setSYSTEM(sys_num, SYSTEM_REGEXP)
    global SYSTEM_DIR
    if sys_num == SYSTEM_DIR     ret = SYSTEM_DIR;    return;   end
    fid = fopen('SysDef.h','r');
    ret = 0;
    while feof(fid) == 0
       str = regexp(fgetl(fid), {SYSTEM_REGEXP{1,sys_num}});        % 正規表現にマッチすれば1,しなければ空行列を返す
       if  str{1,1} > 1     ret = sys_num; break;   end
    end
    fclose(fid);
% システム初期設定,タスク初期設定
function    makeCurrentDir(SYSTEM_PATH)
    LIB_FILE = [' ../../basis/pmatrix.c ../../basis/homoMat.c ../../basis/myMatrix.c ../../basis/Vector.c ../../basis/Kalman.c '];
    MY_FILE = [' HandApp.c WAMApp.c WAM2App.c AVSApp.c XYZstageApp.c Predict3D.c throwJnt2armJnt.c '];
    eval(['mex' SYSTEM_PATH 'SystemSet.c'])
    eval(['mex' SYSTEM_PATH 'TaskSet.c ../../hand/HandFunc.c ../../wam/WAMFunc.c' LIB_FILE MY_FILE])
    % RTI ethernet blockset decode and encode
    eval(['mex' SYSTEM_PATH 'rtiethernetdecode.c'])
    eval(['mex' SYSTEM_PATH 'rtiethernetencode.c'])
% ハンド
function    makeHAND(SYSTEM_PATH)
    LIB_FILE = [' ../../basis/pmatrix.c ../../basis/homoMat.c ../../basis/myMatrix.c ../../basis/Vector.c ../../basis/Kalman.c '];
    HAND_DLL_PATH = [' -outdir ../../hand ../../hand/'];
    HAND_LIB_FILE = [' ../../hand/HandFunc.c '];
    HAND_WAM_CONNECT_FILE = [' WAMApp.c WAM2App.c Predict3D.c throwJnt2armJnt.c '];
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'handCtrl.c HandApp.c'])
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'handDALimit.c HandApp.c'])
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'handEnc2JntAng.c HandApp.c'])
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'handJntAngLimit.c'])
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'handJntTrq2DA.c'])
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'HandSetZero.c'])
    eval(['mex' SYSTEM_PATH HAND_DLL_PATH 'handTraj.c HandApp.c ../../basis/pmatrix.c ../../basis/homoMat.c storeData.c' HAND_LIB_FILE HAND_WAM_CONNECT_FILE LIB_FILE])
% アーム
function    makeWAM(SYSTEM_PATH)
    LIB_FILE = [' ../../basis/pmatrix.c ../../basis/homoMat.c ../../basis/myMatrix.c ../../basis/Vector.c ../../basis/Kalman.c '];
    WAM_DLL_PATH = [' -outdir ../../wam ../../wam/'];
    WAM_LIB_FILE = [' ../../wam/WAMFunc.c '];
    WAM_MY_FILE = [' WAMApp.c WAM2App.c Predict3D.c throwJnt2armJnt.c '];
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamCtrl.c' WAM_MY_FILE LIB_FILE])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamDA2MotorTrq.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamDALimit.c' WAM_MY_FILE LIB_FILE])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamEnc2MotorAng.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamJntAng2MotorAng.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamJntAngLimit.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamJntTrq2MotorTrq.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamMotorAng2JntAng.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamMotorTrq2DA.c'])
    eval(['mex' SYSTEM_PATH WAM_DLL_PATH 'wamTraj.c ../../basis/pmatrix.c ../../basis/homoMat.c storeData.c' WAM_MY_FILE WAM_LIB_FILE LIB_FILE ])
% アクティブビジョン
function    makeAVS(SYSTEM_PATH)
    AVS_DLL_PATH = [' -outdir ../../avs ../../avs/'];
    AVS_LIB_FILE = [' ../../basis/LinkHomoMat.c ../../basis/vec3DRot.c ../../basis/vec3DInProduct.c '];
    AVS_MY_FILE = [' AVSApp.c '];
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avs3Dcalc.c' AVS_LIB_FILE AVS_MY_FILE])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsCtrl.c' AVS_LIB_FILE AVS_MY_FILE])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsDALimit.c' AVS_MY_FILE])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsEnc2JntAng.c'])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsJntAngLimit.c'])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsJntTrq2DA.c'])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsTraj.c AVSApp.c'])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsVisualTraj.c' AVS_MY_FILE])
    eval(['mex' SYSTEM_PATH AVS_DLL_PATH 'avsStoreData.c'])
% FORCE
function    makeFORCE(SYSTEM_PATH)
    FORCE_DLL_PATH = [' -outdir ../../force ../../force/'];
    eval(['mex' SYSTEM_PATH FORCE_DLL_PATH 'Force.c'])
    eval(['mex' SYSTEM_PATH FORCE_DLL_PATH 'ForceVol2Force.c'])
% XYZstage
function    makeXYZ(SYSTEM_PATH)
    XYZ_DLL_PATH = [' -outdir ../../xyzstage ../../xyzstage/'];
    XYZ_MY_FILE = [' XYZstageApp.c '];
    eval(['mex' SYSTEM_PATH XYZ_DLL_PATH 'XYZstageCtrl.c' XYZ_MY_FILE])
    eval(['mex' SYSTEM_PATH XYZ_DLL_PATH 'XYZstageDALimit.c'])
    eval(['mex' SYSTEM_PATH XYZ_DLL_PATH 'XYZstageEnc2JntAng.c'])
    eval(['mex' SYSTEM_PATH XYZ_DLL_PATH 'XYZstageAngLimit.c'])
    eval(['mex' SYSTEM_PATH XYZ_DLL_PATH 'XYZstageTrq2DA.c'])
    eval(['mex' SYSTEM_PATH XYZ_DLL_PATH 'XYZstageTraj.c' XYZ_MY_FILE])
% カレントディレクトリ削除
function    cleanCurrentDir()
    %delete *.dll *.exp *.lib *.map *.ppc *.sdf base.trc *.asv
    delete *.mexw64 *.exp *.lib *.map *.ppc *.sdf base.trc *.asv
    % ディレクトリの場合は返り値7
    if exist('./base_rti1007','dir') == 7   rmdir('./base_rti1007','s');    end
    if exist('./slprj','dir') == 7  rmdir('./slprj','s');   end
