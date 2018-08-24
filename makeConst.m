global  SYSTEM_HAND;
global  SYSTEM_WAM;
global  SYSTEM_AVS;
global  SYSTEM_FORCE;
global  SYSTEM_XYZ;
global  SYSTEM_DIR;

% MATLAB���ł̒ʂ��ԍ�
SYSTEM_HAND = 1;
SYSTEM_WAM = 2;
SYSTEM_AVS = 3;
SYSTEM_FORCE = 4;
SYSTEM_XYZ = 5;
SYSTEM_DIR = 6;

SYSTEM_NUM = 6;
% �p�X�̐ݒ�
SYSTEM_PATH = [' -I. -I../../system -I../../basis -I../../hand -I../../wam -I../../avs -I../../force  -I../../xyzstage '];    % �ŏ��ƍŌ�ɋ󔒂����ނ���!!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �ȉ��̔z��͒ʂ��ԍ��ƑΉ����鏇�Ԃŏ������ƁI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �֐��n���h��
makeSYSTEM = {@makeHAND, @makeWAM, @makeAVS, @makeFORCE, @makeXYZ, @makeCurrentDir};
% �R�}���h������
% SYSTEM_DLL_DELETE = char('../../hand/*.dll',...
%                         '../../wam/*.dll',...
%                         '../../avs/*.dll',...
%                         '../../cpv/*.dll',...
%                         '../../force/*.dll',...
%                         '*.dll *.exp *.lib *.map *.ppc *.sdf base.trc *.asv');
SYSTEM_DLL_DELETE = char('../../hand/*.mexw64',...
                        '../../wam/*.mexw64',...
                        '../../avs/*.mexw64',...
                        '../../force/*.mexw64',...
                        '../../xyzstage/*.mexw64',...
                        '*.mexw64 *.exp *.lib *.map *.ppc *.sdf base.trc *.asv');
% �\��������
SYSTEM_STRING = char('HAND library',...
                        'WAM library',...
                        'AVS library',...
                        'FORCE library',...
                        'XYZStage library',...
                        'Current directory');
% ���K�\��
SYSTEM_REGEXP{1,1} = ['SYSTEM_HAND[1-2]\s+1'];
SYSTEM_REGEXP{1,2} = ['SYSTEM_WAM[1-2]\s+1'];
SYSTEM_REGEXP{1,3} = ['SYSTEM_AVS\s+1'];
SYSTEM_REGEXP{1,4} = ['SYSTEM_FORCE\s+1'];
SYSTEM_REGEXP{1,5} = ['SYSTEM_XYZ\s+1'];
