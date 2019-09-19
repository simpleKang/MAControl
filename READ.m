fname = 'result[0.8-0.7-0.7][0.8-0.8-0.8new]';
vname = 'score_877_888n';

[~,c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,score,~]=...
    textread(strcat(fname,'.log'),...
    '%s %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %f %s',...
    'delimiter',',',...
    'headerlines',4);

clear c*
score = score(1:200,:);
s.(vname) = score

% READ SCORES from a STANDARD log file generated from a SINGLE CLEAN run
% 1) put the xxx.log file and this .m file under the same directory
% 2) modify 'fname' and 'vname' accordingly

save(strcat(fname,'.mat'),'s')