clc
clear

[index,c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,score,time]=...
    textread('result[0.8-0.7-0.7][0.5-0.5].log',...
    '%s %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %f %s',...
    'delimiter',',',...
    'headerlines',4);

score = score(1:200,:);

save result[0.8-0.7-0.7][0.5-0.5].mat score

% 作用：从纯净的单次结果的log文件中读取分数，已测试通过
% 使用方法：将这个文件和log文件放在同一个目录下，根据实际情况修改xxxxx.log及xxxxxx.mat并运行