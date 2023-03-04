%% generate the folder named by date
% DateString=datestr(datetime('now'));
% DateString = strrep(DateString,':','_');
% DateString = strrep(DateString,'-','_');
% DateString = strrep(DateString,' ','_');
% mkdir(DateString);
% copyfile('./Init.m',[DateString,'/']);
% copyfile('./zhuan1.m',[DateString,'/']);

%% generate the gait Date
Generator();


%% ZMP check
% ZMP_check_1(DateString);
