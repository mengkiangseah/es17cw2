%% startup
clear variables
clc
close all
%% read input file
[~, ~, raw] = xlsread('posSpeed.xlsx');

spd5 = raw(2:end,1);
spd10 = raw(2:end,3);
spd15 = raw(2:end,5);
spd20 = raw(2:end,8);
spd30 = raw(2:end,10);
spd40 = raw(2:end,12);

Ts = 1/(15.4*10);
time = 1:Ts:100;

%% extract pwm and velocity data
%5
temp = strsplit(spd5{1}, ' ');
pwm5(1,1) = str2double(temp{1});
vel5(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd5)
    temp = strsplit(spd5{i}, ' ');
    pwm5(i,1) = str2double(temp{2});
    vel5(i,1) = str2double(temp{3});
end
catch e
end
%10
temp = strsplit(spd10{1}, ' ');
pwm10(1,1) = str2double(temp{1});
vel10(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd10)
    temp = strsplit(spd10{i}, ' ');
    pwm10(i,1) = str2double(temp{2});
    vel10(i,1) = str2double(temp{3});
end
catch e
end

%15
temp = strsplit(spd15{1}, ' ');
pwm15(1,1) = str2double(temp{1});
vel15(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd15)
    temp = strsplit(spd15{i}, ' ');
    pwm15(i,1) = str2double(temp{2});
    vel15(i,1) = str2double(temp{3});
end
catch e
end

%20
temp = strsplit(spd20{1}, ' ');
pwm20(1,1) = str2double(temp{1});
vel20(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd20)
    temp = strsplit(spd20{i}, ' ');
    pwm20(i,1) = str2double(temp{2});
    vel20(i,1) = str2double(temp{3});
end
catch e
end

%30
temp = strsplit(spd30{1}, ' ');
pwm30(1,1) = str2double(temp{1});
vel30(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd30)
    temp = strsplit(spd30{i}, ' ');
    pwm30(i,1) = str2double(temp{2});
    vel30(i,1) = str2double(temp{3});
end
catch e
end

%40
temp = strsplit(spd40{1}, ' ');
pwm40(1,1) = str2double(temp{1});
vel40(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd40)
    temp = strsplit(spd40{i}, ' ');
    pwm40(i,1) = str2double(temp{2});
    vel40(i,1) = str2double(temp{3});
end
catch e
end

figure
plot(time(1:length(pwm5)), pwm5,'r');
hold on
plot(time(1:length(pwm10)), pwm10,'g');
plot(time(1:length(pwm15)), pwm15,'b');
plot(time(1:length(pwm20)), pwm20,'m');
plot(time(1:length(pwm30)), pwm30,'c');
plot(time(1:length(pwm40)), pwm40,'k');
title('PWM duty');
legend('5','10','15','20','30','40');
ylabel('PWM duty');
xlabel('time [s]');
saveas(gcf,'PWM_pos.png');

figure
plot(time(1:length(vel5)), vel5,'r');
hold on
hline(5,'r:');
plot(time(1:length(vel10)), vel10,'g');
hline(10,'g:');
plot(time(1:length(vel15)), vel15,'b');
hline(15,'b:');
plot(time(1:length(vel20)), vel20,'m');
hline(20,'m:');
plot(time(1:length(vel30)), vel30,'c');
hline(30,'c:');
plot(time(1:length(vel40)), vel40,'k');
hline(40,'k:');

title('velocity')
legend('5','10','15','20','30','40');
ylim([0,45]);
ylabel('speed [rps]');
xlabel('time [s]');
saveas(gcf,'velocity_pos.png');


%% read input file
[~, ~, raw] = xlsread('negSpeed.xlsx');

spd5 = raw(2:end,1);
spd10 = raw(2:end,3);
spd15 = raw(2:end,5);
spd20 = raw(2:end,7);
spd30 = raw(2:end,9);
spd40 = raw(2:end,11);
spd50 = raw(2:end,13);
spd60 = raw(2:end,15);

%% extract pwm and velocity data
%5
temp = strsplit(spd5{1}, ' ');
pwm5(1,1) = str2double(temp{1});
vel5(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd5)
    temp = strsplit(spd5{i}, ' ');
    pwm5(i,1) = str2double(temp{2});
    vel5(i,1) = str2double(temp{3});
end
catch e
end
%10
temp = strsplit(spd10{1}, ' ');
pwm10(1,1) = str2double(temp{1});
vel10(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd10)
    temp = strsplit(spd10{i}, ' ');
    pwm10(i,1) = str2double(temp{2});
    vel10(i,1) = str2double(temp{3});
end
catch e
end

%15
temp = strsplit(spd15{1}, ' ');
pwm15(1,1) = str2double(temp{1});
vel15(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd15)
    temp = strsplit(spd15{i}, ' ');
    pwm15(i,1) = str2double(temp{2});
    vel15(i,1) = str2double(temp{3});
end
catch e
end

%20
temp = strsplit(spd20{1}, ' ');
pwm20(1,1) = str2double(temp{1});
vel20(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd20)
    temp = strsplit(spd20{i}, ' ');
    pwm20(i,1) = str2double(temp{2});
    vel20(i,1) = str2double(temp{3});
end
catch e
end

%30
temp = strsplit(spd30{1}, ' ');
pwm30(1,1) = str2double(temp{1});
vel30(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd30)
    temp = strsplit(spd30{i}, ' ');
    pwm30(i,1) = str2double(temp{2});
    vel30(i,1) = str2double(temp{3});
end
catch e
end

%40
temp = strsplit(spd40{1}, ' ');
pwm40(1,1) = str2double(temp{1});
vel40(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd40)
    temp = strsplit(spd40{i}, ' ');
    pwm40(i,1) = str2double(temp{2});
    vel40(i,1) = str2double(temp{3});
end
catch e
end

%50
temp = strsplit(spd50{1}, ' ');
pwm50(1,1) = str2double(temp{1});
vel50(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd50)
    temp = strsplit(spd50{i}, ' ');
    pwm50(i,1) = str2double(temp{2});
    vel50(i,1) = str2double(temp{3});
end
catch e
end

%60
temp = strsplit(spd40{1}, ' ');
pwm60(1,1) = str2double(temp{1});
vel60(1,1) = str2double(temp{2});
try     %because it will fail once it reaches NaN
for i=2:size(spd60)
    temp = strsplit(spd60{i}, ' ');
    pwm60(i,1) = str2double(temp{2});
    vel60(i,1) = str2double(temp{3});
end
catch e
end


figure
plot(time(1:length(pwm5)), pwm5);
hold on
plot(time(1:length(pwm10)), pwm10);
plot(time(1:length(pwm15)), pwm15);
plot(time(1:length(pwm20)), pwm20);
plot(time(1:length(pwm30)), pwm30);
plot(time(1:length(pwm40)), pwm40);
plot(time(1:length(pwm30)), pwm30);
plot(time(1:length(pwm40)), pwm40);

title('PWM duty');
legend('5','10','15','20','30','40','50','60');
ylabel('PWM duty');
xlabel('time [s]');
saveas(gcf,'PWM_neg.png');

figure
plot(time(1:length(vel5)), vel5);
hold on
hline(5,':');
plot(time(1:length(vel10)), vel10);
hline(10,':');
plot(time(1:length(vel15)), vel15);
hline(15,':');
plot(time(1:length(vel20)), vel20);
hline(20,':');
plot(time(1:length(vel30)), vel30);
hline(30,':');
plot(time(1:length(vel40)), vel40);
hline(40,':');
plot(time(1:length(vel50)), vel50);
hline(50,':');
plot(time(1:length(vel60)), vel60);
hline(60,':');


title('velocity')
legend('5','10','15','20','30','40','50','60');
ylim([0,75]);
ylabel('speed [rps]');
xlabel('time [s]');
saveas(gcf,'velocity_neg.png');
