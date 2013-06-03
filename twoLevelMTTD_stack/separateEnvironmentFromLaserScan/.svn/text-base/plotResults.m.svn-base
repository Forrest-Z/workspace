clear all; close all;

load report.txt

dimension = 2;
nbeams = 181;
treshold_env = 0.1;
maxNumComp = 5;

colors=['b' 'g' 'r' 'c' 'm' 'y' 'k'];
counter = 1;

num = 1;
timestep = report(:,counter:counter+num-1);
counter = counter + num;

num = 1;
numMeasurements = report(:,counter:counter+num-1);
counter = counter + num;

num = nbeams;
distancesSeparatedObjects = report(:,counter:counter+num-1);
counter = counter + num;

num = nbeams;
anglesSeparatedObjects = report(:,counter:counter+num-1);
counter = counter + num;

num = nbeams;
probMeasurementsEnv = report(:,counter:counter+num-1);
counter = counter + num;

num = nbeams;
laserAngle = report(:,counter:counter+num-1);
counter = counter + num;

num = nbeams;
laserDistance = report(:,counter:counter+num-1);
counter = counter + num;



for i=1:length(timestep)
    figure(1);
    subplot(2,1,1);
    legende = {};
    hold off;
    plot(laserDistance(i,:) .* cos(laserAngle(i,:)), laserDistance(i,:) .* sin(laserAngle(i,:)), 'k*');
    legende = cat(1,legende,{'measurements' });
    hold on;
    plot(distancesSeparatedObjects(i,1:numMeasurements(i)) .* cos(anglesSeparatedObjects(i,1:numMeasurements(i))), distancesSeparatedObjects(i,1:numMeasurements(i)) .* sin(anglesSeparatedObjects(i,1:numMeasurements(i))), 'r*');
    legende = cat(1,legende,{'separated measurements' });
    hold on;
    title('Results seperator')
    hold on;
    axis([-8,8,-1,8])
    legend(legende);

    figure(1);
    subplot(2,1,2);
    legende = {};
    hold off;
    indices_obj = find(probMeasurementsEnv(i,:)<treshold_env);
    indices_env = find(probMeasurementsEnv(i,:)>=treshold_env);
    plot(laserDistance(i,indices_env) .* cos(laserAngle(i,indices_env)), laserDistance(i,indices_env) .* sin(laserAngle(i,indices_env)), 'k*');
    legende = cat(1,legende,{'environment' });
    hold on;
    plot(laserDistance(i,indices_obj) .* cos(laserAngle(i,indices_obj)), laserDistance(i,indices_obj) .* sin(laserAngle(i,indices_obj)), 'r*');
    legende = cat(1,legende,{'objects' });
    %title('Results seperator')
    hold on;
    axis([-8,8,-1,8])
    legend(legende);

    pause(0.1);

endfor

