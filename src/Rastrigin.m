% x = -1:0.01:1;
% y = x
% z = zeros(length(x),length(y));
% 
% for i=1:length(x),
%     for j=1:length(y),
%         z(i,j) = 20 + x(i).^2 + y(j).^2 -10*(cos(2*pi.*x(i))+cos(2*pi.*y(j)));
%     end
% end
% 
% figure(1)
% % contour(x,y,z)
% surfc(x,y,z)
% grid on
% title('Function of Rastrigin')
% xlabel('X axis')
% ylabel('Y axis')


output = zeros(50,2);
% options = optimoptions('particleswarm','SwarmSize',20);
options = gaoptimset('PopulationSize',20);

for k=1:50,
%     optXY = particleswarm(@rastriginsfcn,2,[-1;-1],[1;1],options);
    optXY = ga(@rastriginsfcn,2,[],[],[],[],[-1;-1],[1;1],[],options);
    output(k,:) = optXY;
end
% figure
subplot(3,1,1)
histogram(sqrt(output(:,1).^2+output(:,2).^2),'BinWidth',0.1)
title('MC simulation - Genetic Algorithm - Rastrigin')
xlabel('Range to the global minima')
ylabel('Frequency')
axis([0 1.8 0 50])
grid on


output = zeros(50,2);
% options = optimoptions('particleswarm','SwarmSize',50);
options = gaoptimset('PopulationSize',50);

for k=1:50,
%     optXY = particleswarm(@rastriginsfcn,2,[-1;-1],[1;1],options);
    optXY = ga(@rastriginsfcn,2,[],[],[],[],[-1;-1],[1;1],[],options);
    output(k,:) = optXY;
end

subplot(3,1,2)
histogram(sqrt(output(:,1).^2+output(:,2).^2),'BinWidth',0.1)
xlabel('Range to the global minima')
ylabel('Frequency')
axis([0 1.8 0 50])
grid on


output = zeros(50,2);
% options = optimoptions('particleswarm','SwarmSize',20);
options = gaoptimset('PopulationSize',20);

for k=1:50,
%     optXY = particleswarm(@rastriginsfcn,2,[-0.5;-0.5],[0.5;0.5],options);
    optXY = ga(@rastriginsfcn,2,[],[],[],[],[-0.5;-0.5],[0.5;0.5],[],options);
    output(k,:) = optXY;
end

subplot(3,1,3)
histogram(sqrt(output(:,1).^2+output(:,2).^2),'BinWidth',0.1)
xlabel('Range to the global minima')
ylabel('Frequency')
axis([0 1.8 0 50])
grid on

