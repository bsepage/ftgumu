function plotSimType1( k, agt )
    global n_pause n_figure
    if k==1,
        figure(n_figure)
        hold on
    else
        TGT = agt.TGT1;
        UAV = agt.UAV1;
        
        h_TGT = plot(TGT.o(1,k),TGT.o(2,k),'Color',TGT.color,'Marker',TGT.marker);
        h_UAV = plot(UAV.s(1,k),UAV.s(2,k),'Color',UAV.color,'Marker',UAV.marker);
        
        plot(TGT.o(1,1:k),TGT.o(2,1:k),'Color',TGT.color,'LineStyle',TGT.trace);
        plot(UAV.s(1,1:k),UAV.s(2,1:k),'Color',UAV.color,'LineStyle',UAV.trace);
        
        if mod(k+1,2),
            lbda_1  = max(eig(UAV.P(:,:,k)));
            lbda_2  = min(eig(UAV.P(:,:,k)));
            a       = sqrt(lbda_1);
            b       = sqrt(lbda_2);
            phi     = (1/2)*atan((2*UAV.P(1,2,k))/(UAV.P(1,1,k)-UAV.P(2,2,k)));
            theta   = linspace(0,2*pi,360);        
            X       = ones(size(theta))*TGT.o(1,k) + a*cos(theta)*cos(phi) - b*sin(theta)*sin(phi);
            Y       = ones(size(theta))*TGT.o(2,k) + a*cos(theta)*sin(phi) + b*sin(theta)*cos(phi);
            plot(X,Y,'Color','b','LineWidth',1)
        end
        
        axis([0 300 -80 80])
        title('XY position')
%         legend([h_TGT h_UAV],'TGT','UAV')
        xlabel('x position (m)')
        ylabel('y position (m)')
        grid on
    end
    pause(n_pause)
end