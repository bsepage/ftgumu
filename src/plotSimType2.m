function plotSimType2( k, pbm, agt )
    global n_pause n_figure
    if k==1,
        figure(n_figure)
        hold on
    else
        h_TGT = plot(agt.TGT1.o(1,k),agt.TGT1.o(2,k),'Color',agt.TGT1.color,'Marker',agt.TGT1.marker);
        h_UAV1 = plot(agt.UAV1.s(1,k),agt.UAV1.s(2,k),'Color',agt.UAV1.color,'Marker',agt.UAV1.marker);
        h_UAV2 = plot(agt.UAV2.s(1,k),agt.UAV2.s(2,k),'Color',agt.UAV2.color,'Marker',agt.UAV2.marker);
        
        plot(agt.TGT1.o(1,1:k),agt.TGT1.o(2,1:k),'Color',agt.TGT1.color,'LineStyle',agt.TGT1.trace);
        plot(agt.UAV1.s(1,1:k),agt.UAV1.s(2,1:k),'Color',agt.UAV1.color,'LineStyle',agt.UAV1.trace);
        plot(agt.UAV2.s(1,1:k),agt.UAV2.s(2,1:k),'Color',agt.UAV2.color,'LineStyle',agt.UAV2.trace);
        
        if mod(k+1,2),
            if strcmp(pbm.sol,'independant'),
                P = inv(inv(agt.UAV1.P(:,:,k))+inv(agt.UAV2.P(:,:,k)));
            elseif strcmp(pbm.sol,'dependant') || strcmp(pbm.sol,'group'),
                P = agt.UAV2.P(:,:,k);
            end
            lbda_1  = max(eig(P));
            lbda_2  = min(eig(P));
            a       = sqrt(lbda_1);
            b       = sqrt(lbda_2);
            phi     = (1/2)*atan((2*P(1,2))/(P(1,1)-P(2,2)));
            theta   = linspace(0,2*pi,360);        
            X       = ones(size(theta))*agt.TGT1.o(1,k) + a*cos(theta)*cos(phi) - b*sin(theta)*sin(phi);
            Y       = ones(size(theta))*agt.TGT1.o(2,k) + a*cos(theta)*sin(phi) + b*sin(theta)*cos(phi);
            h_cov = plot(X,Y,'Color',[0.2 0.6 0.],'LineWidth',1);
%             legend([h_TGT h_UAV1 h_UAV2 h_cov],'TGT','UAV1','UAV2','P')
        end
        
        axis([0 300 -80 80])
        title('XY position')
        xlabel('x position (m)')
        ylabel('y position (m)')
        grid on
    end
    pause(n_pause)
end