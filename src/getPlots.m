% Author: Benjamin Semal
% Date: 24/08/2016
% @: Cranfield University
%
% Description: This script generates all the plots

global n_pause n_figure
n_pause     = 0;
n_figure    = 1;

if cst.N<=61, N_sim = cst.N;
else N_sim = 61; end


%% Plot trajectories
if cst.N_MC==1,
    switch(pbm.scn)
        case 1
            for k=1:N_sim-1, plotSimType1(k,agt); end
            display(mean(agt.UAV1.L))
        case 2
            for k=1:N_sim-1, plotSimType2(k,pbm,agt); end
            display(mean(agt.UAV1.L))
            display(mean(agt.UAV2.L))
        case 3
            for k=1:N_sim-1, plotSimType3(k,pbm,agt); end
            display(mean(agt.UAV1.L))
            display(mean(agt.UAV2.L))
        otherwise
            error('simType is not set properly')
    end
end


%% Plot L, d, and gamma
if cst.N_MC==1,
    switch(pbm.scn)
        case 1
            k = 1:N_sim-1;
            figure
            subplot(3,1,1)
            stem(agt.UAV1.L,agt.UAV1.color)
            title('Error Covariance')
            xlabel('Time (s)')
            ylabel('Trace(P)')
            grid on
            axis([1 N_sim-1 0 max(agt.UAV1.L)])
            
            subplot(3,1,2)
            plot(agt.UAV1.d,'r')
            title('Planar Distance')
            xlabel('Time (s)')
            ylabel('d (m)')
            grid on
            axis([1 N_sim-1 0 max(agt.UAV1.d)])
            
            subplot(3,1,3)
            stem(agt.UAV1.L,agt.UAV1.color)
            hold on
            plot(agt.UAV1.d*mean(agt.UAV1.L)/mean(agt.UAV1.d),'r')
            hold off
            xlabel('Time (s)')
            grid on
            axis([1 N_sim-1 0 max(agt.UAV1.d*mean(agt.UAV1.L)/mean(agt.UAV1.d))])
            
        case 2
            k = 1:N_sim-1;
            figure
            subplot(3,1,1)
            stem(agt.UAV1.L,agt.UAV1.color)
            hold on
            stem(agt.UAV2.L,agt.UAV2.color)
            hold off
            title('Error Covariance')
            xlabel('Time (s)')
            ylabel('Trace(P)')
            grid on
            axis([1 N_sim-1 0 max(agt.UAV1.L)])
            
            if strcmp(pbm.sol,'group'),
                subplot(3,1,2)
                plot(agt.UAV1.d,agt.UAV1.color)
                hold on
                plot(agt.UAV2.d,agt.UAV2.color)
                plot(mean([agt.UAV1.d' agt.UAV2.d'],2),'--r')
                hold off
                title('Planar Distance')
                xlabel('Time (s)')
                ylabel('d (m)')
                grid on
                axis([1 N_sim-1 0 max(agt.UAV2.d)])
            else
                subplot(3,1,2)
                plot(agt.UAV1.d,agt.UAV1.color)
                hold on
                plot(agt.UAV2.d,agt.UAV2.color)
                hold off
                title('Planar Distance')
                xlabel('Time (s)')
                ylabel('d (m)')
                grid on
                axis([1 N_sim-1 0 max(agt.UAV2.d)])
            end
            
            subplot(3,1,3)
            plot(rad2deg(agt.UAV2.gamma),'r')
            title('Separation angle')
            xlabel('Time (s)')
            ylabel('\gamma (°)')
            grid on
            axis([1 N_sim-1 0 max(rad2deg(agt.UAV1.gamma))])

        case 3
            k = 1:N_sim-1;
            figure
            subplot(3,1,1)
            stem(agt.UAV1.L,agt.UAV1.color)
            hold on
            stem(agt.UAV2.L,agt.UAV2.color)
            hold off
            title('Error Covariance')
            xlabel('Time (s)')
            ylabel('Trace(P)')
            grid on
            axis([1 N_sim-1 0 max(agt.UAV1.L)])

            if strcmp(pbm.sol,'group'),
                subplot(3,1,2)
                plot(agt.UAV1.d,agt.UAV1.color)
                hold on
                plot(agt.UAV2.d,agt.UAV2.color)
                plot(mean([agt.UAV1.d' agt.UAV2.d'],2),'--r')
                hold off
                title('Planar Distance')
                xlabel('Time (s)')
                ylabel('d (m)')
                grid on
                axis([1 N_sim-1 0 max(agt.UAV2.d)])
            else
                subplot(3,1,2)
                plot(agt.UAV1.d,agt.UAV1.color)
                hold on
                plot(agt.UAV2.d,agt.UAV2.color)
                hold off
                title('Planar Distance')
                xlabel('Time (s)')
                ylabel('d (m)')
                grid on
                axis([1 N_sim-1 0 max(agt.UAV2.d)])
            end

            subplot(3,1,3)
            plot(rad2deg(agt.UAV2.gamma),'r')
            title('Separation angle')
            xlabel('Time (s)')
            ylabel('\gamma (°)')
            grid on
            axis([1 N_sim-1 0 max(rad2deg(agt.UAV1.gamma))])

        otherwise
            error('simType is not set properly')
    end
end


%% Plot L and gamma
if cst.N_MC==1,
    figure
    stem(agt.UAV2.L,agt.UAV2.color)
    hold on
    plot(rad2deg(agt.UAV2.gamma),'Color','k','LineStyle','-.','LineWidth',1)
    plot([1 N_sim-1],[90 90],'Color','r','LineStyle','--','LineWidth',1)
    hold off
    title('Error Covariance & Separation angle')
    xlabel('Time (s)')
    legend('trace(P) (m^2)','\gamma (°)','90°')
    grid on
    axis([1 N_sim-1 0 180])
end


%% Plot Monte-Carlo histograms
if cst.N_MC>1,
    figure
    histogram(data.outputVector,'BinWidth',5)
    title(strcat(num2str(pbm.scn),';',pbm.sol))
    xlabel('Average (trace(P))')
    ylabel('Frequency')
end

