classdef FleetManagerApp < matlab.apps.AppBase

    properties (Access = public)
        UIFigure              matlab.ui.Figure
        MainGridLayout        matlab.ui.container.GridLayout
        
        LeftPanel             matlab.ui.container.Panel
        LeftGrid              matlab.ui.container.GridLayout
        
        AppTitleLabel         matlab.ui.control.Label
        AppSubTitleLabel      matlab.ui.control.Label
        
        MapSectionLabel       matlab.ui.control.Label
        MapButtonGroup        matlab.ui.container.GridLayout
        RandomMapButton       matlab.ui.control.Button
        DarKoridorButton      matlab.ui.control.Button
        PlaceRobotsButton     matlab.ui.control.Button
        
        PlanSectionLabel      matlab.ui.control.Label
        PlannerDropDownLabel  matlab.ui.control.Label
        PlannerDropDown       matlab.ui.control.DropDown
        CongestDropDownLabel  matlab.ui.control.Label
        CongestDropDown       matlab.ui.control.DropDown
        
        StartSimulationBtn    matlab.ui.control.Button
        
        LayersLabel           matlab.ui.control.Label
        CheckPlanRoute        matlab.ui.control.CheckBox
        CheckRealPath         matlab.ui.control.CheckBox
        CheckWaitFor          matlab.ui.control.CheckBox

        RightPanel            matlab.ui.container.Panel
        RightGrid             matlab.ui.container.GridLayout
        
        TopBarGrid            matlab.ui.container.GridLayout
        AnalysisButton        matlab.ui.control.Button
        StatusLabel           matlab.ui.control.Label
        
        MetricsPanel          matlab.ui.container.Panel
        MetricsGrid           matlab.ui.container.GridLayout
        
        MetricCard1           matlab.ui.container.Panel
        MetricCard2           matlab.ui.container.Panel
        MetricCard3           matlab.ui.container.Panel
        
        MetricTimeLabel       matlab.ui.control.Label
        MetricTimeValue       matlab.ui.control.Label
        MetricDeadLabel       matlab.ui.control.Label
        MetricDeadValue       matlab.ui.control.Label
        MetricDoneLabel       matlab.ui.control.Label
        MetricDoneValue       matlab.ui.control.Label
        
        MapAxes               matlab.ui.control.UIAxes
        
        LegendPanel           matlab.ui.container.Panel
        LegendGrid            matlab.ui.container.GridLayout
    end

    properties (Access = private)
        OccupancyMap    binaryOccupancyMap 
        RobotPaths      cell   
        
        RobotStartPos   double 
        RobotGoalPos    double 
        RobotColors     double 
        NumRobots       double = 10 
        
        SimulationRunning logical = false 
        RobotStepIndices  double          
        SimTime           double = 0      
        stuckCounters     double
        
        RobotPathHistory cell
        
        CurrentSeed     double 
    end
    
    methods (Access = private)
        
   
        function success = attemptReplanning(app, robotIdx, activePositions)
            success = false;
            
            tempMap = copy(app.OccupancyMap);
            
            for j = 1:app.NumRobots
                if robotIdx == j, continue; end
                pos = activePositions(j, :);
                setOccupancy(tempMap, pos, 1); 
            end
            
            planner = plannerAStarGrid(tempMap);
            startWorld = app.RobotStartPos(robotIdx, :);
            goalWorld  = app.RobotGoalPos(robotIdx, :);
            
            startIdx = world2grid(tempMap, startWorld);
            goalIdx  = world2grid(tempMap, goalWorld);
            
            try
                pathIndices = plan(planner, startIdx, goalIdx);
                
                if ~isempty(pathIndices)
                    pathWorld = grid2world(tempMap, pathIndices);
                    app.RobotPaths{robotIdx} = pathWorld;
                    app.RobotStepIndices(robotIdx) = 1;
                    success = true;
                end
            catch
                success = false;
            end
        end

       
        function placeRobots(app)
            app.SimulationRunning = false;
            app.RobotStepIndices = [];
            app.SimTime = 0;
            app.MetricTimeValue.Text = '0.0s';
            app.MetricDoneValue.Text = '0/0';
            app.MetricDeadValue.Text = '0';
            app.StartSimulationBtn.Text = '▶  Simülasyonu Başlat';
            app.StatusLabel.Text = '● STOPPED';
            app.StatusLabel.FontColor = [0.95 0.35 0.35];
            
            if isempty(app.OccupancyMap)
                uialert(app.UIFigure, 'Lütfen önce bir harita oluşturun!', 'Hata');
                return;
            end
            
            app.CurrentSeed = randi(100000);
            rng(app.CurrentSeed);
            app.AppSubTitleLabel.Text = sprintf('Seed: %d | Multi-Agent Pathfinding System', app.CurrentSeed);
            
            occMat = app.OccupancyMap.occupancyMatrix;
            [rows, cols] = find(occMat == 0); 
            freeSpaces = grid2world(app.OccupancyMap, [rows, cols]);
            
            if size(freeSpaces, 1) < app.NumRobots * 2
                uialert(app.UIFigure, 'Haritada yeterli boş alan yok!', 'Hata');
                return;
            end
            
            app.RobotStartPos = zeros(app.NumRobots, 2);
            app.RobotGoalPos  = zeros(app.NumRobots, 2);
            app.RobotColors = lines(app.NumRobots);
            
            mapW = app.OccupancyMap.XWorldLimits(2);
            mapH = app.OccupancyMap.YWorldLimits(2);
            maxDiag = sqrt(mapW^2 + mapH^2);
            
            for i = 1:app.NumRobots
                randIdx = randi(size(freeSpaces, 1));
                startPt = freeSpaces(randIdx, :);
                app.RobotStartPos(i, :) = startPt;
                
                dists = sqrt(sum((freeSpaces - startPt).^2, 2));
                
                if i <= 3
                    minD = maxDiag * 0.10; maxD = maxDiag * 0.20;
                elseif i <= 7
                    minD = maxDiag * 0.30; maxD = maxDiag * 0.50;
                else
                    minD = maxDiag * 0.60; maxD = maxDiag * 1.00;
                end
                
                candidateIndices = find(dists >= minD & dists <= maxD);
                if isempty(candidateIndices), candidateIndices = find(dists > 5); end
                
                goalIdxInCandidates = randi(length(candidateIndices));
                actualIdx = candidateIndices(goalIdxInCandidates);
                goalPt = freeSpaces(actualIdx, :);
                
                app.RobotGoalPos(i, :) = goalPt;
            end
            
            planRoutes(app);
            updateMapDisplay(app);
        end

        
        function planRoutes(app)
            if isempty(app.OccupancyMap) || isempty(app.RobotStartPos)
                return;
            end
            
            app.RobotPaths = cell(app.NumRobots, 1);
            plannerType = app.PlannerDropDown.Value;
            gridMap = app.OccupancyMap.occupancyMatrix; 
            
            for i = 1:app.NumRobots
                startWorld = app.RobotStartPos(i, :);
                goalWorld  = app.RobotGoalPos(i, :);
                
                startIdx = world2grid(app.OccupancyMap, startWorld);
                goalIdx  = world2grid(app.OccupancyMap, goalWorld);
                
                pathIndices = [];
                switch plannerType
                    case 'A* (Standard Optimal)'
                        planner = plannerAStarGrid(app.OccupancyMap);
                        try pathIndices = plan(planner, startIdx, goalIdx); catch, end
                    case 'Breadth-First Search (BFS)'
                        pathIndices = bfs_grid(gridMap, startIdx, goalIdx);
                    case 'Depth-First Search (DFS)'
                        pathIndices = dfs_grid(gridMap, startIdx, goalIdx);
                    otherwise
                        planner = plannerAStarGrid(app.OccupancyMap);
                        try pathIndices = plan(planner, startIdx, goalIdx); catch, end
                end
                
                if ~isempty(pathIndices)
                    pathWorld = grid2world(app.OccupancyMap, pathIndices);
                    app.RobotPaths{i} = pathWorld;
                else
                    app.RobotPaths{i} = [];
                end
            end
        end

       
        function runSimulation(app)
            app.SimulationRunning = true;
            
            app.RobotPathHistory = cell(app.NumRobots, 1);
            for i = 1:app.NumRobots
                if ~isempty(app.RobotPaths{i})
                    startPos = app.RobotPaths{i}(1, :);
                    app.RobotPathHistory{i} = [startPos, 0];
                end
            end
            
            if isempty(app.RobotStepIndices) || length(app.RobotStepIndices) ~= app.NumRobots
                app.RobotStepIndices = ones(app.NumRobots, 1);
                app.SimTime = 0;
            end
            
            app.stuckCounters = zeros(app.NumRobots, 1);
            totalDeadlocks = 0; 
            
            while app.SimulationRunning
                allRobotsFinished = true;
                finishedCount = 0;
                currentTime = app.SimTime + 0.1;
                activePositions = app.RobotStartPos; 
                waitsFor = zeros(app.NumRobots, 1);
                
                if all(app.stuckCounters < 5)
                    app.StatusLabel.Text = '● RUNNING';
                    app.StatusLabel.FontColor = [0.35 0.85 0.45];
                end

                for i = 1:app.NumRobots
                    path = app.RobotPaths{i};
                    currentStep = app.RobotStepIndices(i);
                    
                    if ~isempty(path) && currentStep < size(path, 1)
                        allRobotsFinished = false;
                        candidatePos = path(currentStep + 1, :);
                        
                        isBlocked = false;
                        blockingRobotID = 0;
                        
                        for j = 1:app.NumRobots
                            if i == j, continue; end
                            dist = norm(candidatePos - activePositions(j, :));
                            if dist < 0.1 
                                isBlocked = true;
                                blockingRobotID = j;
                                break; 
                            end
                        end
                        
                        if ~isBlocked
                            nextStep = currentStep + 1;
                            app.RobotStepIndices(i) = nextStep;
                            app.RobotStartPos(i, :) = candidatePos;
                            activePositions(i, :) = candidatePos;
                            
                            currentLog = app.RobotPathHistory{i};
                            app.RobotPathHistory{i} = [currentLog; candidatePos, currentTime];
                            app.stuckCounters(i) = 0; 
                        else
                            waitsFor(i) = blockingRobotID;
                            currentLog = app.RobotPathHistory{i};
                            app.RobotPathHistory{i} = [currentLog; app.RobotStartPos(i, :), currentTime];
                            app.stuckCounters(i) = app.stuckCounters(i) + 1;
                            
                            if app.stuckCounters(i) > 5
                                app.StatusLabel.Text = '⚠  RESOLVING';
                                app.StatusLabel.FontColor = [0.95 0.65 0.25];
                                success = attemptReplanning(app, i, activePositions);
                                if success
                                    app.stuckCounters(i) = 0;
                                    totalDeadlocks = totalDeadlocks + 1;
                                end
                            end
                        end
                    else
                        finishedCount = finishedCount + 1;
                    end
                end
                
                currentCycleDeadlocks = 0;
                for r = 1:app.NumRobots
                    targetID = waitsFor(r);
                    if targetID > 0
                        visited = []; curr = r;
                        while curr > 0 && ~ismember(curr, visited)
                            visited(end+1) = curr;
                            curr = waitsFor(curr);
                            if curr == r
                                currentCycleDeadlocks = currentCycleDeadlocks + 1;
                                break;
                            end
                        end
                    end
                end
                totalDeadlocks = totalDeadlocks + currentCycleDeadlocks;

                updateMapDisplay(app); 
                
                if app.CheckWaitFor.Value
                    hold(app.MapAxes, 'on');
                    arrowX = []; arrowY = []; arrowU = []; arrowV = [];
                    for r = 1:app.NumRobots
                        targetID = waitsFor(r);
                        if targetID > 0
                            p1 = app.RobotStartPos(r, :);
                            p2 = app.RobotStartPos(targetID, :);
                            arrowX(end+1) = p1(1); arrowY(end+1) = p1(2);
                            arrowU(end+1) = p2(1) - p1(1); arrowV(end+1) = p2(2) - p1(2);
                        end
                    end
                    if ~isempty(arrowX)
                        quiver(app.MapAxes, arrowX, arrowY, arrowU, arrowV, 0, 'Color', [0.95 0.75 0.25], 'LineWidth', 2.5, 'MaxHeadSize', 0.6);
                    end
                    hold(app.MapAxes, 'off');
                end
                
                app.SimTime = currentTime;
                app.MetricTimeValue.Text = sprintf('%.1fs', app.SimTime);
                app.MetricDoneValue.Text = sprintf('%d/%d', finishedCount, app.NumRobots);
                app.MetricDeadValue.Text = num2str(totalDeadlocks);
                
                drawnow; pause(0.1); 
                
                if allRobotsFinished
                    app.SimulationRunning = false;
                    app.StartSimulationBtn.Text = '✓  Simülasyon Tamamlandı';
                    app.StatusLabel.Text = '● COMPLETED';
                    app.StatusLabel.FontColor = [0.35 0.85 0.45];
                    uialert(app.UIFigure, 'Tüm robotlar hedefe ulaştı!', 'Tamamlandı');
                    break;
                end
            end
        end

       
        function showAnalysisReport(app)
            if isempty(app.RobotPathHistory) || isempty(app.RobotPathHistory{1})
                uialert(app.UIFigure, 'Analiz için veri yok. Lütfen önce simülasyonu çalıştırın.', 'Veri Yok');
                return;
            end
            
            reportFig = uifigure('Name', 'Analiz Raporu: Diferansiyel Sürüş Kinematiği');
            reportFig.Position = [200 150 900 650];
            reportFig.Color = [0.08 0.09 0.12];
            
            mainLayout = uigridlayout(reportFig);
            mainLayout.ColumnWidth = {'1x'};
            mainLayout.RowHeight = {70, '1x', '1x', '1x'};
            mainLayout.Padding = [20 20 20 20];
            mainLayout.RowSpacing = 15;
            
          
            headerPanel = uipanel(mainLayout);
            headerPanel.BackgroundColor = [0.12 0.14 0.18];
            headerPanel.BorderType = 'none';
            headerPanel.Layout.Row = 1;
            
            headerGrid = uigridlayout(headerPanel);
            headerGrid.ColumnWidth = {'1x'};
            headerGrid.RowHeight = {30, 25};
            headerGrid.Padding = [15 10 15 10];
            
            titleLbl = uilabel(headerGrid);
            titleLbl.Text = '📊 KİNEMATİK ANALİZ RAPORU';
            titleLbl.FontColor = [0.35 0.65 1.0];
            titleLbl.FontSize = 18;
            titleLbl.FontWeight = 'bold';
            titleLbl.Layout.Row = 1;
            
            dd = uidropdown(headerGrid);
            robotNames = {};
            for i = 1:app.NumRobots
                robotNames{end+1} = sprintf('🤖 Robot %d', i);
            end
            dd.Items = robotNames;
            dd.BackgroundColor = [0.15 0.17 0.22];
            dd.FontColor = 'white';
            dd.FontSize = 12;
            dd.Layout.Row = 2;
            
            ax1 = uiaxes(mainLayout); ax1.Layout.Row = 2;
            ax2 = uiaxes(mainLayout); ax2.Layout.Row = 3;
            ax3 = uiaxes(mainLayout); ax3.Layout.Row = 4;
            
            setupAxisTheme(ax1, 'Doğrusal Hız (v)', 'Hız (m/s)');
            setupAxisTheme(ax2, 'Açısal Hız (ω)', 'Hız (rad/s)');
            setupAxisTheme(ax3, 'Tekerlek Hızları', 'Hız (m/s)');
            
            function updatePlots(robotIdx)
                data = app.RobotPathHistory{robotIdx};
                if isempty(data) || size(data, 1) < 2, return; end
                
                X = data(:, 1); Y = data(:, 2); T = data(:, 3);
                dt_vec = diff(T); dx = diff(X); dy = diff(Y);
                validIdx = dt_vec > 0;
                dt_vec = dt_vec(validIdx);
                dx = dx(validIdx);
                dy = dy(validIdx);
                plotTime = T(2:end);
                plotTime = plotTime(validIdx);
                
                dist = sqrt(dx.^2 + dy.^2);
                v = dist ./ dt_vec;
                
                theta = atan2(dy, dx);
                dTheta = zeros(size(theta));
                dTheta(1) = 0;
                for k = 2:length(theta)
                    diffVal = theta(k) - theta(k-1);
                    while diffVal > pi, diffVal = diffVal - 2*pi; end
                    while diffVal < -pi, diffVal = diffVal + 2*pi; end
                    dTheta(k) = diffVal;
                end
                w = dTheta ./ dt_vec;
                
                L = 0.5;
                v_Right = v + (L * w) / 2;
                v_Left  = v - (L * w) / 2;
                
                color = app.RobotColors(robotIdx, :);
                
                cla(ax1); cla(ax2); cla(ax3);
                
                plot(ax1, plotTime, v, 'LineWidth', 2.5, 'Color', color);
                yline(ax1, mean(v), '--', 'Ortalama', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
                
                plot(ax2, plotTime, w, 'LineWidth', 2.5, 'Color', [0.95 0.65 0.25]);
                
                plot(ax3, plotTime, v_Left, 'LineWidth', 2, 'Color', [0.35 0.85 0.45]);
                hold(ax3, 'on');
                plot(ax3, plotTime, v_Right, 'LineWidth', 2, 'Color', [0.35 0.65 1.0]);
                legend(ax3, {'Sol Tekerlek', 'Sağ Tekerlek'}, 'TextColor', 'white', 'Color', [0.12 0.14 0.18], 'EdgeColor', [0.25 0.27 0.32]);
                hold(ax3, 'off');
            end
            
            function setupAxisTheme(ax, t, y)
                ax.Color = [0.10 0.12 0.16];
                ax.XColor = [0.6 0.6 0.6];
                ax.YColor = [0.6 0.6 0.6];
                ax.GridColor = [0.25 0.27 0.32];
                ax.GridAlpha = 0.4;
                ax.LineWidth = 1;
                grid(ax, 'on');
                title(ax, t, 'Color', 'white', 'FontSize', 13, 'FontWeight', 'bold');
                xlabel(ax, 'Zaman (s)', 'Color', [0.8 0.8 0.8]);
                ylabel(ax, y, 'Color', [0.8 0.8 0.8]);
            end
            
            dd.ValueChangedFcn = @(src, event) updatePlots(find(strcmp(src.Value, robotNames)));
            updatePlots(1);
        end

        function updateMapDisplay(app)
            if isempty(app.OccupancyMap), return; end
            
            show(app.OccupancyMap, 'Parent', app.MapAxes);
            title(app.MapAxes, '');
            grid(app.MapAxes, 'on');
            app.MapAxes.XColor = [0.4 0.4 0.4];
            app.MapAxes.YColor = [0.4 0.4 0.4];
            app.MapAxes.GridColor = [0.2 0.22 0.28];
            app.MapAxes.GridAlpha = 0.3;
            colormap(app.MapAxes, [0.08 0.10 0.15; 0.25 0.28 0.35]);
            
            hold(app.MapAxes, 'on');
            
            if app.CheckPlanRoute.Value && ~isempty(app.RobotPaths)
                for i = 1:length(app.RobotPaths)
                    path = app.RobotPaths{i};
                    if ~isempty(path)
                        plot(app.MapAxes, path(:,1), path(:,2), ...
                            'LineStyle', '--', 'LineWidth', 2, ...
                            'Color', app.RobotColors(i, :));
                    end
                end
            end
            
            if ~isempty(app.RobotStartPos)
                for i = 1:size(app.RobotStartPos, 1)
                    c = app.RobotColors(i, :);
                    
                    plot(app.MapAxes, app.RobotStartPos(i, 1), app.RobotStartPos(i, 2), ...
                        'o', 'MarkerSize', 10, 'LineWidth', 2, ...
                        'MarkerEdgeColor', c, 'MarkerFaceColor', c);
                    
                    plot(app.MapAxes, app.RobotGoalPos(i, 1), app.RobotGoalPos(i, 2), ...
                        'x', 'MarkerSize', 12, 'LineWidth', 3, 'Color', c);
                end
            end
            
            hold(app.MapAxes, 'off');
        end

        function createRandomMap(app)
            w = 50; h = 50;
            mapM = false(w, h);
            curD = 0;
            targetD = 0.20;
            
            while curD < targetD
                cx = randi([2, w-5]);
                cy = randi([2, h-5]);
                
                if randi(2) == 1
                    mapM(cx:cx+randi([2,6]), cy:cy+randi([2,6])) = true;
                else
                    L = randi([4,10]);
                    T = randi([1,2]);
                    mapM(cx:min(cx+L,w), cy:min(cy+T,h)) = true;
                    mapM(cx:min(cx+T,w), cy:min(cy+L,h)) = true;
                end
                
                curD = sum(mapM(:)) / (w*h);
            end
            
            app.OccupancyMap = binaryOccupancyMap(mapM);
            app.RobotStartPos = [];
            app.RobotGoalPos = [];
            updateMapDisplay(app);
        end

        function createNarrowCorridorMap(app)
            w = 30; h = 30; 
            mapM = false(w, h);
            
            mapM(1,:) = true;
            mapM(end,:) = true;
            mapM(:,1) = true;
            mapM(:,end) = true;
            
            cols = [10, 20];
            for c = cols
                mapM(:, c) = true;
                mid = round(h/2);
                mapM(mid-1:mid+1, c) = false; 
            end
            
            row = 15;
            mapM(row, :) = true;
            midC1 = 10; midC2 = 20;
            mapM(row, midC1-1:midC1+1) = false;
            mapM(row, midC2-1:midC2+1) = false;
            
            midW = round(w/2);
            mapM(:, midW) = false; 
            
            app.OccupancyMap = binaryOccupancyMap(mapM);
            app.RobotStartPos = [];
            app.RobotGoalPos = [];
            updateMapDisplay(app);
        end
        
        function toggleSimulation(app)
            if app.SimulationRunning
                app.SimulationRunning = false; 
                app.StartSimulationBtn.Text = '▶  Devam Et';
                app.StatusLabel.Text = '● PAUSED';
                app.StatusLabel.FontColor = [0.95 0.65 0.25];
            else
                app.StartSimulationBtn.Text = '❚❚  Duraklat';
                app.StatusLabel.Text = '● RUNNING';
                app.StatusLabel.FontColor = [0.35 0.85 0.45];
                runSimulation(app);
            end
        end

        function createComponents(app)
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1400 800];
            app.UIFigure.Name = 'FleetManager Pro • Multi-Agent Pathfinding';
            app.UIFigure.Color = [0.05 0.06 0.10];
            
            app.MainGridLayout = uigridlayout(app.UIFigure);
            app.MainGridLayout.ColumnWidth = {320, '1x'};
            app.MainGridLayout.RowHeight = {'1x'};
            app.MainGridLayout.Padding = [0 0 0 0];
            app.MainGridLayout.ColumnSpacing = 0;

            app.LeftPanel = uipanel(app.MainGridLayout);
            app.LeftPanel.BackgroundColor = [0.10 0.12 0.16];
            app.LeftPanel.BorderType = 'none';
            
            app.LeftGrid = uigridlayout(app.LeftPanel);
            app.LeftGrid.ColumnWidth = {'1x'};
            app.LeftGrid.RowHeight = {50, 25, 15, 35, 80, 50, 15, 30, 25, 35, 25, 35, 15, 60, 15, 30, 28, 28, 28, '1x'};
            app.LeftGrid.Padding = [20 20 20 20];
            app.LeftGrid.RowSpacing = 8;
            
            app.AppTitleLabel = uilabel(app.LeftGrid);
            app.AppTitleLabel.Text = '⚡ FLEETMANAGER';
            app.AppTitleLabel.FontSize = 24;
            app.AppTitleLabel.FontWeight = 'bold';
            app.AppTitleLabel.FontColor = [0.35 0.65 1.0];
            app.AppTitleLabel.HorizontalAlignment = 'center';
            
            app.AppSubTitleLabel = uilabel(app.LeftGrid);
            app.AppSubTitleLabel.Text = 'Multi-Agent Pathfinding System';
            app.AppSubTitleLabel.FontSize = 11;
            app.AppSubTitleLabel.FontColor = [0.5 0.55 0.65];
            app.AppSubTitleLabel.HorizontalAlignment = 'center';
            
            divider1 = uilabel(app.LeftGrid);
            divider1.Text = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
            divider1.FontColor = [0.2 0.22 0.28];
            divider1.HorizontalAlignment = 'center';
            
            app.MapSectionLabel = uilabel(app.LeftGrid);
            app.MapSectionLabel.Text = '🗺️  HARİTA OLUŞTURMA';
            app.MapSectionLabel.FontWeight = 'bold';
            app.MapSectionLabel.FontColor = [0.85 0.85 0.85];
            app.MapSectionLabel.FontSize = 12;
            
            mapBtnGrid = uigridlayout(app.LeftGrid);
            mapBtnGrid.ColumnWidth = {'1x', '1x'};
            mapBtnGrid.RowHeight = {35, 35};
            mapBtnGrid.Padding = [0 0 0 0];
            mapBtnGrid.ColumnSpacing = 10;
            mapBtnGrid.RowSpacing = 10;
            
            app.RandomMapButton = uibutton(mapBtnGrid, 'push');
            app.RandomMapButton.Text = '🎲 Random';
            app.RandomMapButton.BackgroundColor = [0.18 0.20 0.26];
            app.RandomMapButton.FontColor = 'white';
            app.RandomMapButton.FontSize = 11;
            app.RandomMapButton.ButtonPushedFcn = @(btn,e) createRandomMap(app);
            app.RandomMapButton.Layout.Row = 1;
            app.RandomMapButton.Layout.Column = 1;
            
            app.DarKoridorButton = uibutton(mapBtnGrid, 'push');
            app.DarKoridorButton.Text = '🔲 Koridor';
            app.DarKoridorButton.BackgroundColor = [0.18 0.20 0.26];
            app.DarKoridorButton.FontColor = 'white';
            app.DarKoridorButton.FontSize = 11;
            app.DarKoridorButton.ButtonPushedFcn = @(btn,e) createNarrowCorridorMap(app);
            app.DarKoridorButton.Layout.Row = 1;
            app.DarKoridorButton.Layout.Column = 2;
            
            app.PlaceRobotsButton = uibutton(mapBtnGrid, 'push');
            app.PlaceRobotsButton.Text = '🤖 Robotları Yerleştir';
            app.PlaceRobotsButton.BackgroundColor = [0.25 0.28 0.35];
            app.PlaceRobotsButton.FontColor = 'white';
            app.PlaceRobotsButton.FontSize = 12;
            app.PlaceRobotsButton.FontWeight = 'bold';
            app.PlaceRobotsButton.ButtonPushedFcn = @(btn,e) placeRobots(app);
            app.PlaceRobotsButton.Layout.Row = 2;
            app.PlaceRobotsButton.Layout.Column = [1 2];
            
            robotPlacePanel = uipanel(app.LeftGrid);
            robotPlacePanel.BackgroundColor = [0.10 0.12 0.16];
            robotPlacePanel.BorderType = 'none';
            
            divider2 = uilabel(app.LeftGrid);
            divider2.Text = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
            divider2.FontColor = [0.2 0.22 0.28];
            divider2.HorizontalAlignment = 'center';
            
            app.PlanSectionLabel = uilabel(app.LeftGrid);
            app.PlanSectionLabel.Text = '⚙️  PLANLAMA & ÇÖZÜM';
            app.PlanSectionLabel.FontWeight = 'bold';
            app.PlanSectionLabel.FontColor = [0.85 0.85 0.85];
            app.PlanSectionLabel.FontSize = 12;
            
            app.PlannerDropDownLabel = uilabel(app.LeftGrid);
            app.PlannerDropDownLabel.Text = '1️⃣  Global Rota Planlayıcı';
            app.PlannerDropDownLabel.FontColor = [0.45 0.65 0.95];
            app.PlannerDropDownLabel.FontSize = 11;
            
            app.PlannerDropDown = uidropdown(app.LeftGrid);
            app.PlannerDropDown.Items = {'A* (Standard Optimal)', 'Breadth-First Search (BFS)', 'Depth-First Search (DFS)'};
            app.PlannerDropDown.BackgroundColor = [0.15 0.17 0.22];
            app.PlannerDropDown.FontColor = 'white';
            app.PlannerDropDown.FontSize = 11;
            
            app.CongestDropDownLabel = uilabel(app.LeftGrid);
            app.CongestDropDownLabel.Text = '2️⃣  Tıkanıklık Yönetimi';
            app.CongestDropDownLabel.FontColor = [0.95 0.65 0.25];
            app.CongestDropDownLabel.FontSize = 11;
            
            app.CongestDropDown = uidropdown(app.LeftGrid);
            app.CongestDropDown.Items = {'Reactive Re-planning (Standart)', 'Priority Wait'};
            app.CongestDropDown.BackgroundColor = [0.15 0.17 0.22];
            app.CongestDropDown.FontColor = 'white';
            app.CongestDropDown.FontSize = 11;
            
            divider3 = uilabel(app.LeftGrid);
            divider3.Text = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
            divider3.FontColor = [0.2 0.22 0.28];
            divider3.HorizontalAlignment = 'center';
            
            app.StartSimulationBtn = uibutton(app.LeftGrid, 'push');
            app.StartSimulationBtn.Text = '▶  Simülasyonu Başlat';
            app.StartSimulationBtn.BackgroundColor = [0.30 0.40 0.95];
            app.StartSimulationBtn.FontColor = 'white';
            app.StartSimulationBtn.FontSize = 14;
            app.StartSimulationBtn.FontWeight = 'bold';
            app.StartSimulationBtn.ButtonPushedFcn = @(btn,e) toggleSimulation(app);
            
            divider4 = uilabel(app.LeftGrid);
            divider4.Text = '━━━━━━━━━━━━━━━━━━━━━━━━━━━━';
            divider4.FontColor = [0.2 0.22 0.28];
            divider4.HorizontalAlignment = 'center';
            
            app.LayersLabel = uilabel(app.LeftGrid);
            app.LayersLabel.Text = '👁️  GÖRSEL KATMANLAR';
            app.LayersLabel.FontColor = [0.6 0.65 0.75];
            app.LayersLabel.FontSize = 11;
            app.LayersLabel.FontWeight = 'bold';
            
            app.CheckPlanRoute = uicheckbox(app.LeftGrid);
            app.CheckPlanRoute.Text = '  Planlanan Rota';
            app.CheckPlanRoute.FontColor = 'white';
            app.CheckPlanRoute.Value = true;
            app.CheckPlanRoute.FontSize = 11;
            
            app.CheckRealPath = uicheckbox(app.LeftGrid);
            app.CheckRealPath.Text = '  Gerçekleşen Yol';
            app.CheckRealPath.FontColor = 'white';
            app.CheckRealPath.Value = true;
            app.CheckRealPath.FontSize = 11;
            
            app.CheckWaitFor = uicheckbox(app.LeftGrid);
            app.CheckWaitFor.Text = '  Wait-For Graph';
            app.CheckWaitFor.FontColor = 'white';
            app.CheckWaitFor.FontSize = 11;
            
            app.RightPanel = uipanel(app.MainGridLayout);
            app.RightPanel.BackgroundColor = [0.05 0.06 0.10];
            app.RightPanel.BorderType = 'none';
            
            app.RightGrid = uigridlayout(app.RightPanel);
            app.RightGrid.ColumnWidth = {15, '1x', 15};
            app.RightGrid.RowHeight = {70, 120, '1x', 20};
            app.RightGrid.Padding = [0 0 0 0];
            app.RightGrid.RowSpacing = 12;
            
            app.TopBarGrid = uigridlayout(app.RightGrid);
            app.TopBarGrid.Layout.Row = 1;
            app.TopBarGrid.Layout.Column = 2;
            app.TopBarGrid.ColumnWidth = {'1x', 200};
            app.TopBarGrid.RowHeight = {'1x'};
            app.TopBarGrid.Padding = [15 15 15 10];
            
            app.StatusLabel = uilabel(app.TopBarGrid);
            app.StatusLabel.Text = '● STOPPED';
            app.StatusLabel.FontColor = [0.95 0.35 0.35];
            app.StatusLabel.HorizontalAlignment = 'left';
            app.StatusLabel.FontSize = 16;
            app.StatusLabel.FontWeight = 'bold';
            app.StatusLabel.Layout.Column = 1;
            
            app.AnalysisButton = uibutton(app.TopBarGrid, 'push');
            app.AnalysisButton.Text = '📊 Analiz Raporu';
            app.AnalysisButton.BackgroundColor = [0.18 0.20 0.26];
            app.AnalysisButton.FontColor = 'white';
            app.AnalysisButton.FontSize = 12;
            app.AnalysisButton.ButtonPushedFcn = @(btn,e) showAnalysisReport(app);
            app.AnalysisButton.Layout.Column = 2;
            
            app.MetricsPanel = uipanel(app.RightGrid);
            app.MetricsPanel.BackgroundColor = [0.05 0.06 0.10];
            app.MetricsPanel.BorderType = 'none';
            app.MetricsPanel.Layout.Row = 2;
            app.MetricsPanel.Layout.Column = 2;
            
            app.MetricsGrid = uigridlayout(app.MetricsPanel);
            app.MetricsGrid.ColumnWidth = {'1x', 12, '1x', 12, '1x', 100};
            app.MetricsGrid.RowHeight = {'1x'};
            app.MetricsGrid.Padding = [15 0 15 0];
            
            app.MetricCard1 = uipanel(app.MetricsGrid);
            app.MetricCard1.BackgroundColor = [0.12 0.14 0.18];
            app.MetricCard1.BorderType = 'none';
            app.MetricCard1.Layout.Column = 1;
            
            card1Grid = uigridlayout(app.MetricCard1);
            card1Grid.ColumnWidth = {'1x'};
            card1Grid.RowHeight = {28, '1x'};
            card1Grid.Padding = [12 12 12 12];
            
            app.MetricTimeLabel = uilabel(card1Grid);
            app.MetricTimeLabel.Text = '⏱️  SİMÜLASYON SÜRESİ';
            app.MetricTimeLabel.FontColor = [0.55 0.60 0.70];
            app.MetricTimeLabel.FontSize = 10;
            app.MetricTimeLabel.FontWeight = 'bold';
            app.MetricTimeLabel.Layout.Row = 1;
            
            app.MetricTimeValue = uilabel(card1Grid);
            app.MetricTimeValue.Text = '0.0s';
            app.MetricTimeValue.FontSize = 32;
            app.MetricTimeValue.FontWeight = 'bold';
            app.MetricTimeValue.FontColor = 'white';
            app.MetricTimeValue.HorizontalAlignment = 'center';
            app.MetricTimeValue.VerticalAlignment = 'center';
            app.MetricTimeValue.Layout.Row = 2;
            
            app.MetricCard2 = uipanel(app.MetricsGrid);
            app.MetricCard2.BackgroundColor = [0.12 0.14 0.18];
            app.MetricCard2.BorderType = 'none';
            app.MetricCard2.Layout.Column = 3;
            
            card2Grid = uigridlayout(app.MetricCard2);
            card2Grid.ColumnWidth = {'1x'};
            card2Grid.RowHeight = {28, '1x'};
            card2Grid.Padding = [12 12 12 12];
            
            app.MetricDeadLabel = uilabel(card2Grid);
            app.MetricDeadLabel.Text = '⚠️  DEADLOCK / ÇARPIŞMA';
            app.MetricDeadLabel.FontColor = [0.55 0.60 0.70];
            app.MetricDeadLabel.FontSize = 10;
            app.MetricDeadLabel.FontWeight = 'bold';
            app.MetricDeadLabel.Layout.Row = 1;
            
            app.MetricDeadValue = uilabel(card2Grid);
            app.MetricDeadValue.Text = '0';
            app.MetricDeadValue.FontSize = 32;
            app.MetricDeadValue.FontWeight = 'bold';
            app.MetricDeadValue.FontColor = [0.95 0.65 0.25];
            app.MetricDeadValue.HorizontalAlignment = 'center';
            app.MetricDeadValue.VerticalAlignment = 'center';
            app.MetricDeadValue.Layout.Row = 2;
            
            app.MetricCard3 = uipanel(app.MetricsGrid);
            app.MetricCard3.BackgroundColor = [0.12 0.14 0.18];
            app.MetricCard3.BorderType = 'none';
            app.MetricCard3.Layout.Column = 5;
            
            card3Grid = uigridlayout(app.MetricCard3);
            card3Grid.ColumnWidth = {'1x'};
            card3Grid.RowHeight = {28, '1x'};
            card3Grid.Padding = [12 12 12 12];
            
            app.MetricDoneLabel = uilabel(card3Grid);
            app.MetricDoneLabel.Text = '✓  TAMAMLANAN';
            app.MetricDoneLabel.FontColor = [0.55 0.60 0.70];
            app.MetricDoneLabel.FontSize = 10;
            app.MetricDoneLabel.FontWeight = 'bold';
            app.MetricDoneLabel.Layout.Row = 1;
            
            app.MetricDoneValue = uilabel(card3Grid);
            app.MetricDoneValue.Text = '0/0';
            app.MetricDoneValue.FontSize = 32;
            app.MetricDoneValue.FontWeight = 'bold';
            app.MetricDoneValue.FontColor = [0.35 0.85 0.45];
            app.MetricDoneValue.HorizontalAlignment = 'center';
            app.MetricDoneValue.VerticalAlignment = 'center';
            app.MetricDoneValue.Layout.Row = 2;
            
            app.MapAxes = uiaxes(app.RightGrid);
            app.MapAxes.Layout.Row = 3;
            app.MapAxes.Layout.Column = 2;
            app.MapAxes.Color = [0.08 0.10 0.15];
            app.MapAxes.XColor = [0.3 0.35 0.45];
            app.MapAxes.YColor = [0.3 0.35 0.45];
            app.MapAxes.Box = 'on';
            app.MapAxes.XTick = [];
            app.MapAxes.YTick = [];
            app.MapAxes.LineWidth = 1.5;
            hold(app.MapAxes, 'on');
            axis(app.MapAxes, [0 12 0 12]);
            grid(app.MapAxes, 'on');
            app.MapAxes.GridColor = [0.2 0.22 0.28];
            app.MapAxes.GridAlpha = 0.3;
            
            app.LegendPanel = uipanel(app.UIFigure);
            app.LegendPanel.BackgroundColor = [0.12 0.14 0.18];
            app.LegendPanel.BorderType = 'none';
            app.LegendPanel.Position = [1180 580 180 150];
            
            app.LegendGrid = uigridlayout(app.LegendPanel);
            app.LegendGrid.ColumnWidth = {30, '1x'};
            app.LegendGrid.RowHeight = {25, 28, 28, 28, 28};
            app.LegendGrid.Padding = [15 12 15 12];
            app.LegendGrid.RowSpacing = 5;
            
            legendTitle = uilabel(app.LegendGrid);
            legendTitle.Text = 'GÖSTERGE';
            legendTitle.FontColor = [0.75 0.80 0.90];
            legendTitle.FontSize = 11;
            legendTitle.FontWeight = 'bold';
            legendTitle.Layout.Row = 1;
            legendTitle.Layout.Column = [1 2];
            
            function createLegendItem(l, r, c, t)
                lblI = uilabel(l);
                lblI.Text = '●';
                lblI.FontColor = c;
                lblI.FontSize = 16;
                lblI.Layout.Row = r;
                lblI.Layout.Column = 1;
                
                lblT = uilabel(l);
                lblT.Text = t;
                lblT.FontColor = [0.85 0.85 0.85];
                lblT.FontSize = 11;
                lblT.Layout.Row = r;
                lblT.Layout.Column = 2;
            end
            
            createLegendItem(app.LegendGrid, 2, [0.25 0.28 0.35], 'Engel');
            createLegendItem(app.LegendGrid, 3, [0.35 0.65 1.0], 'Robot');
            createLegendItem(app.LegendGrid, 4, [0.35 0.85 0.45], 'Hedef');
            createLegendItem(app.LegendGrid, 5, [0.95 0.75 0.25], 'Bekleme');
            
            app.UIFigure.Visible = 'on';
        end
    end

    methods (Access = public)
        function app = FleetManagerApp
            createComponents(app)
        end
        function delete(app)
            delete(app.UIFigure)
        end
    end
end


function path = bfs_grid(gridMap, startNode, goalNode)
    [rows, cols] = size(gridMap);
    q = startNode; 
    cameFrom = zeros(rows, cols, 2); 
    visited = false(rows, cols);
    visited(startNode(1), startNode(2)) = true;
    found = false;
    
    while ~isempty(q)
        current = q(1, :);
        q(1,:) = [];
        
        if isequal(current, goalNode)
            found = true;
            break;
        end
        
        dirs = [0 1; 0 -1; 1 0; -1 0];
        for k = 1:4
            next = current + dirs(k, :);
            if next(1) > 0 && next(1) <= rows && next(2) > 0 && next(2) <= cols
                if ~visited(next(1), next(2)) && ~gridMap(next(1), next(2))
                    visited(next(1), next(2)) = true;
                    cameFrom(next(1), next(2), :) = current;
                    q(end+1, :) = next;
                end
            end
        end
    end
    
    path = [];
    if found
        curr = goalNode;
        while ~isequal(curr, startNode)
            path = [curr; path];
            prev = squeeze(cameFrom(curr(1), curr(2), :))';
            curr = prev;
        end
        path = [startNode; path];
    end
end

function path = dfs_grid(gridMap, startNode, goalNode)
    [rows, cols] = size(gridMap);
    stack = startNode;
    cameFrom = zeros(rows, cols, 2);
    visited = false(rows, cols);
    visited(startNode(1), startNode(2)) = true;
    found = false;
    
    while ~isempty(stack)
        current = stack(end, :);
        stack(end,:) = []; 
        
        if isequal(current, goalNode)
            found = true;
            break;
        end
        
        dirs = [0 1; 1 0; 0 -1; -1 0]; 
        for k = 1:4
            next = current + dirs(k, :);
            if next(1) > 0 && next(1) <= rows && next(2) > 0 && next(2) <= cols
                if ~visited(next(1), next(2)) && ~gridMap(next(1), next(2))
                    visited(next(1), next(2)) = true;
                    cameFrom(next(1), next(2), :) = current;
                    stack(end+1, :) = next;
                end
            end
        end
    end
    
    path = [];
    if found
        curr = goalNode;
        while ~isequal(curr, startNode)
            path = [curr; path];
            prev = squeeze(cameFrom(curr(1), curr(2), :))';
            curr = prev;
        end
        path = [startNode; path];
    end
end