function plot_battery_histograms(metrics, vehicle)
    % plot_battery_histograms - Generate comprehensive battery health visualizations
    %
    % Inputs:
    %   metrics  - Metrics struct from calculate_battery_metrics()
    %   vehicle  - Vehicle parameters struct
    %
    % Creates a multi-panel figure with:
    %   1. DoD Histogram
    %   2. SOH Degradation over Vehicle Lifetime
    %   3. Cycle Count Breakdown
    %   4. Battery Efficiency Summary Dashboard
    
    % Create new figure
    figure('Name', 'Battery Health Dashboard', 'NumberTitle', 'off', ...
           'Position', [100, 100, 1400, 900]);
    
    %% Subplot 1: Depth of Discharge Histogram
    subplot(2, 2, 1);
    
    % Create bar chart
    bar(metrics.dod_histogram.bin_centers, metrics.dod_histogram.counts, 'FaceColor', [0.2, 0.6, 0.8]);
    
    xlabel('Depth of Discharge (%)');
    ylabel('Number of Cycles');
    title('Depth of Discharge (DoD) Distribution');
    grid on;
    
    % Add statistics text
    avg_dod = mean(metrics.dod_data);
    max_dod = max(metrics.dod_data);
    
    text_str = sprintf('Avg DoD: %.1f%%\nMax DoD: %.1f%%', avg_dod, max_dod);
    text(0.65, 0.95, text_str, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontSize', 10, ...
         'BackgroundColor', 'w', 'EdgeColor', 'k');
    
    % Color zones
    hold on;
    % Green zone: 0-40% (healthy)
    patch([0 40 40 0], [0 0 max(ylim) max(ylim)], [0.8 1 0.8], ...
          'FaceAlpha', 0.1, 'EdgeColor', 'none');
    % Yellow zone: 40-80% (moderate)
    patch([40 80 80 40], [0 0 max(ylim) max(ylim)], [1 1 0.8], ...
          'FaceAlpha', 0.1, 'EdgeColor', 'none');
    % Red zone: 80-100% (high stress)
    patch([80 100 100 80], [0 0 max(ylim) max(ylim)], [1 0.8 0.8], ...
          'FaceAlpha', 0.1, 'EdgeColor', 'none');
    hold off;
    
    %% Subplot 2: SOH Degradation over Vehicle Lifetime
    subplot(2, 2, 2);
    
    % Plot SOH vs km
    yyaxis left
    plot(metrics.soh_projection.km / 1000, metrics.soh_projection.soh, ...
         'LineWidth', 2, 'Color', [0.1, 0.5, 0.2]);
    ylabel('State of Health (%)');
    ylim([75, 105]);
    
    % Add EOL line
    hold on;
    if isfield(vehicle, 'battery') && isfield(vehicle.battery, 'soh_eol')
        soh_eol = vehicle.battery.soh_eol;
    else
        soh_eol = 80;
    end
    
    plot([0, max(metrics.soh_projection.km)/1000], [soh_eol, soh_eol], ...
         'r--', 'LineWidth', 1.5);
    text(max(metrics.soh_projection.km)/2000, soh_eol + 2, ...
         sprintf('EOL Threshold (%.0f%%)', soh_eol), ...
         'Color', 'r', 'FontSize', 9);
    
    % Mark current position
    plot(0, metrics.soh_current, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(max(metrics.soh_projection.km)/20000, metrics.soh_current + 1, ...
         'Current', 'Color', 'g', 'FontSize', 9);
    hold off;
    
    yyaxis right
    plot(metrics.soh_projection.km / 1000, metrics.soh_projection.years, ...
         'LineWidth', 1.5, 'Color', [0.6, 0.3, 0.7], 'LineStyle', ':');
    ylabel('Years of Operation');
    
    xlabel('Distance Traveled (1000 km)');
    title('Battery State of Health Projection');
    grid on;
    legend('SOH', sprintf('EOL @ %.0f%%', soh_eol), '', 'Years', ...
           'Location', 'southwest');
    
    %% Subplot 3: Cycle Count Breakdown by DoD Range
    subplot(2, 2, 3);
    
    % Create pie chart or bar chart
    bar(metrics.cycle_breakdown.counts, 'FaceColor', 'flat');
    
    % Color code by stress level
    colormap([0.2 0.8 0.2;   % Green: 0-20%
              0.5 0.9 0.3;   % Light green: 20-40%
              1.0 0.9 0.2;   % Yellow: 40-60%
              1.0 0.6 0.2;   % Orange: 60-80%
              0.9 0.2 0.2]); % Red: 80-100%
    
    set(gca, 'XTickLabel', metrics.cycle_breakdown.dod_ranges);
    xlabel('DoD Range');
    ylabel('Equivalent Full Cycles (EFC)');
    title(sprintf('Cycle Count Breakdown (Total: %.3f EFC)', metrics.cycle_count));
    grid on;
    
    % Rotate x-axis labels for readability
    xtickangle(45);
    
    %% Subplot 4: Battery Efficiency Summary Dashboard
    subplot(2, 2, 4);
    axis off;
    
    % Create text-based dashboard
    dashboard_text = {
        '\fontsize{14}\bf Battery Efficiency Summary', ...
        '', ...
        sprintf('\\fontsize{11}\\bf Energy Efficiency:       \\rm%.2f Wh/km', metrics.wh_per_km), ...
        '', ...
        sprintf('\\fontsize{11}\\bf Current Cycle Count:     \\rm%.4f EFC', metrics.cycle_count), ...
        '', ...
        sprintf('\\fontsize{11}\\bf Battery Lifetime:'), ...
        sprintf('\\fontsize{10}  • Distance:  \\rm%.0f km (%.0f miles)', ...
                metrics.battery_lifetime_km, metrics.battery_lifetime_km * 0.621371), ...
        sprintf('\\fontsize{10}  • Duration:  \\rm%.1f years', metrics.battery_lifetime_years), ...
        '', ...
        sprintf('\\fontsize{11}\\bf Expected Replacements:  \\rm%d times', metrics.battery_replacements), ...
        '', ...
        sprintf('\\fontsize{11}\\bf Lifetime Throughput:     \\rm%.0f kWh', metrics.lifetime_kwh_throughput), ...
        '', ...
        sprintf('\\fontsize{11}\\bf Current SOH:              \\rm%.2f%%', metrics.soh_current), ...
        '', ...
        sprintf('\\fontsize{11}\\bf SOH Degradation Rate:    \\rm%.3f%%/1000km', metrics.soh_degradation_rate), ...
        '', ...
        '\fontsize{9}\it Metrics based on extrapolation from current simulation cycle'
    };
    
    % Display text
    text(0.1, 0.95, dashboard_text, ...
         'VerticalAlignment', 'top', ...
         'HorizontalAlignment', 'left', ...
         'FontName', 'FixedWidth', ...
         'Interpreter', 'tex');
    
    % Add colored background box
    rectangle('Position', [0.05, 0.05, 0.9, 0.9], ...
              'FaceColor', [0.95, 0.98, 1], ...
              'EdgeColor', [0.2, 0.4, 0.8], ...
              'LineWidth', 2);
    uistack(findobj(gca, 'Type', 'rectangle'), 'bottom');
    
    %% Add overall title
    sgtitle('Battery Health & Efficiency Analysis', ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    fprintf('Battery health visualizations generated.\n');
end
