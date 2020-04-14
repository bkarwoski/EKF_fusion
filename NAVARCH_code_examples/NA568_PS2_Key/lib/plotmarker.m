function h = plotmarker(markerPos, color)

h = plot(markerPos(1), markerPos(2), '.', ...
         'linewidth', 2, 'Color', color);

