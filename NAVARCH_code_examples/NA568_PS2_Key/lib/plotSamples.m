function plotSamples(samples)

WAS_HOLD = ishold;

if ~WAS_HOLD
    hold on;
end

numSamples = size(samples, 2);

for s = 1:numSamples
    plotmarker(samples(1:2,s), 'y');
end

if ~WAS_HOLD
    hold off;
end
