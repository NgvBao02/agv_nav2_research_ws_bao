function [smoothed,info] = nav2SavitzkyGolaySmootherEquivalent( ...
        path,map,config,comparison)
%NAV2SAVITZKYGOLAYSMOOTHEREQUIVALENT Tich chap dung cua so Nav2.
p=comparison.savitzkyGolay;smoothed=path;originalInput=path;
if mod(p.windowSize,2)==0||p.windowSize<3
    error('Savitzky-Golay windowSize phai le va >=3.');
end
if p.polynomialOrder>=p.windowSize
    error('polynomialOrder phai nho hon windowSize.');
end
info=struct('plugin','nav2_smoother::SavitzkyGolaySmoother', ...
    'implementation','MATLAB_EQUIVALENT','iterations',0, ...
    'refinementPasses',0,'footprintRollback',false, ...
    'maximumDisplacement',0,'coreTime',0,'validation',struct(),'coefficients',[], ...
    'message','');
if size(path,1)<=p.windowSize+2
    info.message=sprintf('Nav2 bo qua segment <= window_size+2 (%d diem).', ...
        p.windowSize+2);
    info.validation=validatePostprocessedPath(smoothed,map,config, ...
        comparison.reference.sampleSpacing);return;
end
half=(p.windowSize-1)/2;abscissa=(-half:half).';
vandermonde=zeros(p.windowSize,p.polynomialOrder+1);
for degree=0:p.polynomialOrder
    vandermonde(:,degree+1)=abscissa.^degree;
end
coefficients=pinv(vandermonde);
filterRow=coefficients(1,:);info.coefficients=filterRow;
passes=1;if p.doRefinement,passes=passes+p.refinementCount;end
coreTimer=tic;
for pass=1:passes
    source=smoothed;next=source;
    for i=2:size(source,1)-1
        indices=min(size(source,1),max(1,i+(-half:half)));
        next(i,1)=filterRow*source(indices,1);
        next(i,2)=filterRow*source(indices,2);
    end
    next(1,:)=source(1,:);next(end,:)=source(end,:);
    smoothed=next;
end
info.coreTime=toc(coreTimer);
info.iterations=passes;info.refinementPasses=passes;
validation=validatePostprocessedPath(smoothed,map,config, ...
    comparison.reference.sampleSpacing);
if ~validation.safe
    smoothed=originalInput;info.footprintRollback=true;
    validation=validatePostprocessedPath(smoothed,map,config, ...
        comparison.reference.sampleSpacing);
    info.message='Fallback ve path goc do footprint/clearance.';
else
    info.message='OK';
end
info.validation=validation;
info.maximumDisplacement=max(hypot(smoothed(:,1)-originalInput(:,1), ...
    smoothed(:,2)-originalInput(:,2)));
end
