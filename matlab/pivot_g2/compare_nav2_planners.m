function comparisonResult = compare_nav2_planners(mapIndex,scenarioIndex)
%COMPARE_NAV2_PLANNERS Ten cu, nay chuyen sang benchmark hau xu ly dung.
% Global-planner comparison cu thay doi dong thoi planner va hinh hoc nen
% khong phai thiet ke chinh de danh gia dong gop pivot-arc.
if nargin<1,mapIndex=1;end
if nargin<2,scenarioIndex=1;end
warning('Comparison:RenamedEntry', ...
    ['compare_nav2_planners nay goi benchmark hau xu ly voi THETA_STAR. ' ...
     'Nen dung truc tiep compare_path_postprocessors.']);
comparisonResult=compare_path_postprocessors('THETA_STAR',mapIndex,scenarioIndex);
end
