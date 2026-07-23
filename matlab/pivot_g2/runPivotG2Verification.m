function reports = runPivotG2Verification()
%RUNPIVOTG2VERIFICATION Chay toan bo regression truoc benchmark.
reports=struct();
reports.adaptivePivotG2=verifyAdaptivePivotG2Regression();
reports.postprocessor=verifyPostprocessorBenchmarkRegression();
reports.k2AndNoCorner=verifyK2AndNoCornerRegression();
fprintf('PIVOT-G2 VERIFICATION: ALL PASS\n');
end
