function potential = computeCostmapPotential(occupancy,resolution,decayDistance)
%COMPUTECOSTMAPPOTENTIAL Xap xi distance transform chamfer khong toolbox.
occupancy=logical(occupancy);
[rows,columns]=size(occupancy);
distance=inf(rows,columns);
distance(occupancy)=0;
diagonal=sqrt(2);
for pass=1:2
    for row=1:rows
        for column=1:columns
            value=distance(row,column);
            if row>1,value=min(value,distance(row-1,column)+1);end
            if column>1,value=min(value,distance(row,column-1)+1);end
            if row>1&&column>1,value=min(value,distance(row-1,column-1)+diagonal);end
            if row>1&&column<columns,value=min(value,distance(row-1,column+1)+diagonal);end
            distance(row,column)=value;
        end
    end
    distance=rot90(distance,2);
end
distance=rot90(distance,2);
distanceMeters=distance*resolution;
potential=exp(-distanceMeters/max(decayDistance,eps));
potential(occupancy)=1;
potential(~isfinite(potential))=0;
end
