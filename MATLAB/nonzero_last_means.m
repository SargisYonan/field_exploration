function [ mean_ends ] = nonzero_last_means( A )

ends = zeros(1, size(A,1));
for row = 1 : size(A,1)
    
    nz = nonzeros(A(row,:));
    ends(row) = nz(end);

end

mean_ends = mean(ends);

