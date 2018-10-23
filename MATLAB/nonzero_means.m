function [ means ] = nonzero_means( A )

means = zeros(1, size(A, 2));
for col = 1:size(A,2)
    means(col) = mean(nonzeros(A(:,col)));
end

end

