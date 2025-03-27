function cost = path_cost(g)

[T, N] = size(g);
r0 = rand;
path = zeros(T,1);
path(1) = 1+floor(N*r0);
cost = g(1,path(1));

for k = 1:(T-1)
    
    path(k+1) = path(k)+(-1+floor(3*rand));
    
    if(path(k+1) == N+1)
        
        path(k+1) = 1;
    end
    if(path(k+1) == 0)
        
        path(k+1) = N;
    end
    
    cost = cost+g(k+1,path(k+1));
end

