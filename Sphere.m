function z=Sphere(x)

    global NFE ;
    if isempty( NFE)
        NFE = 0 ;
    end
    NFE = NFE + 1;
    z=sum(x.^2); % دونه دونه میگیرد و مربع میکند و در اخر جمع میکنه .
    
end