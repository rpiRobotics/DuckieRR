function im = DuckieImageToImage(dim)
    r = reshape(dim.data(1:3:end),dim.width, dim.height)';
    g = reshape(dim.data(2:3:end),dim.width, dim.height)';
    b = reshape(dim.data(3:3:end),dim.width, dim.height)';
    
    im = cat(3,r,g,b);
end