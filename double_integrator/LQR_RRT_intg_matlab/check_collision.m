function isColliding = check_collision(path)

    isColliding = 0;
    for i=1:size(path,1)
        if((path(i,1) < -4))
            isColliding = 1;
            return;
        end
        
        if((path(i,1) > 10))
            isColliding = 1;
            return;
        end
        
        if((path(i,3) < -8))
            isColliding = 1;
            return;
        end
        
        if((path(i,3) > 10))
            isColliding = 1;
            return;
        end
        
        if((path(i,1) > 1) && (path(i,1) < 3) && (path(i,3) > -2) && (path(i,3) < 8))
            isColliding = 1;
            return;
        end
        
        if((path(i,1) > 5) && (path(i,1) < 7) && (path(i,3) > -8) && (path(i,3) < 2))
            isColliding = 1;
            return;
        end
    end
    
    return;
end
