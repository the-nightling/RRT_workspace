function [A,B] = get_A_B_for_sys()

    b = 0.1;
    
    A = [0,1;   ...
         0,-b];

    B = [0;   ...
         1];

end
