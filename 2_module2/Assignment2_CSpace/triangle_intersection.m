function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag = true;

for i=1:3
    for k =1:3
        if P1(i,1) == P2(k,1) && P1(i,2) == P2(k,2)
            flag = true;
            9999999
            return
        end
    end
end


%check P1
for i=1:3
    p3 = i; %used to check "sign" 
    p1 = mod(i, 3) + 1;   % used for line calc
    p2 = mod(i+1, 3) + 1; % used for line calc
    
    if P1(p2,1) == P1(p1,1) %x2 == x1
        x_thrs = P1(p2,1);
        temp_sign = x_thrs > P1(p3,1);

        if (x_thrs >= P2(1,1)) ~= temp_sign  && ...
                (x_thrs >= P2(2,1)) ~= temp_sign && ...
                (x_thrs >= P2(3,1)) ~= temp_sign
            flag = false;
            111
            return;
        end
    else
        m = (P1(p2,2) - P1(p1,2)) / (P1(p2,1) - P1(p1,1));
        q = P1(p1,2) - m * P1(p1,1);
        temp_sign = (P1(p3,2) - m*P1(p3,1) - q) > 0;

        if (P2(1,2) - m*P2(1,1) - q > 0) ~= temp_sign  && ...
                (P2(2,2) - m*P2(2,1) - q > 0) ~= temp_sign && ...
                (P2(3,2) - m*P2(3,1) - q > 0) ~= temp_sign

            flag = false;
            i
            222
            return;
        end
    end
end

temp = P1;
P1 = P2;
P2 = temp;

%check P2
for i=1:3
    p3 = i; %used to check "sign" 
    p1 = mod(i, 3) + 1;   % used for line calc
    p2 = mod(i+1, 3) + 1; % used for line calc
    
    if P1(p2,1) == P1(p1,1) %x2 == x1
        x_thrs = P1(p2,1);
        temp_sign = x_thrs > P1(p3,1);

        if (x_thrs > P2(1,1)) ~= temp_sign  && ...
                (x_thrs > P2(2,1)) ~= temp_sign && ...
                (x_thrs > P2(3,1)) ~= temp_sign
            flag = false;
            333
            return;
        end
    else
        m = (P1(p2,2) - P1(p1,2)) / (P1(p2,1) - P1(p1,1));
        q = P1(p1,2) - m * P1(p1,1);
        temp_sign = (P1(p3,2) - m*P1(p3,1) - q) > 0;

        if (P2(1,2) - m*P2(1,1) - q > 0) ~= temp_sign  && ...
                (P2(2,2) - m*P2(2,1) - q > 0) ~= temp_sign && ...
                (P2(3,2) - m*P2(3,1) - q > 0) ~= temp_sign

            flag = false;
            i, 444
            return;
        end
    end
end

% for both triangles
% for all 3 edges
%test with if


% *******************************************************************
end