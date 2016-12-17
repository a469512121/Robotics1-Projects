function [q1,q2,q3] = inverseK(P0T,P,H)
    d=norm(P0T-P(:,1));
    p=P(:,4);
    Q=P(:,3);
    k=H(:,3);
    q3=subproblem3(k,-p,Q,d);
    for i=1:length(q3)
        if norm(q3(i)) > pi/2
            q3(i) = [];
        end
    end

    k1=H(:,1);
    k2=H(:,2);
    Q=P0T-P(:,1);
    %Find q1 and q2 using q3
    for i=1:length(q3)
        p=P(:,3)+rot(H(:,3),q3(i))*P(:,4);
        [theta1,theta2]=subproblem2(k1,k2,p,Q);
        q1(:,i)=theta1;
        q2(:,i)=theta2;
    end

    for i=1:length(q2)
        try
            if norm(q2(i)) > pi/2
                q2(i) = [];
                q1(i) = [];
            end
        catch
        end
    end

    if length(q2) > 1
        for i=1:length(q2)
            try
                if norm(q2(i)) <= norm(q2(i+1))
                    q2(i+1) = [];
                    q1(i+1) = [];
                else
                    q2(i) = [];
                    q1(i) = [];
                end
            catch
            end
        end
    end

    q1 = 2*q1/pi*300 + 500;
    q2 = 2*q2/pi*300 + 500;
    q3 = 2*q3/pi*300 + 500;