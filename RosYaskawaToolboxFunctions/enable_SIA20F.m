function resp = enable_SIA20F()
%   ENABLE_SIA20F calls the "rosservice call enable_robot" function, which
%   allows the Yaskawa to begin accepting commands remotely
%
%   L. DeVries, 1Mar20, USNA

try
    %enable_client1 = rossvcclient('/ysk/robot_enable');
    enable_client2 = rossvcclient('/ysk/robot_enable_MATLAB');
    %req1 = rosmessage(enable_client1);
    req2 = rosmessage(enable_client2);
    %call(enable_client1,req1,'Timeout',3);
    call(enable_client2,req2,'Timeout',3);
    
    resp = 'passed'
catch
    disp('enable call failed')
    resp = 'failed';
end

end