function resp = disable_SIA20F()

% disable robot
try
    disable_client = rossvcclient('/ysk/robot_disable_MATLAB');
    req = rosmessage(disable_client);
    call(disable_client,req,'Timeout',3);
    resp = 'passed';
catch
    disp('enable call failed')
    resp = 'failed';
end

end