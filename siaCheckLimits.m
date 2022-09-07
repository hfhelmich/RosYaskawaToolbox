function y = siaCheckLimits(q, tA, qd, tV)
%SIACHECKLIMITS validates joint angles and velocities while referencing
% the Yaskawa SIA20F. Both inputs must have same number of elements.         
% Method assumes input arrays follow standard order of joints:
% [S L E U R B T]. 
%
%   Inputs
%       q   - 7xN array of joint angles
%       tA  - tolerance for joint angles prevents Yaskawa alarm from being
%             set off during motion (0.001 - 0.1)
%       dq  - 7xN array of joint velocities
%       tV  - tolerance for joint velocities (1e-5 - 1e-4)
%       
%
%   Outputs 
%       y   - returns a 1 or 0 boolean to indicate whether numbers are 
%             within limits
%
%   Harrison Helmich, 8 Mar 22

% Updates:
%       9 Mar 22 - added tolerance for joint angles

%% Method vars
    
    y = 0;
    a = true;
    b = true;
    count = 0;
    
    % Cancel function if arrays are not same size
    if numel(q) ~= numel(qd)
        error("Arrays must be same size");
    end
    
    % TODO - user submits one or two arrays
    
    %% Joint Angles
    % fprintf('\nChecking joint angles...');
    for i = 1:length(q(1,:))
        % joint S - base
        if abs(q(1,i)) >= pi - tA            % +/-180 degrees
            fprintf('Angle \t%d violates Joint S limit\n', i);
            count = count + 1;
            a = false;
        end
        % joint L -
        if abs(q(2,i)) >= 11/18*pi - tA      % +/-110 degrees
            fprintf('Angle \t%d violates Joint L limit\n', i);
            count = count + 1;
            a = false;
        end
        % joint E - elbow
        if abs(q(3,i)) >= 17/18*pi - tA      % +/-170 degrees
            fprintf('Angle \t%d violates Joint E limit\n', i);
            count = count + 1;
            a = false;
        end
        % joint U - 
        if abs(q(4,i)) >= 13/18*pi - tA      % +/-130 degrees
            fprintf('Angle \t%d violates Joint U limit\n', i);
            count = count + 1;
            a = false;
        end
        % joint R - 
        if abs(q(5,i)) >= pi - tA      % +/-180 degrees
            fprintf('Angle \t%d violates Joint R limit\n', i);
            count = count + 1;
            a = false;
        end
        % joint B - 
        if abs(q(6,i)) >= 11/18*pi - tA      % +/-110 degrees
            fprintf('Angle \t%d violates Joint B limit\n', i);
            count = count + 1;
            a = false;
        end
        % joint T -                                                
        if abs(q(7,i)) >= pi - tA      % +/-180 degrees
            fprintf('Angle \t%d violates Joint T limit\n', i);
            count = count + 1;
            a = false;
        end
       
    end
%     % If all angles pass, print success msg
%     if a
%         fprintf('\n*** Joint angles pass. ***\n\n');
%         y = 1;
%     end
    
    %% Joint Velocities
    % fprintf('\nChecking joint velocities...');
    for i = 1:length(qd(1,:))
        % joint S - base
        if abs(qd(1,i)) >= 13/18*pi - tV    % +/-130 deg/s
            fprintf('Velocity %d\t violates Joint S limit\n', i);
            count = count + 1;
            b = false;
        end
        % joint L
        if abs(qd(2,i)) >= 13/18*pi - tV    % +/-110 deg/s
            fprintf('Velocity %d\t violates Joint L limit\n', i);
            count = count + 1;
            b = false;
        end
        % joint E - elbow
        if abs(qd(3,i)) >= 17/18*pi - tV    % +/-170 deg/s
            fprintf('Velocity %d\t violates Joint E limit\n', i);
            count = count + 1;
            b = false;
        end
        % joint U - 
        if abs(qd(4,i)) >= 13/18*pi - tV    % +/-130 deg/s
            fprintf('Velocity %d\t violates Joint U limit\n', i);
            count = count + 1;
            b = false;
        end
        % joint R - 
        if abs(qd(5,i)) >= pi - tV          % +/-180 deg/s
            fprintf('Velocity %d\t violates Joint R limit\n', i);
            count = count + 1;
            b = false;
        end
        % joint B - 
        if abs(qd(6,i)) >= 11/18*pi - tV    % +/-110 deg/s
            fprintf('Velocity %d\t violates Joint B limit\n', i);
            count = count + 1;
            b = false;
        end
        % joint T -                                               
        if abs(qd(7,i)) >= pi - tV          % +/-180 deg/s
            fprintf('Velocity %d\t violates Joint T limit\n', i);
            count = count + 1;
            b = false;
        end
       
    end
%     % If all velocities are within bounds, print success msg
%     if b
%         fprintf('\n*** Joint velocities pass. ***\n\n');
%     end
    
    % If there are violations, print number
    if count > 0
        fprintf('\n\n%d total violations\n\n',count);
    end
    
    if a && b
        y = 1;
        fprintf('\nInputs pass.\n');
    else
        y = 0;
        fprintf('\nInputs do NOT pass.\n');
    end
    
end

