classdef Car < handle
    % THIS IS A STRIPPED-DOWN VERSION OF THE CAR MODEL, FOR VEHICLE DYNAMIC TESTING ONLY
    properties
        m
        l
        w
        state % 0: go straight, 1: change lanes right, -1: change lanes left
        curState
        acc
    end
    
    properties(Access = private)
        maxYAccel = 9.81; % will be used later for braking
        desiredYAccel = 9.81 / 4;
        maxXAccel = 0.981;
        desiredXAccel = .981;
        desiredSpeed
        p
        sys
        kI
        kP
        kD
        uCap
        lastU = [0, 0];
        deltaUCap = [3, 1];
        errSum = [0, 0; 0, 0; 0, 0; 0, 0]
        lastErr = 0
        integralWindow = [0.25; 0; 0; 2.5] % [x, x-dot, y, y-dot]
        targetState = [0; 0; 0; 0]
    end
    
    methods
        function obj = Car()
            
            % Vehicle State: [x, x-dot, y, y-dot]'
            obj.curState = [0; 0; 0; 0];
            
            obj.m = 10 + 10 * rand;
            obj.l = 3 + 3 * rand;
            obj.w = 2 + rand;
            
            obj.state = 0;
            obj.acc = [0, 0];
            
            obj.desiredSpeed = [0, 20 + 20 * rand];
            
            xArr = obj.curState(1) + (obj.w / 2) * [1; 1; -1; -1];
            yArr = obj.curState(3) + (obj.l / 2) * [1; -1; -1; 1];
            obj.p = patch(xArr, yArr, 'k');
            
            b = .1; % seems like a good value from testing, can change later
            
            % Car Physical Model
            % http://ctms.engin.umich.edu/CTMS/index.php?example=CruiseControl&section=SystemModeling
            A = [0, 1;
                0, -b / obj.m];
            
            B = [0; 1 / obj.m];
            
            C = [1, 0;
                0, 1];
            
            D = [0; 0];
            
            A = blkdiag(A, A);
            B = blkdiag(B, B);
            C = blkdiag(C, C);
            D = blkdiag(D, D);
            
            obj.sys = ss(A, B, C, D);
            
            obj.kP = [8, 0,...
                0, 8];
            obj.kI = [0, 0,...
                0, 0.5];
            obj.kD = [17, 0,...
                0, 10];
            
            obj.uCap = [obj.m * obj.desiredXAccel, obj.m * obj.desiredYAccel];
            obj.deltaUCap(1) = 0.981 * obj.m; % https://www.hindawi.com/journals/mpe/2014/478573/
            obj.deltaUCap(2) = 0.18 * 9.81 * obj.m;
            
        end
        
        function doPhysics(obj, dt)
            
            err = obj.targetState - obj.curState;
            err = [[err(1); err(2); 0; 0], [0; 0; err(3); err(4)]];
            errDer = (err - obj.lastErr) / dt;
            
            for i = 1:4
                if abs(err(i, ceil(i / 2))) < obj.integralWindow(i) % ceil(i / 2) makes it so the column sequence is 1-1-2-2;
                    obj.errSum(i, ceil(i / 2)) = obj.errSum(i, ceil(i / 2)) + err(i, ceil(i / 2));
                end
            end
            
            obj.lastErr = err;
            
            u = obj.kP * err + obj.kI * obj.errSum + obj.kD * errDer; % u will be 1x2 [ux, uy]
            
            % lazy low-pass filter u
            deltaU = u - obj.lastU;
            for i = 1:2
                if abs(deltaU(i)) > obj.deltaUCap(i)
                    u(i) = obj.lastU(i) + sign(deltaU(i)) * obj.deltaUCap(i);
                end
                if u(i) > obj.uCap(i)
                    u(i) = obj.uCap(i);
                end
            end
            
            obj.lastU = u;
            
            numSteps = 10;
            t = linspace(0, dt, numSteps)';
            y = lsim(obj.sys, u .* ones(numSteps, 1), t, obj.curState');
            
            lastYPos = obj.curState(3);
            lastXPos = obj.curState(1);
            lastXVel = obj.curState(2);
            lastYVel = obj.curState(4);
            obj.curState = y(end, :)';
            obj.acc(1) = (obj.curState(2) - lastXVel) / dt;
            obj.acc(2) = (obj.curState(4) - lastYVel) / dt;
            obj.p.Vertices = obj.p.Vertices + [obj.curState(1) - lastXPos, obj.curState(3) - lastYPos];
        end
        
        function update(obj, dt)
            
            % hard-code changes for testing
            obj.targetState(4) = obj.desiredSpeed(2);
            obj.targetState(1) = 3.7;
            
            obj.doPhysics(dt);
        end
        
        function out = getDesiredSpeed(obj)
            out = obj.desiredSpeed;
        end
        
        function out = getYVel(obj)
            out = obj.curState(4);
        end
        
        function out = getYPos(obj)
            out = obj.curState(3);
        end
        
        function out = getXVel(obj)
            out = obj.curState(2);
        end
        
        function out = getXPos(obj)
            out = obj.curState(1);
        end
        
        function out = getCurState(obj)
            out = obj.curState;
        end
        
        function out = getTargetState(obj)
            out = obj.targetState;
        end
        
        function kill(obj)
            delete(obj.p);
        end
        
    end
end





















