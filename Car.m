classdef Car < handle
    properties
        m
        l
        w
        state % 0: go straight, 1: change lanes right, -1: change lanes left
        pos
        vel
        acc
        laneNum
        lanePos
        lane
    end
    
    properties(Access = private)
        maxYAccel = 9.81;
        desiredYAccel = 9.81 / 2;
        maxXAccel = 0.981;
        desiredXAccel = 0.981 / 2;
        lastLane
        targetLane
        desiredLane
        desiredSpeed
        targetSpeed
        minGap
        decideState % 0: ready for new decision, -1: busy, 1: on-ramping, 2: off-ramping
        p
        lastVelErr = 0
        velErrSum = 0
        sys
        kI
        kP
        kD
        uCap
        integralWindow
    end
    
    methods
        function obj = Car()
            
            obj.m = 10 + 10 * rand;
            obj.l = 3 + 3 * rand;
            obj.w = 2 + rand;
            
            obj.state = 0;
            obj.pos = [0, 0];
            obj.vel = [0, 0];
            obj.acc = [0, 0];
            
            obj.desiredSpeed = [0, 20 + 20 * rand];
            %             obj.desiredSpeed = [0, 30];
            obj.desiredLane = 1;
            
            obj.lanePos = 1;
            obj.decideState = 1;
            xArr = obj.pos(1) + (obj.w / 2) * [1; 1; -1; -1];
            yArr = obj.pos(2) + (obj.l / 2) * [1; -1; -1; 1];
            obj.p = patch(xArr, yArr, 'k');
            
            
            b = .1;
            
            A = [0, 1;
                0, -b / obj.m];
            
            obj.uCap = obj.m * obj.desiredYAccel;
            
            B = [0; 1 / obj.m];
            
            C = [1, 0;
                0, 1];
            
            D = [0; 0];
            
            obj.sys = ss(A, B, C, D);
            
            obj.kP = [0, 8];
            obj.kI = [0, 0.5];
            obj.kD = [0, 10];
            
            obj.integralWindow = 2.5;
            
            
        end
        
        function out = getDesiredSpeed(obj)
            out = obj.desiredSpeed;
        end
        
        function doPhysics(obj, dt)
            %             obj.acc = obj.acc - [0 0.001]; %fake drag, hard coded, woooooo
            %             obj.vel = obj.vel + dt * obj.acc;
            %             obj.pos = obj.pos + dt * obj.vel;
            
            velErr = obj.targetSpeed(2) - obj.vel(2);
            velErrDer = (velErr - obj.lastVelErr) / dt;
            if abs(velErr) < obj.integralWindow
                obj.velErrSum = obj.velErrSum + velErr;
            end
            obj.lastVelErr = velErr;
            
            
            totalError = [0; velErr];
            totalErrDer = [0; velErrDer];
            totalErrSum = [0; obj.velErrSum];
            
            % u = obj.kP * error + obj.kI * obj.errorSum + obj.kD * errorDer;
            u = obj.kP * totalError + obj.kI * totalErrSum + obj.kD * totalErrDer;
            if u > obj.uCap
                u = obj.uCap;
            end
            
            numSteps = 10;
            t = linspace(0, dt, numSteps)';
            y = lsim(obj.sys, u .* ones(numSteps, 1), t, [obj.pos(2); obj.vel(2)]);
            
            lastPos = obj.pos(2);
            lastVel = obj.vel(2);
            obj.pos(2) = y(end, 1);
            obj.vel(2) = y(end, 2);
            obj.acc(2) = (obj.vel(2) - lastVel) / dt;
%             obj.p.Vertices = obj.p.Vertices + [0, (obj.pos(2) - lastPos)];
        end
        
        function update(obj, dt)
            % this is low level work based on high-level decisions.  Keep high-level concepts in the
            % decide() method
            
            global highway
            
            switch obj.state
                case 0
                    % go straight
                    %                     obj.acc = [0, 0];
                    %                     obj.acc(2) = 0.01 * (obj.desiredSpeed(2) - obj.vel(2));
                    obj.targetSpeed = obj.desiredSpeed;
                    
                    
                case 1
                    % change lanes +
                    if abs(obj.pos(1) - obj.targetLane.x) > 0.01
                        % lane change complete, snap car to middle of lane
                        obj.pos(1) = obj.targetLane.x;
                        obj.laneNum = obj.targetLane.laneNum;
                        obj.acc(1) = 0;
                        obj.vel(1) = 0;
                        obj.state = 0;
                        
                    else
                        % keep changing lanes right
                        obj.acc(1) = 0.01 * (obj.targetLane.x - obj.pos(1)); % hard-coded values are bad
                        if obj.acc(1) > obj.maxXAccel
                            obj.acc(1) = obj.maxXAccel;
                        end
                        
                        
                        rightSpeed = highway.lanes(obj.laneNum + 1).getSpeedAtY(obj.pos(2));
                        targetSpeed = min([rightSpeed, obj.desiredSpeed(2)]);
                        
                        % accelerate to match lane speed
                        obj.acc(2) = 0.01 * (targetSpeed - obj.vel(2));
                    end
                    
                case -1
                    % change lanes -
                    if obj.pos(1) < obj.targetLane.x + 0.01
                        % lane change complete, snap car to middle of lane
                        obj.pos(1) = obj.targetLane.x;
                        obj.laneNum = obj.targetLane.laneNum;
                        obj.acc(1) = 0;
                        obj.vel(1) = 0;
                        obj.state = 0;
                    else
                        % keep changing lanes left
                        obj.acc(1) = 0.01 * (obj.targetLane.x - obj.pos(1)); % hard-coded values are bad
                        if obj.acc(1) < -1 * obj.maxXAccel
                            obj.acc(1) = -1 * obj.maxXAccel;
                        end
                        
                        leftSpeed = highway.lanes(obj.laneNum - 1).getSpeedAtY(obj.pos(2));
                        targetSpeed = min([leftSpeed, obj.desiredSpeed(2)]);
                        
                        % accelerate to match lane speed
                        obj.acc(2) = 0.01 * (targetSpeed - obj.vel(2));
                    end
            end
            obj.doPhysics(dt);
        end
        
        
        function out = getLaneSpeed(obj)
            out = obj.vel(2);
        end
        
        function out = getYPos(obj)
            out = obj.pos(2);
        end
        
        %         function out = decide(obj)
        %
        %             out = 0;
        %             global highway
        %
        %             frontSpeed = obj.lane.getFrontSpeed(obj.lanePos);
        %
        %             switch obj.decideState
        %                 case -1
        %
        %                 case 0
        %                     % only worry about speed if not doing something else
        %                     if frontSpeed > obj.desiredSpeed
        %                         obj.state = 0;
        %
        %                     else
        %                         % look at other lanes to see if they're faster
        %                         % look at left lane
        %                         if obj.laneNum > 1
        %                             leftSpeed = highway.lanes(obj.laneNum - 1).getSpeedAtY(obj.pos(2));
        %                         else
        %                             leftSpeed = -1;
        %                         end
        %
        %                         if obj.laneNum < highway.numLanes
        %                             rightSpeed = highway.lanes(obj.laneNum + 1).getSpeedAtY(obj.pos(2));
        %                         else
        %                             rightSpeed = -1;
        %                         end
        %
        %                         if leftSpeed > rightSpeed && leftSpeed > obj.vel(2)
        %                             % check to see if there's a gap that the car can fit in to the left
        %                             gap = highway.lanes(obj.laneNum - 1).getGapAtY(obj.pos(2));
        %
        %                             if (obj.l / 2) - gap(2) > obj.minGap && gap(1) - ( obj.l / 2) > obj.minGap
        %                                 % gap is big enough, initiate move over
        %                                 obj.state = -1;
        %                                 obj.decideState = -1;
        %                             else
        %                                 % check to see if we can accel/decel to get to the gap
        %                             end
        %
        %                         elseif rightSpeed > leftSpeed && rightSpeed > obj.vel(2)
        %                             % check to see if there's a gap that the car can fit in to the right
        %                             gap = highway.lanes(obj.laneNum + 1).getGapAtY(obj.pos(2));
        %
        %                             if (obj.l / 2) - gap(2) > obj.minGap && gap(1) - ( obj.l / 2) > obj.minGap
        %                                 % gap is big enough, initiate move over
        %                                 obj.state = 1;
        %                                 obj.decideState = -1;
        %                             else
        %                                 % check to see if we can accel/decel to get to the gap
        %                             end
        %
        %                         else
        %                             % track the car
        %                         end
        %                     end
        %
        %                 case 1
        %                     % on-ramping
        %                     if obj.state == 0
        %                         % do one lane change into last lane of the highway
        %                         if obj.laneNum > (highway.numLanes - 1)
        %                             obj.targetLane = highway.lanes(obj.laneNum - 1);
        %                         else
        %                             obj.decideState = 0;
        %                         end
        %                     else
        %                         obj.targetLane = highway.lanes(end - 1);
        %                     end
        %
        %                 case 2
        %                     % off-ramping
        %                     if obj.state == 0
        %                         % done lane change, initiate another lane change right, unless we're already
        %                         % at the offramp, then end offramp state
        %                         if obj.laneNum ~= obj.desiredLane
        %                             obj.targetLane = highway.lanes(obj.laneNum + 1);
        %                         else
        %                             % car has been off-ramped, kill it
        %                             obj.kill;
        %                             out = 1;
        %                         end
        %                     else
        %                         obj.targetLane = highway.lanes(obj.laneNum + 1);
        %                     end
        %
        %
        %             end
        %
        %
        %
        %         end
        
        function setIdx(obj, idx)
            obj.lanePos = idx;
        end
        
        function kill(obj)
            delete(obj.p);
        end
        
        
        
    end
end





















