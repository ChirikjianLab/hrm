function collision = IsEdgeInCollision(Obj, V1, V2, dt)
collision = 0;
n_step  = floor(sqrt(sum((V1-V2).^2))*dt);

% c_sp_step = zeros(length(properties(ObjPRM.GetRobot)), n_step);
c_sp_step = zeros(3, n_step);

for i = 1:size(V1,1)
    c_sp_step(i,:) = linspace(V1(i), V2(i), n_step);
end

% Check if the rabbit hits the obstacle or arena, step by step
N = 50;
the = linspace(2*pi/(N+1),2*pi+2*pi/(N+1),N);
obs = Obj.Obs;
for j = 1:length(obs)
    pts = obs(j).GetPoints;
    obsPnt(j).Vertices = pts';
end

for i = 1:n_step
    % discretize robot and environment
    % robot
    x   = Obj.Robot.ra*Obj.Robot.sc_eps(the, Obj.Robot.eps, 'cos');
    y   = Obj.Robot.rb*Obj.Robot.sc_eps(the, Obj.Robot.eps, 'sin');
    pts = rot2(c_sp_step(3,i))*[x; y] +...
        repmat([c_sp_step(1,i); c_sp_step(2,i)], 1, N);
    robotPnt.Vertices = pts';
    plot(pts(1,:), pts(2,:),'b');
    
    % obstacle
    for j = 1:length(obs)
        collision = GJK2D(robotPnt',obsPnt(j)',20);
        if collision
%             disp(['Collision with obstacle # ', num2str(j)])
            return;
        end
    end
end