%*********************************************************************
% Generate CSACS Geometry
%*********************************************************************
theta = linspace(0,2*pi);

% Platform:
Face(1).bcf = [ zeros(1,100) - 0.025; 
                9*cos(theta);
                9*sin(theta)];
Face(2).bcf = [ zeros(1,100) + 0.025; 
                9*cos(theta);
                9*sin(theta)];
            
for n = 1:numel(Face)
    Face(n).bcf = R2(pi/2)*Face(n).bcf;
    hold on;
end

% Reaction Wheels:
Face(3).bcf = [ 2.25*ones(1,100); 
                cos(theta);
                sin(theta)-0.25];
Face(7).bcf = [ 1.5*ones(1,100); 
                cos(theta);
                sin(theta)-0.25];
for n = 1:length(theta)
    Face(3).bcf(:,n)  = R2(45*pi/180)*Face(3).bcf(:,n);
    Face(4).bcf(:,n)  = R3(pi/2)*Face(3).bcf(:,n);
    Face(5).bcf(:,n)  = R3(pi)*Face(3).bcf(:,n);
    Face(6).bcf(:,n)  = R3(3*pi/2)*Face(3).bcf(:,n);
    Face(7).bcf(:,n)  = R2(45*pi/180)*Face(7).bcf(:,n);
    Face(8).bcf(:,n)  = R3(pi/2)*Face(7).bcf(:,n);
    Face(9).bcf(:,n)  = R3(pi)*Face(7).bcf(:,n);
    Face(10).bcf(:,n) = R3(3*pi/2)*Face(7).bcf(:,n);
end

for f = 1:length(theta)-1
    Wheel(1).Face(f).bcf = [2.25 2.25 1.5 1.5 2.25;
                            cos([theta(f) theta(f+1) theta(f+1) theta(f) theta(f)]);
                            sin([theta(f) theta(f+1) theta(f+1) theta(f) theta(f)])-0.25];
    for v=1:5
        Wheel(1).Face(f).bcf(:,v) = R2(45*pi/180)*Wheel(1).Face(f).bcf(:,v);
        Wheel(2).Face(f).bcf(:,v) = R3(pi/2)*Wheel(1).Face(f).bcf(:,v);
        Wheel(3).Face(f).bcf(:,v) = R3(pi)*Wheel(1).Face(f).bcf(:,v);
        Wheel(4).Face(f).bcf(:,v) = R3(3*pi/2)*Wheel(1).Face(f).bcf(:,v);
    end
end

wfaces = length(Wheel(1).Face);