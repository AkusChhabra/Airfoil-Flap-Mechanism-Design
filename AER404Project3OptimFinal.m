close all; clc; clear; 

% ––––––––––––––––––––––––
% | 1 Angular Velocities |
% ––––––––––––––––––––––––

% Link Lengths: r1 > r3 > r4 > r2
    r1 = 1.270; % {INPUT} <<
    r2 = 0.762; % {INPUT} <<
    r3 = 0.889; % {INPUT} <<
    r4 = 1.016; % {INPUT} <<
    F1 = 1350; % {INPUT} <<
    
    w2 = 2; % Enter Input angular velocity (rad/s) {INPUT} <<
    
    % Theta 2 Input Angle
    
    theta2 = 315; % {INPUT} <<   

% theta3 computation

    a3 = 2 * r2 * r3 * cosd(theta2) - (2 * r1 * r3);
    b3 = 2 * r2 * r3 * sind(theta2);
    d3 = r4^2 - (r1^2 + r2^2 + r3^2) + (2 * r1 * r2 * cosd(theta2));
    alpha3 = atand(b3/a3);
    
    R3 = sqrt(a3^2 + b3^2);
    theta3 = acosd(d3/R3) + alpha3;

% theta4 computation

    a4 = -2 * r2 * r4 * cosd(theta2) - (2 * r1 * r4);
    b4 = -2 * r2 * r4 * sind(theta2);
    d4 = r3^2 - (r1^2 + r2^2 + r4^2) + 2 * r1 * r2 * cosd(theta2);
    alpha4 = atand(b4/a4);
    
    R4 = sqrt(a4^2 + b4^2);
    theta4 = acosd(d4/R4) + alpha4;

% Substitute to find angular velocity of links 3 and 4

    [w3, w4] = angularVelocity(w2, theta2, theta3, theta4, r2, r3, r4) % Outputs angular velocity of links 3 and 4

% ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
% | 2 Mechanical Advantage and Resultant Force Calculation |
% ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

% Assumption: Friction is negligible and constant input angular velocity.
% Force acts at the end of r2 and r4.

    gamma = gammaCalc(theta4, theta3);
    
    v = theta2 - theta3;
    rin = r2; % Input force vector distance from fixed link
    rout = r4; % Output force vector distance from fixed link

    % MA = (rin*r4*sind(gamma))/(rout*r2*sind(v)) % Outputs Mechanical Advantage
    MA = mechAdvantage(w2, w4, rin, rout) % Outputs Mechanical Advantage
    
    F2 = forceCalc(F1,MA)
    
% ––––––––––––––––––––––––––––––––––––––––––
% | 3 Pitching Moments about Quarter Chord |
% ––––––––––––––––––––––––––––––––––––––––––

    for i = 1:11
        AOA(i,1) = i-1;
    end
    
    sizeAOA = size(AOA,1);
    CMc = sizeAOA; % Size of quarter chord moment coefficient moment data for Wings + Flaps
    CMcFlap = sizeAOA; % Size of quarter chord moment coefficient moment data for Wings + Flaps
    
    CMle = [-1.215;-1.25;-1.285;-1.31;-1.335;-1.36;-1.375;-1.395;-1.405;-1.41;-1.39]; % Drag coefficient data for Wing + Flaps
    CMleFlap = [-0.685;-0.69;-0.695;-0.695;-0.69;-0.685;-0.68;-0.675;-0.66;-0.655;-0.635]; % Drag coefficient data for Flaps
    CLArray = [2.4;2.53;2.65;2.75;2.85;3;3.15;3.2;3.25;3.38;3.4]; % Lift coefficient data for Wing + Flaps
    CLArrayFlap = [0.75;0.745;0.74;0.73;0.72;0.71;0.7;0.69;0.67;0.65;0.61]; % Lift coefficient data for Flaps

        for i = 1:sizeAOA
            CMc(i) = pitchingMoment(CLArray(i),CMle(i));
            CMcFlap(i) = pitchingMoment(CLArrayFlap(i),CMleFlap(i));
        end

            figure;
            plot(AOA, CMc);
            xlabel('Angle of Attack (˚)');
            ylabel('Pitching Moment Coefficients about c/4');
            title('AoA vs. Cm,c/4 Ratio');

            hold on;
            plot(AOA, CMcFlap);
            hold off;
            legend('Wing + Flaps', 'Flaps');

% –––––––––––––––––
% | 4 Stall Speed |
% –––––––––––––––––

% Assumption: Steady level flight & lift just equal to weight of aircraft
% Data taken from EngineeringToolBox: https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html

    FL = [0;1000;2000;3000;4000;5000;6000;7000;8000;9000;10000;15000;20000;25000;30000;40000;50000;60000;70000;80000]; % Height above sea level [m]
    rhoInf = [1.225;1.112;1.007;0.9093;0.8194;0.7364;0.6601;0.5900;0.5258;0.4671;0.4135;0.1948;0.08891;0.04008;0.01841;0.003996;0.001027;0.0003097;0.00008283;0.00001846]; % Density of air [kg/m^3]
    gConst = [9.807;9.804;9.801;9.797;9.794;9.791;9.788;9.785;9.782;9.779;9.776;9.761;9.745;9.730;9.715;9.684;9.654;9.624;9.594;9.564]; % Graviation Constant above sea level [m]
    
    sizeRho = size(rhoInf,1);
    Vstall = sizeRho;
    VstallFlap = sizeRho;

    m = 1600; % Weight of Aircraft [N] {INPUT} <<
    b = 1371.6/1000; % Span of airfoil [m]
    c = 2971.8/1000; % Chord of airfoil [m]
    S = b*c; % Area of airfoil [m^2]

    for j = 1:3
            if (j == 1) % Stall Speed at 0% Flaps
                FlapP = 0;
                CLmax = 2.4; % Maximum lift coeffient for flaps + wing
                CLmaxFlap = 0.75; % Maximum lift coeffient for flaps 
              
            elseif (j == 2)  % Stall Speed at 50% Flaps
                FlapP = 50;
                CLmax = 3; % Maximum lift coeffient for flaps + wing
                CLmaxFlap = 0.71; % Maximum lift coeffient for flaps 
              
            elseif (j == 3) % Stall Speed at 100% Flaps
                FlapP = 100;
                CLmax = 3.4; % Maximum lift coeffient for flaps + wing
                CLmaxFlap = 0.61; % Maximum lift coeffient for flaps 
            end
    
        for i = 1:sizeRho
            W = m*gConst(i);
            Vstall(i) = stallSpeed(W, rhoInf(i), S, CLmax);
            VstallFlap(i) = stallSpeed(W, rhoInf(i), S, CLmaxFlap);
        end
        
        % Plot data for both configurations
        
        figure;
        plot(FL, Vstall);
        xlabel('Height above Sea Level (m)');
        ylabel('Stall Speed (m/s)');
        title("Height above Sea Level vs. Stall Speed for " + FlapP + " % Flaps");
        
        hold on;
        plot(FL, VstallFlap);
        legend('Wing + Flaps', 'Flaps');
        hold off;
    end

% ––––––––––––––––––––––––––
% | 5 Lift and Drag Values |
% ––––––––––––––––––––––––––

% Altitude vs Drag and Altitude vs Lift Graphs
    
    Dforce = sizeRho;
    DforceFlap = sizeRho;
    Lforce = sizeRho;
    LforceFlap = sizeRho;

    Vc = 205; % Cruising Speed [m/s] {INPUT} <<
    t = 537.18/1000; % Maximum thickness of airfoil [m]
    A = t*b; % Front surface area [m^2]

        for j = 1:3
            if (j == 1) % Drag Calaculation 0% Flaps
                FlapP = 0;
                Cd = 0.24; % Drag Coefficient for Wing + Flaps
                Cdf = 0.06; % Drag Coefficient for Flaps
                Cl = 2.4; % Lift Coefficient for Wing + Flaps
                Clf = 0.75; % Lift Coefficient for Flaps
              
            elseif (j == 2)  % Drag Calaculation 50% Flaps
                FlapP = 50;
                Cd = 0.29; % Drag Coefficient for Wing + Flaps
                Cdf = 0.077; % Drag Coefficient for Flaps
                Cl = 3; % Lift Coefficient for Wing + Flaps
                Clf = 0.71; % Lift Coefficient for Flaps
                
            elseif (j == 3) % Drag Calaculation 100% Flaps
                FlapP = 100;
                Cd = 0.317; % Drag Coefficient for Wing + Flaps
                Cdf = 0.135; % Drag Coefficient for Flaps
                Cl = 3.4; % Lift Coefficient for Wing + Flaps
                Clf = 0.61; % Lift Coefficient for Flaps
            end
    
        % Plot data for both configurations
            
            for i = 1:sizeRho
                Dforce(i) = DragCalc(Cd, Vc, A, rhoInf(i));
                DforceFlap(i) = DragCalc(Cdf, Vc, A, rhoInf(i));
                Lforce(i) = LiftCalc(Cl, Vc, S, rhoInf(i));
                LforceFlap(i) = LiftCalc(Clf, Vc, S, rhoInf(i));
            end
            
            % Plot Height vs Drag Force
            
            figure;
            plot(FL, Dforce);
            xlabel('Height above Sea Level (m)');
            ylabel('Drag Force (N)');
            title("Height above Sea Level vs. Drag Force for " + FlapP + " % Flaps");
            
            hold on;
            plot(FL, DforceFlap);
            legend('Wing + Flaps', 'Flaps');
            hold off;
            
            % Plot Height vs Lift Force
            
            figure;
            plot(FL, Lforce);
            xlabel('Height above Sea Level (m)');
            ylabel('Lift Force (N)');
            title("Height above Sea Level vs. Lift Force for " + FlapP + " % Flaps");
            
            hold on;
            plot(FL, LforceFlap);
            legend('Wing + Flaps', 'Flaps');
            hold off;
        end

% ––––––––––––––––––––––––––––––––––––––––––––––––––
% | 6 Speed versus Lift and Drag Force Calculation |
% ––––––––––––––––––––––––––––––––––––––––––––––––––

    rhoConst = 1.225; % Assume constant density [kg/m^3]
    
    for i = 1:200
        V(i,1) = i;
    end

        for j = 1:3
            if (j == 1) % Drag Calaculation 0% Flaps
                FlapP = 0;
                Cd = 0.24; % Drag Coefficient for Wing + Flaps
                Cdf = 0.06; % Drag Coefficient for Flaps
                Cl = 2.4; % Lift Coefficient for Wing + Flaps
                Clf = 0.75; % Lift Coefficient for Flaps
              
            elseif (j == 2)  % Drag Calaculation 50% Flaps
                FlapP = 50;
                Cd = 0.29; % Drag Coefficient for Wing + Flaps
                Cdf = 0.077; % Drag Coefficient for Flaps
                Cl = 3; % Lift Coefficient for Wing + Flaps
                Clf = 0.71; % Lift Coefficient for Flaps
                
            elseif (j == 3) % Drag Calaculation 100% Flaps
                FlapP = 100;
                Cd = 0.317; % Drag Coefficient for Wing + Flaps
                Cdf = 0.135; % Drag Coefficient for Flaps
                Cl = 3.4; % Lift Coefficient for Wing + Flaps
                Clf = 0.61; % Lift Coefficient for Flaps
            end
    
        for i = 1:size(V,1)
            Dforce(i) = DragCalc(Cd, V(i), A, rhoConst);
            DforceFlap(i) = DragCalc(Cdf, V(i), A, rhoConst);
            Lforce(i) = LiftCalc(Cl, V(i), S, rhoConst);
            LforceFlap(i) = LiftCalc(Clf, V(i), S, rhoConst);
        end
        
        % Plot data for both configurations
        
            % Plot Speed vs Drag Force
            
            figure;
            plot(V, Dforce);
            xlabel('Speed (m/s)');
            ylabel('Drag Force (N)');
            title("Speed vs. Drag Force for " + FlapP + " % Flaps");
            
            hold on;
            plot(V, DforceFlap);
            legend('Wing + Flaps', 'Flaps');
            hold off;
            
            % Plot Speed vs Lift Force
            
            figure;
            plot(V, Lforce);
            xlabel('Speed (m/s)');
            ylabel('Lift Force (N)');
            title("Speed vs. Lift Force for " + FlapP + " % Flaps");
            
            hold on;
            plot(V, LforceFlap);
            legend('Wing + Flaps', 'Flaps');
            hold off;
        end
        
% –––––––––––––––––––––––––
% | 7 AOA versus L/D Plot |
% –––––––––––––––––––––––––        
        
    % Angle of Attack versus Lift-Drag Coefficient Ratio Graph
    % Create Angle of Attack data
    
    LDArray = sizeAOA;
    LDArrayFlap = sizeAOA;

    % Data inputted manually from project AOA vs Cd and AOA vs Cl graphs.
    % No data file available.

    CDArray = [0.06;0.062;0.067;0.07;0.073;0.077;0.08;0.09;0.099;0.115;0.135]; % Drag coefficient data for Wing + Flaps
    CDArrayFlap = [0.24;0.25;0.26;0.27;0.28;0.29;0.3;0.305;0.315;0.32;0.317]; % Drag coefficient data for Flaps

        for i = 1:sizeAOA
            LDArray(i) = LDCalc(CLArray(i), CDArray(i));
            LDArrayFlap(i) = LDCalc(CLArrayFlap(i), CDArrayFlap(i));
        end

            figure;
            plot(AOA, LDArray);
            xlabel('Angle of Attack (˚)');
            ylabel('Cl/Cd');
            title('AoA vs. Cl/Cd Ratio');

            hold on;
            plot(AOA, LDArrayFlap);
            hold off;
            legend('Wing + Flaps', 'Flaps');

% ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
% –––––––––––––––––––––            Functions             –––––––––––––––––––––––––––––
% ––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

% Angular Velocity Function
function [w3, w4] = angularVelocity(w2, theta2, theta3, theta4, r2, r3, r4)

    gamma = gammaCalc(theta4, theta3);
    
    w3 = w2 * r2 * sind(gamma)/(r3 * sind(theta3 - theta4));
    w4 = w2 * r2 * sind(theta2 - theta3)/(r4 * sind(gamma));

end

% Gamma Angle Function
function gamma = gammaCalc(theta4, theta3)
        if (abs(theta4 - theta3) <= 90)
            gamma = abs(theta4 - theta3);
        elseif (theta4 - theta3 > 90)
            gamma = 180 - abs(theta4 - theta3);
        end
end

% Mechanical Advantage Function
function [MA] = mechAdvantage(w2, w4, rin, rout)
    MA = (w2/w4) * (rin/rout);
end

% Resultant Force Function
function [F2] = forceCalc(F1,MA)
    F2 = F1 * MA;
end

% Stall Speed Function
function [Vstall] = stallSpeed(W, rhoInf, S, CLmax)
    Vstall = sqrt( (2 * W) / (rhoInf * S * CLmax) );
end

% Drag function
function [Dforce] = DragCalc(Cd, Vc, A, rhoInf)
    Dforce = .5 * Cd * rhoInf * Vc^2 * A;
end

% Lift function
function [Lforce] = LiftCalc(Cl, Vc, S, rhoInf)
    Lforce = .5 * Cl * rhoInf * Vc^2 * S;
end

% L/D function
function [LD] = LDCalc(CL,CD)
    LD = CL/CD;
end

% Pitching Moment Coefficient about Quart Chord Function
function [Cmc] = pitchingMoment(Cl, Cmle)
    Cmc = Cmle + Cl/4;
end