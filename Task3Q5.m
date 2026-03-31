clear; clc; close all;

%% Givens

g = 9.81; % m/s^2
Ix = 5.8e-5; % kg*m^2
Iy = 7.2e-5; % kg*m^2
tauDominant = 0.5; % s
tauModal = 1.25; % s
rootScale = 5;

root1 = 1/tauDominant;
root2 = rootScale * root1;

%% LATERAL
%{
k1_lat = (root1+root2) * Ix;
k2_lat = (root1*root2) *Ix;
k3_lat = (1/tauModal)*Ix:1e-9:1e-4;

for index = 1:length(k3_lat)

    A_CL_Qlat = [           0,          g,          0;
                            0,          0,          1;
                -k3_lat(index)/Ix, -k2_lat/Ix, -k1_lat/Ix];

    poles(:,index) = eig(A_CL_Qlat);

end

figure();
title('All Roots for Varied k_3')
xlabel('Real Component')
ylabel('Imaginary Component')
hold on; grid on;
scatter(real(poles(1,:)),imag(poles(1,:)),'r')
scatter(real(poles(2,:)),imag(poles(2,:)),'b')
scatter(real(poles(3,:)),imag(poles(3,:)),'g')
legend('k_1 Poles', 'k_2 Poles', 'k_3 Poles',Location='best')

% Find indices where all poles are real AND the slowest pole is <= -0.8
is_valid = all(imag(poles) == 0, 1) & all(real(poles) <= -0.8, 1);

valid_poles = real(poles(:, is_valid)); % Only need the real parts now
valid_k3 = k3_lat(is_valid);

figure(2);
title('Only Real Roots with \tau = 1.25 s')
hold on; grid on;
xlabel('k_3 Value')
ylabel('Real Root')

if any(is_valid)
    
    figure(2);
    scatter(valid_k3,valid_poles(1,:),'r')
    scatter(valid_k3,valid_poles(2,:),'b')
    scatter(valid_k3,valid_poles(3,:),'g')
    
    fprintf('Valid k3 range: [%.4e, %.4e]\n', min(valid_k3), max(valid_k3));
else
    disp('No k3 found satisfying both real and <= -0.8 criteria. Try a different range.');
end

figure(2);
legend('k_1 Poles', 'k_2 Poles', 'k_3 Poles',Location='best')
%}


%% LONGITUDINAL

k1_long = (root1+root2) *Iy;
k2_long = (root1*root2) *Iy;
k3_long = (1/tauModal)*Iy:1e-9:1e-4;

for index = 1:length(k3_long)

    A_CL_Qlong = [           0,          g,          0;
                            0,          0,          1;
                -k3_long(index)/Iy, -k2_long/Iy, -k1_long/Iy];

    poles(:,index) = eig(A_CL_Qlong);

end

figure();
title('All Roots for Varied k_3')
xlabel('Real Component')
ylabel('Imaginary Component')
hold on; grid on;
scatter(real(poles(1,:)),imag(poles(1,:)),'r')
scatter(real(poles(2,:)),imag(poles(2,:)),'b')
scatter(real(poles(3,:)),imag(poles(3,:)),'g')
legend('k_1 Poles', 'k_2 Poles', 'k_3 Poles',Location='best')

% Find indices where all poles are real AND the slowest pole is <= -0.8
is_valid = all(imag(poles) == 0, 1) & all(real(poles) <= -0.8, 1);

valid_poles = real(poles(:, is_valid)); % Only need the real parts now
valid_k3 = k3_long(is_valid);

figure(2);
title('Only Real Roots with \tau = 1.25 s')
hold on; grid on;
xlabel('k_3 Value')
ylabel('Real Root')

if any(is_valid)
    
    figure(2);
    scatter(valid_k3,valid_poles(1,:),'r')
    scatter(valid_k3,valid_poles(2,:),'b')
    scatter(valid_k3,valid_poles(3,:),'g')
    
    fprintf('Valid k3 range: [%.4e, %.4e]\n', min(valid_k3), max(valid_k3));
else
    disp('No k3 found satisfying both real and <= -0.8 criteria. Try a different range.');
end

figure(2);
legend('k_1 Poles', 'k_2 Poles', 'k_3 Poles',Location='best')
