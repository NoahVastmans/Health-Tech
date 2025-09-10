

acc_data = readtable("Data 0904.xlsx","Sheet","3 throws Acce take2");
gyro_data = readtable("Data 0904.xlsx","Sheet","3 throws Gyr take2");

time_acc = acc_data{:,3};
acc_x = acc_data{:,4} / 1000;  %mg -> g
acc_y = acc_data{:,5} / 1000;
acc_z = acc_data{:,6} / 1000;

time_gyro = gyro_data{:,3}; %contains NaN
time_gyro = fillmissing(time_gyro, 'linear'); % linear interpolation
gyro_x = gyro_data{:,4}; % dps
gyro_y = gyro_data{:,5};
gyro_z = gyro_data{:,6};

dt = mean(diff(time_gyro));

acc_roll = atan2(interp1(time_acc, acc_y, time_gyro), ...
                 sqrt(interp1(time_acc, acc_x, time_gyro).^2 + interp1(time_acc, acc_z, time_gyro).^2)) * 180/pi;

acc_pitch = atan2(-interp1(time_acc, acc_x, time_gyro), ...
                  sqrt(interp1(time_acc, acc_y, time_gyro).^2 + interp1(time_acc, acc_z, time_gyro).^2)) * 180/pi;


dt = mean(diff(time_gyro));
disp(['dt = ', num2str(dt), ' s']);

figure;
plot(time_gyro, gyro_x, 'r'); hold on;
plot(time_gyro, gyro_y, 'g');
plot(time_gyro, gyro_z, 'b');
xlabel('Time (s)'); ylabel('Gyro (°/s)');
title('Gyroscope raw signals'); legend('X','Y','Z'); grid on;

gyro_roll  = cumsum(gyro_x * dt); 
gyro_pitch = cumsum(gyro_y * dt);

% === Fusion ===
alpha = 0.8;
com_roll  = zeros(size(time_gyro));
com_pitch = zeros(size(time_gyro));

for k = 2:length(time_gyro)
    gyro_pred_roll  = com_roll(k-1)  + gyro_x(k) * dt;
    gyro_pred_pitch = com_pitch(k-1) + gyro_y(k) * dt;

    com_roll(k)  = alpha * gyro_pred_roll  + (1-alpha) * acc_roll(k);
    com_pitch(k) = alpha * gyro_pred_pitch + (1-alpha) * acc_pitch(k);
end


rot_speed = sqrt(gyro_x.^2 + gyro_y.^2 + gyro_z.^2); % dps
rot_speed_rpm = rot_speed * 60 / 360; % conversion en tours/min


% === Figure 1 : Angles ===
figure;
subplot(3,1,1);
plot(time_gyro, acc_roll, 'g', 'LineWidth',1.2); hold on;
plot(time_gyro, gyro_roll, 'r', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Angle (°)');
title('Roll (acc vs gyro)');
legend('Acc','Gyro'); grid on;

subplot(3,1,2);
plot(time_gyro, acc_pitch, 'g', 'LineWidth',1.2); hold on;
plot(time_gyro, gyro_pitch, 'r', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Angle (°)');
title('Pitch (acc vs gyro)');
legend('Acc','Gyro'); grid on;

subplot(3,1,3);
plot(time_gyro, com_roll, 'b', 'LineWidth',1.5); hold on;
plot(time_gyro, com_pitch, 'm', 'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Angle (°)');
title(['Fusion (complementary filter, \alpha = ' num2str(alpha) ')']);
legend('Roll fusion','Pitch fusion'); grid on;

% === Figure 2 : Rotational speed ===
figure;
plot(time_gyro, rot_speed_rpm, 'm', 'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Rotational speed (tr/min)');
title('Rotational speed of the sensor');
grid on;
