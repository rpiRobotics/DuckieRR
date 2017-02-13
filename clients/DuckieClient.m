% Duckiebot Matlab Client Example

duckie = RobotRaconteur.ConnectService('rr+tcp://duckiebot1.local:1234/?service=Drive')
	
disp('Set the gain and trim values (determined experimentally so that duckie drives straight)')
duckie.gain = 5.0;
duckie.trim = 0.001;
duckie
pause(2)

disp('Reset the parameters to their defaults')
duckie.resetParams();
duckie
pause(2)

disp('Drive forward with a car command (vel, omega)');
duckie.carCmd(1.0,0.0);
pause(2)
disp('Send another command to stop')
duckie.carCmd(0.0,0.0);

disp('Control the wheel velocities directly (vL, vR)');
duckie.wheelCmd(1.0,-1.0);
pause(2)
disp('Toggle the EStop');
duckie.toggleEStop(); % stop with the eStop
disp('Duckie should have stopped');

disp( 'Try to move again...');
duckie.wheelCmd(-1.0,1.0);
pause(2);
duckie.wheelCmd(0,0);
disp('Nothing happened because the eStop was toggled')

disp('Release the eStop')
duckie.toggleEStop();
disp('Try to move again...');
duckie.wheelCmd(-1.0,1.0);
pause(2);
duckie.wheelCmd(0,0);

%%
cam = RobotRaconteur.ConnectService('rr+tcp://duckiebot1.local:1235/?service=Camera')

% set the format
cam.changeFormat('rgb');

% capture 1 image
frame = DuckieImageToImage(cam.captureImage());
figure(1)
image(frame)
pause(1)

% capture 20 images.
for i = 1:5
   tic
   frame = DuckieImageToImage(cam.captureImage());
   figure(2)
   image(frame)
   toc
   pause(0.03)
end

% set up the stream from the pipe
cam.toggleFramerate(); % set framerate to 15fps 

s = cam.ImageStream.Connect(-1); % connect to the pipe
try 
    cam.startCapturing(); % start the stream
catch % ignore any errors
end

figure(3)
for i = 1:100 % loop through and show 20 secs of the feed
    % if there is a packet available, recieve and show
    while(s.Available > 0)
        im=DuckieImageToImage(s.ReceivePacket());
        clf
        image(im)
    end
    pause(.2)
end

s.Close(); % close the pipe
cam.stopCapturing(); % stop capturing


