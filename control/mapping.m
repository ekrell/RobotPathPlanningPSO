rosshutdown();
IP_address = '192.168.150.130';
rosinit(IP_address);

handles.odomSub = rossubscriber('/odom', 'BufferSize', 25);
receive(handles.odomSub,3);
handles.laserSub = rossubscriber('/scan', 'BufferSize', 5);
receive(handles.laserSub,3);

handles.velPub = rospublisher('/mobile_base/commands/velocity');

exampleHelperTurtleBotKeyboardControl_EK(handles);

%rosshutdown();
