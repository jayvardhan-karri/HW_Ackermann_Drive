function publish_path_ros2(pub, ref, step)
%PUBLISH_PATH_ROS2  Send the reference path as nav_msgs/Path (frame 'odom').
%   publish_path_ros2(pub, ref)        pub = ros2publisher(node, '/reference_path', 'nav_msgs/Path')
%   publish_path_ros2(pub, ref, step)  every `step`-th waypoint (default 2 = every 2 cm)
%   Lets the Python live_plotter (and RViz) show the path MATLAB is tracking.
%   NOTE: the Python controllers also listen to /reference_path - do not run
%   them at the same time as a MATLAB controller.
if nargin < 3, step = 2; end
idx = 1:step:ref.n;
msg = ros2message(pub);
msg.header.frame_id = 'odom';
ps = ros2message('geometry_msgs/PoseStamped');
ps.header.frame_id = 'odom';
poses = repmat(ps, numel(idx), 1);
for k = 1:numel(idx)
    i = idx(k);
    poses(k).pose.position.x = ref.x(i);
    poses(k).pose.position.y = ref.y(i);
    poses(k).pose.orientation.z = sin(ref.yaw(i) / 2);
    poses(k).pose.orientation.w = cos(ref.yaw(i) / 2);
end
msg.poses = poses;
send(pub, msg);
end
