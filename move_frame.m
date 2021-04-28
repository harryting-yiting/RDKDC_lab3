% move the frame by g relative to ref_frame

function move_frame(frame_name,ref_frame_name,g)
    msg = rosmessage('geometry_msgs/TransformStamped');
    msg.ChildFrameId = frame_name;
    msg.Header.FrameId = ref_frame_name;

    % geometry transformation
    q = rotm2quat(g(1:3,1:3));
    t = g(1:3,4);
    msg.Transform.Translation.X = t(1);
    msg.Transform.Translation.Y = t(2);
    msg.Transform.Translation.Z = t(3);
    msg.Transform.Rotation.W = q(1);
    msg.Transform.Rotation.X = q(2);
    msg.Transform.Rotation.Y = q(3);
    msg.Transform.Rotation.Z = q(4);
    rospublisher('matlab_frame', msg);
end