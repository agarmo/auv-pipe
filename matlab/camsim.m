%% Info
%Camera simulator for the AUV pipeline following fall project. need fixed
%time interval, because of the trajectory of the pipeline.
%
%Input: Absolute position from the AUV.
%Output: 3 points with equal spacing placed on top of the pipline.
%        dim specifies if there is a 2D point or 3D point
%
%Parameters: 
%   Global:      The pipeline waypoints or trajectory.
%                camera field of view, dependent on z.
%                
%   Local:       
%
%
%Method:
%     The waypoint are interpolated using some algorithm to get smooth
%     trajectory. Check to see if camera view sees a part of the pipeline. If it does
%     the 3 equal spaced points positioned on top of the pipeline.
%     
%     
%

%% program



function [P1, P2, P3, dim] = camsim(pos, t, dimention)

global pipeline %pipeline is an array containing the trajectory of the pipeline in 3D
global fov % Camera field of view. Array containing 2 parameters, x, y.
global focus % the focus distance of the camera.

if isempty(pipeline)
    error('The pipeline trajectory is empty');
end
if isempty(fov)
    error('Need to specify the field of view');
end
if isempty(focus)
    error('Need to specify the focus distance of the camera');
end

%calculate the field of view for the given height, need altitude over sea
%bottom.
depth = pipeline(t, 3)- pos(3);

realfov = [fov(1)*focus/depth;
           fov(2)*focus/depth];


           

           
           
% Figure out if the some of the pipeline is inside the current field of
% view.




%print the 3 coordinates of the pipeline out.




end
