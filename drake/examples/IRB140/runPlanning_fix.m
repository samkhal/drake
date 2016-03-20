function xtraj = runPlanning_fix(q0, pos_final, options)
% default starting pose
if nargin<1
  q0 = [0,0,0,0,0,0]';
end
if (nargin<2)
  % Somewhere out in front as an example
  pos_final = [0.5, 0.0, 0.5]';
end
if (nargin<3)
  options = [];
end

if ~isfield(options,'visualize')
  options.visualize = true;
end
if ~isfield(options,'base_offset')
  options.base_offset = [0.0, 0, 0.0]';
end
if ~isfield(options,'base_rpy')
  options.base_rpy = [0, 0.0, 0]';
end

r=RigidBodyManipulator();
r = addRobotFromURDF(r,'urdf/irb_140.urdf',options.base_offset,options.base_rpy);
if (options.visualize)
  v=r.constructVisualizer();
end

gripper_idx = findLinkId(r,'link_6');
gripper_pt = [-0.04,0,0.1]';

grasp_orient = [0.5019 -0.6372 0.1823 0.5556]';

T = 1;
N = 2;
Allcons = cell(0,1);

tol = 0.1;
gripper_cons = WorldPositionConstraint(r, gripper_idx,gripper_pt,pos_final-tol,pos_final+tol,[T,T]);
Allcons{end+1} = gripper_cons;

tol = 1;
r_gripper_cons_orient = WorldQuatConstraint(r,gripper_idx,grasp_orient,tol,[0.1*T,T]);
Allcons{end+1} = r_gripper_cons_orient;

% compute seeds
q_start_nom = q0;


ikproblem = InverseKinematics(r,q_start_nom,...
  Allcons{:});
[q_end_nom,F,info,infeasible_cnstr_ik] = ikproblem.solve(q_start_nom);
qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));

t_vec = linspace(0,T,N);

% do IK

ikproblem = InverseKinematicsTrajectory(r,...
  t_vec,qtraj_guess,false,q_start_nom,...
  Allcons{:});
[xtraj,F,info,infeasible_cnstr_ik] = ikproblem.solve(qtraj_guess);

q_end = xtraj.eval(xtraj.tspan(end));
% do visualize

if options.visualize && info <= 10
  v.playback(xtraj,struct('slider',true));
end


if info > 10
  error('IK fail snopt_info: %d\n', snopt_info);
end
end

