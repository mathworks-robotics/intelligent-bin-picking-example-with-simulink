
% Commands for MEXing exampleHelperCHOMPMotionPlanner function
bl = coder.typeof(1);
bw = coder.typeof(1);
bh = coder.typeof(1);
bcp = coder.typeof([1, 1,2]);
br = coder.typeof(1);
no = coder.typeof(1);
tf = coder.typeof(uint8(1));
cnfgs = coder.typeof(1, [3,6], [1 0]);
obs = coder.typeof(1, [20, 4], [1, 0]);
objYaw = coder.typeof(1);
codegen exampleHelperCHOMPMotionPlanner.m -args {tf, bl, bw, bh, bcp, br, no, obs, cnfgs, objYaw}
%#codegen