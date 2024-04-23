%   Copyright 2023 The MathWorks, Inc.

prompt = {'Enter IP address of Robot:'};
dlgtitle = 'Robot IP Address';
dims = [1 50];
if exist('robotAddress', 'var')
    definput = {robotAddress};
else
    definput = {'192.168.1.10'};
end
answer = inputdlg(prompt,dlgtitle,dims,definput);
if ~isempty(answer)
    robotAddress = answer{1};
else
    robotAddress = definput{1};
end

set_param([gcs, '/robotAddress Display'], ...
    'MaskDisplay', 'color(''blue''); try ipad = evalin(''base'', ''robotAddress''); catch ipad = ''192.162.1.10''; end; disp(ipad)');