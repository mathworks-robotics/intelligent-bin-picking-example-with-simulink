function compilerValidation
%This function is for internal use only. It may be removed in the future.
%
% This function builds MEX files for input c source file.

%   Copyright 2024-2025 The MathWorks, Inc.

% the supported compiler list
    archKeys = {'win64', 'glnxa64', 'maci64', 'maca64'};

    supportedCompilerNameList = containers.Map(archKeys,{'Microsoft Visual C','g++','Xcode Clang++','Xcode Clang++'});
    supportedCompilerVersionList = containers.Map(archKeys,{'14.0','6.0.0','9.0.0','9.0.0'});

    compilerList = {supportedCompilerNameList(computer('arch'))};
    versionList = {supportedCompilerVersionList(computer('arch'))};

    %% Check Installed Compiler
    % Retrieves installed MEX configurations
    installedCompiler = mex.getCompilerConfigurations('C','Installed');

    % check installed compiler supported or not
    validInstalledCompiler = 0;
    validInstalledCompilerName = {};
    validInstalledCompilerVersion = {};
    vIdx = 1;
    for cidx = 1 : length(installedCompiler)
        for idx = 1 : length(compilerList)
            if ( ~isempty(strfind(installedCompiler(cidx).Name,  compilerList(idx))) && ...
                 (str2double(erase(installedCompiler(cidx).Version,'.0')) > ...
                  str2double(erase(versionList(idx),'.0'))) )

                validInstalledCompilerName{vIdx} = installedCompiler(cidx).Name; %#ok<AGROW>
                validInstalledCompilerVersion{vIdx} = installedCompiler(cidx).Version; %#ok<AGROW>
                vIdx = vIdx + 1;
                validInstalledCompiler = 1;
            end
        end
    end

    % create error message string of supported compiler names
    CompilerListString = '';
    if(~validInstalledCompiler)
        for idx = 1 : length(compilerList)
            CompilerListString = [CompilerListString,compilerList{idx},': version > ',versionList{idx},' ']; %#ok<AGROW>
            if ( idx < length(compilerList))
                CompilerListString = [CompilerListString,' OR ']; %#ok<AGROW>
            end
        end
    end

    % error-out if installed compiler is not supported
    if(~validInstalledCompiler)
        error(message('robotics:robotgazebo:gazebogenmsg:InvalidInstalledCompiler',CompilerListString));
    end

    %% Check Selected Compiler
    % retrieves selected and installed MEX configurations
    selectedCompiler = mex.getCompilerConfigurations('C','Selected');
    compilerNameString = [selectedCompiler.Name,' ',num2str(selectedCompiler.Version)];
    disp(message('robotics:robotgazebo:gazebogenmsg:CompilerMessage',compilerNameString).getString);

    % check selected compiler supported or not
    validSelectedCompiler = 0;
    for idx = 1 : length(compilerList)
        if ( ~isempty(strfind(selectedCompiler.Name,  compilerList(idx))) &&  ...
             (str2double(erase(selectedCompiler.Version,'.0')) > str2double(erase(versionList(idx),'.0'))) )
            validSelectedCompiler = 1;
        end
    end

    % create error message string of supported compiler names
    CompilerListString = '';
    if(~validSelectedCompiler)
        for idx = 1 : length(compilerList)
            CompilerListString = [CompilerListString,compilerList{idx},': version > ',versionList{idx},' ']; %#ok<AGROW>
            if ( idx < length(compilerList))
                CompilerListString = [CompilerListString,' OR ']; %#ok<AGROW>
            end
        end
    end

    % error-out if selected compiler is not supported
    if(~validSelectedCompiler)
        error('Selected "%s" compiler is invalid. Type the command ''mex -setup'' and select "%s".', selectedCompiler.Name, CompilerListString);
    end
end
