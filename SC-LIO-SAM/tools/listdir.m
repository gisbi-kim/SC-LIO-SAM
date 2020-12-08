function [files] = listdir(dir_path)

files = dir(dir_path); 
files(1:2) = []; % remove . and .. 
files = {files(:).name};

end

