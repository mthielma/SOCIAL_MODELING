function add_function_paths()
try
    functionname='msfm.m';
    functiondir=which(functionname);
    functiondir=functiondir(1:end-length(functionname));
    addpath([functiondir '/functions'])
    addpath([functiondir '/shortestpath'])
catch me
    disp(me.message);
end